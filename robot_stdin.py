import queue
import threading
import time
from robot_core import RobotController

FINGER_MAP = {
    "2": "index",   # 食指 -> 板子B
    "3": "middle",  # 中指 -> 板子B
    "4": "ring",    # 无名指 -> 板子A
    "5": "pinky",   # 小指 -> 板子A
}

class StdinSequenceRunner:
    def __init__(self):
        self.rc = None

        self.cmd_queue = queue.Queue()
        self.running = False
        self.stop_flag = False
        self.worker = None

        # 控制节奏的三个核心参数
        self.gap = 0.15       # 两个完整动作之间的额外间隔
        self.recovery = 0.35  # 重要：手指从 press 回到 rest 所需的物理回弹时间
        self.tail = 0.2       # 整个序列结束后的收尾等待

        self.board_enabled = {
            "board_a": False,
            "board_b": False,
        }

    def log(self, msg):
        print(msg, flush=True)

    def setup_ports(self):
        while True:
            line = input("请输入两个串口编号（如 4 5；若缺失用 * 占位）: ").strip()
            parts = line.split()

            if len(parts) != 2:
                self.log("格式错误，请输入两个值，例如：4 5 或 * 5")
                continue

            a_raw, b_raw = parts

            try:
                port_a = None if a_raw == "*" else f"COM{int(a_raw)}"
                port_b = None if b_raw == "*" else f"COM{int(b_raw)}"
            except ValueError:
                self.log("格式错误：端口必须是数字或 *")
                continue

            self.rc = RobotController(
                board_a_port=port_a,
                board_b_port=port_b,
                logger=self.log,
            )

            swap_line = input("是否互换板B上的食指/中指上下映射？(y/N): ").strip().lower()
            self.rc.set_board_b_swap(swap_line in {"y", "yes", "1"})

            # 连接板子A
            if port_a is None:
                self.board_enabled["board_a"] = False
                self.log("[板子A] 未启用")
            else:
                try:
                    ok = self.rc.board_a.connect()
                    self.board_enabled["board_a"] = bool(ok)
                    if ok:
                        self.log(f"[板子A] 已启用：{port_a}")
                        # 增加死区到 60 以减少静止时的抖动
                        self.rc.board_a.update_pid_params(0.05, 0.0, 0.001, 60, 0) 
                    else:
                        self.log(f"[板子A] 连接失败，已禁用：{port_a}")
                except Exception as e:
                    self.board_enabled["board_a"] = False
                    self.log(f"[板子A] 连接异常，已禁用：{e}")

            # 连接板子B
            if port_b is None:
                self.board_enabled["board_b"] = False
                self.log("[板子B] 未启用")
            else:
                try:
                    ok = self.rc.board_b.connect()
                    self.board_enabled["board_b"] = bool(ok)
                    if ok:
                        self.log(f"[板子B] 已启用：{port_b}")
                        self.rc.board_b.update_pid_params(0.05, 0.0, 0.001, 60, 0) 
                    else:
                        self.log(f"[板子B] 连接失败，已禁用：{port_b}")
                except Exception as e:
                    self.board_enabled["board_b"] = False
                    self.log(f"[板子B] 连接异常，已禁用：{e}")

            break

    def validate_sequence(self, finger_seq: str, time_seq: str):
        if not finger_seq or not time_seq:
            return False, "两段都不能为空"
        if len(finger_seq) != len(time_seq):
            return False, "前后两段长度必须一致"
        for ch in finger_seq:
            if ch not in "2345":
                return False, f"非法手指编号: {ch}，只允许 2/3/4/5"
        for ch in time_seq:
            if ch not in "0123456789":
                return False, f"非法时长字符: {ch}，只允许 0~9"
        return True, "ok"

    def finger_available(self, finger_name: str):
        if finger_name in ("ring", "pinky"):
            return self.board_enabled["board_a"]
        if finger_name in ("index", "middle"):
            return self.board_enabled["board_b"]
        return False

    def safe_motor(self, finger: str, state: str):
        if not self.finger_available(finger):
            self.log(f"[skip] {finger} 对应板未启用，跳过 {state}")
            return
        self.rc.motor(finger, state)

    def safe_relax(self):
        if self.rc is None: return
        try:
            if self.board_enabled["board_a"]:
                self.rc.set_finger_target("pinky", 0, 0)
                self.rc.set_finger_target("ring", 0, 0)
            if self.board_enabled["board_b"]:
                self.rc.set_finger_target("middle", 0, 0)
                self.rc.set_finger_target("index", 0, 0)
            self.log("[preset] 已执行 relax（仅作用于启用板）")
        except Exception as e:
            self.log(f"[preset] relax 失败: {e}")

    def enqueue_line(self, line: str):
        line = line.strip()
        if not line: return
        low = line.lower()

        if low in {"quit", "exit"}:
            raise KeyboardInterrupt
        if low == "stop":
            self.stop_current_sequence()
            self.log("[stdin] 已请求停止当前序列")
            return
        if low == "relax":
            self.safe_relax()
            return
        if low == "reset":
            if self.board_enabled["board_a"]: self.rc.board_a.reset_encoder()
            if self.board_enabled["board_b"]: self.rc.board_b.reset_encoder()
            self.log("[stdin] 已发送 reset")
            return

        parts = line.split()
        if len(parts) != 2:
            self.log("[stdin] 格式错误，应为：手指序列 时长序列")
            return

        finger_seq, time_seq = parts
        ok, msg = self.validate_sequence(finger_seq, time_seq)
        if not ok:
            self.log(f"[stdin] 输入非法：{msg}")
            return

        self.cmd_queue.put((finger_seq, time_seq))
        self.log(f"[stdin] 已入队：{finger_seq} {time_seq}")

        if not self.running:
            self.start_worker()

    def start_worker(self):
        if self.running: return
        self.running = True
        self.stop_flag = False
        self.worker = threading.Thread(target=self.sequence_worker, daemon=True)
        self.worker.start()

    def stop_current_sequence(self):
        self.stop_flag = True

    def sleep_with_stop(self, duration: float):
        t0 = time.time()
        while time.time() - t0 < duration:
            if self.stop_flag: return False
            time.sleep(0.02)
        return True

    def sequence_worker(self):
        """
        核心逻辑：
        Press -> Sleep(duration) -> Rest -> Sleep(recovery) -> Sleep(gap)
        """
        try:
            while not self.cmd_queue.empty():
                finger_seq, time_seq = self.cmd_queue.get()
                self.log(f"[seq] 开始执行：{finger_seq} {time_seq}")

                for fch, tch in zip(finger_seq, time_seq):
                    if self.stop_flag: break

                    finger = FINGER_MAP[fch]
                    duration = int(tch)

                    # 1. 动作阶段：按下
                    self.log(f"[seq] {finger} -> press ({duration}s)")
                    self.safe_motor(finger, "press")
                    if not self.sleep_with_stop(duration): break

                    # 2. 复位阶段：松开
                    self.safe_motor(finger, "rest")
                    self.log(f"[seq] {finger} -> rest (等待回弹)")
                    
                    # 重要：即使 duration 结束，也必须等待手指物理弹回初始位置
                    # 否则连续按同一个手指会因为没时间复位而看起来“连在一起”
                    if not self.sleep_with_stop(self.recovery): break

                    # 3. 停顿阶段：动作间的额外间隙
                    if self.gap > 0:
                        if not self.sleep_with_stop(self.gap): break

                # 队列中单个序列执行完后的收尾
                if self.tail > 0:
                    self.sleep_with_stop(self.tail)
                self.safe_relax()

                if self.stop_flag:
                    self.log("[seq] 当前序列已中断")
                    self.stop_flag = False
                else:
                    self.log("[seq] 当前序列完成")

        except Exception as e:
            self.log(f"[seq] 执行异常: {e}")
        finally:
            self.running = False
            self.stop_flag = False

    def run(self):
        self.setup_ports()
        self.log("输入格式：手指序列 时长序列，例如：23232 23222")
        self.log("附加命令：relax / reset / stop / exit")

        try:
            while True:
                line = input("> ")
                self.enqueue_line(line)
        finally:
            try: self.safe_relax()
            except: pass
            try:
                if self.board_enabled["board_a"]: self.rc.board_a.disconnect()
                if self.board_enabled["board_b"]: self.rc.board_b.disconnect()
            except: pass
            self.log("已断开连接。")

def main():
    runner = StdinSequenceRunner()
    try:
        runner.run()
    except KeyboardInterrupt:
        print("\n收到退出指令，程序结束。", flush=True)

if __name__ == "__main__":
    main()
