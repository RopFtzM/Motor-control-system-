# robot_console.py
import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox

import serial
import threading
import time
import json
from pathlib import Path


# ================== 1) PID ==================

class PIDController:
    def __init__(self):
        self.kp = 0.05
        self.ki = 0.0
        self.kd = 0.01

        self.deadzone = 30
        self.min_pwm = 0          # 可选：克服静摩擦，建议 10~40
        self.max_integral = 5000

        self.target = 0
        self.current = 0
        self.prev_error = 0
        self.integral = 0

    def compute(self, current_pos: int) -> int:
        self.current = current_pos
        error = self.target - self.current

        if abs(error) < self.deadzone:
            self.integral = 0
            return 0

        self.integral += error
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        if self.integral < -self.max_integral:
            self.integral = -self.max_integral

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * (error - self.prev_error))
        self.prev_error = error

        if output > 255:
            output = 255
        if output < -255:
            output = -255

        if self.min_pwm > 0:
            if 0 < output < self.min_pwm:
                output = self.min_pwm
            elif -self.min_pwm < output < 0:
                output = -self.min_pwm

        return int(output)

    def reset(self):
        self.target = 0
        self.current = 0
        self.prev_error = 0
        self.integral = 0


# ================== 2) Board Controller ==================

class BoardController:
    def __init__(self, name: str, default_port: str, log_func):
        self.name = name
        self.port = default_port
        self.log = log_func

        self.ser = None
        self.running = False

        self.lock = threading.Lock()

        self.pids = [PIDController() for _ in range(4)]
        self.motor_pos = [0, 0, 0, 0]
        self.enc_signs = [-1, -1, -1, -1]  # 如方向不对可改

        self.thread = None

    def connect(self, port: str) -> bool:
        try:
            self.port = port
            self.ser = serial.Serial(port, 115200, timeout=0.02)
            self.running = True
            self.thread = threading.Thread(target=self.control_loop, daemon=True)
            self.thread.start()
            self.log(f"[{self.name}] 连接成功: {port}")
            return True
        except Exception as e:
            self.running = False
            self.ser = None
            self.log(f"[{self.name}] 连接失败: {e}")
            return False

    def disconnect(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception as e:
            self.log(f"[{self.name}] 断开时异常: {e}")
        self.ser = None
        self.log(f"[{self.name}] 断开连接")

    def control_loop(self):
        period = 0.03  # 30ms

        while self.running and self.ser and self.ser.is_open:
            start_time = time.time()

            # 读到最新一帧 P:
            latest_line = None
            try:
                while self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line.startswith("P:"):
                        latest_line = line
            except Exception as e:
                self.log(f"[{self.name}] 串口读取异常: {e}")

            if latest_line:
                try:
                    parts = latest_line[2:].split(",")
                    if len(parts) == 4:
                        with self.lock:
                            for i in range(4):
                                self.motor_pos[i] = int(parts[i]) * self.enc_signs[i]
                except Exception as e:
                    self.log(f"[{self.name}] 位置解析失败: {latest_line} | {e}")

            # 算 PWM
            pwms = [0, 0, 0, 0]
            try:
                for i in range(4):
                    with self.lock:
                        pos = self.motor_pos[i]
                    pwms[i] = self.pids[i].compute(pos)
            except Exception as e:
                self.log(f"[{self.name}] PID计算异常: {e}")

            # 发 PWM
            cmd = f"C:{pwms[0]},{pwms[1]},{pwms[2]},{pwms[3]}\n"
            try:
                self.ser.write(cmd.encode("utf-8"))
            except Exception as e:
                self.log(f"[{self.name}] 串口写失败，已停止控制线程: {e}")
                self.running = False
                try:
                    if self.ser and self.ser.is_open:
                        self.ser.close()
                except Exception:
                    pass
                self.ser = None
                break

            # 周期
            elapsed = time.time() - start_time
            remain = period - elapsed
            if remain > 0:
                time.sleep(remain)

    def set_target(self, motor_idx: int, target: int):
        with self.lock:
            self.pids[motor_idx].target = int(target)

    def get_pos(self, motor_idx: int) -> int:
        with self.lock:
            return int(self.motor_pos[motor_idx])

    def update_pid_params(self, kp, ki, kd, deadzone, min_pwm):
        for pid in self.pids:
            pid.kp = float(kp)
            pid.ki = float(ki)
            pid.kd = float(kd)
            pid.deadzone = int(deadzone)
            pid.min_pwm = int(min_pwm)

    def send_reset(self):
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"R\n")
        except Exception as e:
            self.log(f"[{self.name}] RESET发送失败: {e}")

        with self.lock:
            for pid in self.pids:
                pid.reset()
            self.motor_pos = [0, 0, 0, 0]


# ================== 3) App ==================

class ModernRobotApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("8电机灵巧手控制台（按住模式热键版）")
        self.root.geometry("1200x800")

        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.style.configure("TButton", font=("Segoe UI", 10), padding=5)
        self.style.configure("Accent.TButton", background="#4CAF50", foreground="white",
                             font=("Segoe UI", 10, "bold"))
        self.style.configure("Stop.TButton", background="#F44336", foreground="white",
                             font=("Segoe UI", 12, "bold"))
        self.style.configure("Header.TLabel", font=("Segoe UI", 11, "bold"))

        self.board_A = BoardController("板子A(小/无)", "COM11", self.log)
        self.board_B = BoardController("板子B(中/食)", "COM8", self.log)

        # fingers 顺序固定：0小 1无 2中 3食
        self.fingers = [
            {"name": "小指 (Pinky)",   "board": self.board_A, "ids": [0, 1]},
            {"name": "无名指 (Ring)",  "board": self.board_A, "ids": [2, 3]},
            {"name": "中指 (Middle)",  "board": self.board_B, "ids": [0, 1]},
            {"name": "食指 (Index)",   "board": self.board_B, "ids": [2, 3]},
        ]

        self.ui_entries = []
        self.ui_monitors = []

        self.setup_ui()

        self.load_motor_config("motor_config.json")
        self.update_pid()          # 让后端参数与UI一致
        self.bind_hotkeys()        # 按住模式热键

        self.root.after(50, self.update_monitor_loop)

    # ---------- config ----------

    def load_motor_config(self, path="motor_config.json"):
        """
        motor_config.json 结构示例：
        {
          "0": {"press":{"up":2000,"down":2000}, "rest":{"up":0,"down":0}},
          "1": ...
          "2": ...
          "3": {"press":{"up":5000,"down":-5000}, "rest":{"up":0,"down":0}}
        }
        其中：
        - press = 按键按下时刻的目标位置
        - rest  = 松开时的目标位置
        """
        self.config_path = Path(path)

        if not self.config_path.exists():
            template = {}
            for i in range(len(self.fingers)):
                template[str(i)] = {
                    "press": {"up": 2000, "down": -2000},
                    "rest":  {"up": 0,    "down": 0}
                }
            # 默认示例：食指 press 上5000 下-5000（你可改）
            template["3"]["press"] = {"up": 5000, "down": -5000}

            self.config_path.write_text(
                json.dumps(template, indent=2, ensure_ascii=False),
                encoding="utf-8"
            )

        try:
            self.motor_cfg = json.loads(self.config_path.read_text(encoding="utf-8"))
            self.log(f"[Config] 已加载: {self.config_path}")
        except Exception as e:
            self.motor_cfg = {}
            self.log(f"[Config] 读取失败: {e}")

    # ---------- motor API ----------

    def motor(self, finger, state: str):
        """
        finger: 0~3 或手指名字符串（例如 '食指 (Index)'）
        state: 'press' 或 'rest'
        """
        if state not in ("press", "rest"):
            raise ValueError("state 只能是 'press' 或 'rest'")

        if isinstance(finger, str) and not finger.isdigit():
            name_map = {f["name"]: i for i, f in enumerate(self.fingers)}
            if finger not in name_map:
                raise ValueError(f"未知手指名: {finger}")
            idx = name_map[finger]
        else:
            idx = int(finger)

        cfg = self.motor_cfg.get(str(idx))
        if not cfg:
            raise KeyError(f"motor_config.json 缺少手指 {idx} 的配置")

        up = int(cfg[state]["up"])
        down = int(cfg[state]["down"])

        conf = self.fingers[idx]
        board = conf["board"]
        mid_up, mid_down = conf["ids"][0], conf["ids"][1]

        board.set_target(mid_up, up)
        board.set_target(mid_down, down)

        # 同步 UI 输入框
        ent_up, ent_down = self.ui_entries[idx]
        ent_up.delete(0, tk.END); ent_up.insert(0, str(up))
        ent_down.delete(0, tk.END); ent_down.insert(0, str(down))

        self.log(f"motor({idx}, {state}) -> 上:{up}, 下:{down}")

    # ---------- 按住模式热键 ----------

    def bind_hotkeys(self):
        self.root.focus_set()
        self._held_keys = set()

        for k in ["a", "s", "d", "f"]:
            self.root.bind(f"<KeyPress-{k}>", self.on_key_press)
            self.root.bind(f"<KeyRelease-{k}>", self.on_key_release)

        # 小键盘可选
        for kp in ["KP_2", "KP_3", "KP_4", "KP_5"]:
            self.root.bind(f"<KeyPress-{kp}>", self.on_key_press)
            self.root.bind(f"<KeyRelease-{kp}>", self.on_key_release)

        self.log("按住模式：a=食指 s=中指 d=无名指 f=小指（按住=press，松开=rest）")

    def _key_to_finger_idx(self, key: str):
        # 你要求：2/3/4/5 -> 食/中/无/小
        # fingers: 0小 1无 2中 3食
        keymap = {"a": 3, "s": 2, "d": 1, "f": 0}
        return keymap.get(key)

    def on_key_press(self, event):
        key = event.keysym.replace("KP_", "")  # 'KP_2' -> '2'

        idx = self._key_to_finger_idx(key)
        if idx is None:
            return

        # 防止长按重复 press
        if key in self._held_keys:
            return
        self._held_keys.add(key)

        try:
            self.motor(idx, "press")
        except Exception as e:
            self.log(f"[Hotkey] KeyPress {key} press失败: {e}")

    def on_key_release(self, event):
        key = event.keysym.replace("KP_", "")

        idx = self._key_to_finger_idx(key)
        if idx is None:
            return

        if key in self._held_keys:
            self._held_keys.remove(key)

        try:
            self.motor(idx, "rest")
        except Exception as e:
            self.log(f"[Hotkey] KeyRelease {key} rest失败: {e}")

    # ---------- UI ----------

    def setup_ui(self):
        conn_frame = ttk.LabelFrame(self.root, text=" 🔗 硬件连接 ", padding=10)
        conn_frame.pack(fill=tk.X, padx=10, pady=5)

        self.create_conn_ui(conn_frame, self.board_A, self.board_A.port, 0)
        ttk.Separator(conn_frame, orient=tk.VERTICAL).grid(row=0, column=4, sticky="ns", padx=20)
        self.create_conn_ui(conn_frame, self.board_B, self.board_B.port, 5)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.tab_ops = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_ops, text=" 🕹️ 手指操作台 ")
        self.setup_operations_tab(self.tab_ops)

        self.tab_pid = ttk.Frame(self.notebook)
        self.notebook.add(self.tab_pid, text=" ⚙️ PID & 阈值调优 ")
        self.setup_pid_tab(self.tab_pid)

        bottom_frame = ttk.Frame(self.root)
        bottom_frame.pack(fill=tk.X, padx=10, pady=5)

        ttk.Button(bottom_frame, text="⛔ 全局紧急复位 (RESET)", style="Stop.TButton",
                   command=self.global_reset).pack(fill=tk.X)

        self.txt_log = scrolledtext.ScrolledText(self.root, height=6, bg="#222", fg="#0f0",
                                                 font=("Consolas", 9))
        self.txt_log.pack(fill=tk.X, padx=10, pady=(0, 10))

    def create_conn_ui(self, parent, board: BoardController, port: str, col: int):
        ttk.Label(parent, text=f"{board.name}:").grid(row=0, column=col, padx=5)
        ent = ttk.Entry(parent, width=10)
        ent.insert(0, port)
        ent.grid(row=0, column=col + 1, padx=5)

        lbl = tk.Label(parent, text="●", fg="#ccc", font=("Arial", 16))
        lbl.grid(row=0, column=col + 2)

        btn = ttk.Button(parent, text="连接", command=lambda: self.toggle_conn(board, ent, btn, lbl))
        btn.grid(row=0, column=col + 3)

    def setup_operations_tab(self, parent):
        f = ttk.Frame(parent, padding=10)
        f.pack(fill=tk.BOTH, expand=True)

        headers = ["手指名称", "上电机目标", "下电机目标", "执行操作", "整体微调(-)", "整体微调(+)", "🔴 实时反馈 (Actual) 🟢"]
        for i, h in enumerate(headers):
            ttk.Label(f, text=h, style="Header.TLabel").grid(row=0, column=i, pady=10, padx=10, sticky="w")

        ttk.Separator(f, orient=tk.HORIZONTAL).grid(row=1, column=0, columnspan=7, sticky="ew", pady=(0, 10))

        for i, finger in enumerate(self.fingers):
            row = i + 2
            ttk.Label(f, text=finger["name"], font=("Segoe UI", 11)).grid(
                row=row, column=0, pady=10, padx=10, sticky="w"
            )

            ent_up = ttk.Entry(f, width=10, justify="center", font=("Consolas", 11))
            ent_up.insert(0, "0")
            ent_up.grid(row=row, column=1, padx=10)

            ent_down = ttk.Entry(f, width=10, justify="center", font=("Consolas", 11))
            ent_down.insert(0, "0")
            ent_down.grid(row=row, column=2, padx=10)

            self.ui_entries.append((ent_up, ent_down))

            bf = ttk.Frame(f)
            bf.grid(row=row, column=3, padx=10)
            ttk.Button(bf, text="▶ 执行", style="Accent.TButton", command=lambda x=i: self.action_set(x)).pack(side=tk.LEFT, padx=2)
            ttk.Button(bf, text="⏹ 归零", command=lambda x=i: self.action_zero(x)).pack(side=tk.LEFT, padx=2)

            ttk.Button(f, text="- 100", width=6, command=lambda x=i: self.adjust(x, -100)).grid(row=row, column=4, padx=5)
            ttk.Button(f, text="+ 100", width=6, command=lambda x=i: self.adjust(x, 100)).grid(row=row, column=5, padx=5)

            monitor_frame = tk.Frame(f, bg="#000000", padx=10, pady=5, relief="sunken", bd=2)
            monitor_frame.grid(row=row, column=6, padx=10, sticky="ew")
            lbl_mon = tk.Label(monitor_frame, text="等待连接...", bg="black", fg="#00FF00",
                               font=("Consolas", 12, "bold"), width=25)
            lbl_mon.pack()
            self.ui_monitors.append(lbl_mon)

            if i < len(self.fingers) - 1:
                ttk.Separator(f, orient=tk.HORIZONTAL).grid(row=row + 1, column=0, columnspan=7, sticky="ew", pady=5)

    def setup_pid_tab(self, parent):
        f = ttk.Frame(parent, padding=20)
        f.pack(fill=tk.BOTH, expand=True)

        ttk.Label(f, text="PID 参数 & 阈值设定", font=("Segoe UI", 14)).pack(pady=10)
        gf = ttk.Frame(f)
        gf.pack()

        self.create_slider(gf, "Kp (比例)", 0, 0.5, 0.05, 0, self.update_pid)
        self.create_slider(gf, "Ki (积分)", 0, 0.05, 0.00, 1, self.update_pid)
        self.create_slider(gf, "Kd (微分)", 0, 0.5, 0.01, 2, self.update_pid)
        self.create_slider(gf, "Deadzone (死区)", 0, 200, 30, 3, self.update_pid)
        self.create_slider(gf, "MinPWM (最小有效PWM)", 0, 80, 0, 4, self.update_pid)

    def create_slider(self, parent, title, min_v, max_v, def_v, col, cb):
        lf = ttk.LabelFrame(parent, text=f" {title} ", padding=10)
        lf.grid(row=0, column=col, padx=15)

        var = tk.DoubleVar(value=def_v)

        def fmt_value(v):
            if "Deadzone" in title or "MinPWM" in title:
                return f"{float(v):.0f}"
            return f"{float(v):.3f}"

        lbl = ttk.Label(lf, text=fmt_value(def_v), font=("Consolas", 12), foreground="blue")
        lbl.pack()

        def on_chg(v):
            lbl.config(text=fmt_value(v))
            cb()

        s = ttk.Scale(lf, from_=min_v, to=max_v, variable=var, orient=tk.VERTICAL, length=200, command=on_chg)
        s.pack()

        setattr(self, f"scale_{title.split()[0]}", s)

    # ---------- Loop & Actions ----------

    def update_monitor_loop(self):
        deadzone_val = int(self.scale_Deadzone.get())

        for i, finger in enumerate(self.fingers):
            board = finger["board"]
            ids = finger["ids"]
            label = self.ui_monitors[i]
            ent_up, ent_down = self.ui_entries[i]

            if not board.running:
                label.config(text="未连接", fg="#555")
                continue

            act_up = board.get_pos(ids[0])
            act_down = board.get_pos(ids[1])

            try:
                tgt_up = int(ent_up.get())
                tgt_down = int(ent_down.get())
            except Exception:
                tgt_up, tgt_down = 0, 0

            max_err = max(abs(tgt_up - act_up), abs(tgt_down - act_down))
            label.config(text=f"上:{act_up} | 下:{act_down}")

            if max_err <= deadzone_val:
                label.config(fg="#00FF00")
            elif max_err < deadzone_val * 2:
                label.config(fg="#FFFF00")
            else:
                label.config(fg="#FF5555")

        self.root.after(50, self.update_monitor_loop)

    def toggle_conn(self, board: BoardController, ent: ttk.Entry, btn: ttk.Button, lbl: tk.Label):
        if not board.running:
            if board.connect(ent.get()):
                btn.config(text="断开", style="Stop.TButton")
                lbl.config(fg="#0f0")
            else:
                messagebox.showerror("Error", "连接失败")
        else:
            board.disconnect()
            btn.config(text="连接", style="TButton")
            lbl.config(fg="#ccc")

    def action_set(self, idx: int):
        try:
            ent_up, ent_down = self.ui_entries[idx]
            val_up = int(ent_up.get())
            val_down = int(ent_down.get())

            conf = self.fingers[idx]
            conf["board"].set_target(conf["ids"][0], val_up)
            conf["board"].set_target(conf["ids"][1], val_down)

            self.log(f"CMD: {conf['name']} -> 上:{val_up}, 下:{val_down}")
        except Exception as e:
            self.log(f"[UI] action_set错误: {e}")

    def action_zero(self, idx: int):
        ent_up, ent_down = self.ui_entries[idx]
        ent_up.delete(0, tk.END); ent_up.insert(0, "0")
        ent_down.delete(0, tk.END); ent_down.insert(0, "0")
        self.action_set(idx)

    def adjust(self, idx: int, delta: int):
        try:
            ent_up, ent_down = self.ui_entries[idx]
            cur_up = int(ent_up.get())
            cur_down = int(ent_down.get())

            ent_up.delete(0, tk.END); ent_up.insert(0, str(cur_up + delta))
            ent_down.delete(0, tk.END); ent_down.insert(0, str(cur_down + delta))

            self.action_set(idx)
        except Exception as e:
            self.log(f"[UI] adjust错误: {e}")

    def update_pid(self):
        kp = float(self.scale_Kp.get())
        ki = float(self.scale_Ki.get())
        kd = float(self.scale_Kd.get())
        dz = int(self.scale_Deadzone.get())
        mp = int(self.scale_MinPWM.get())

        self.board_A.update_pid_params(kp, ki, kd, dz, mp)
        self.board_B.update_pid_params(kp, ki, kd, dz, mp)

    def global_reset(self):
        self.board_A.send_reset()
        self.board_B.send_reset()

        for i in range(len(self.fingers)):
            ent_up, ent_down = self.ui_entries[i]
            ent_up.delete(0, tk.END); ent_up.insert(0, "0")
            ent_down.delete(0, tk.END); ent_down.insert(0, "0")

        self.log("!!! 全局复位 !!!")

    def log(self, msg: str):
        try:
            self.txt_log.insert(tk.END, msg + "\n")
            self.txt_log.see(tk.END)
        except Exception:
            pass


if __name__ == "__main__":
    root = tk.Tk()
    app = ModernRobotApp(root)
    root.mainloop()
