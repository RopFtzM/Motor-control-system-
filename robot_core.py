import json
import os
import time
import threading
import serial


FINGER_KEYS = {
    "a": "index",   # 食指
    "s": "middle",  # 中指
    "d": "ring",    # 无名指
    "f": "pinky",   # 小指
}


class PIDController:
    def __init__(self, kp=0.05, ki=0.0, kd=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.last_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0

    def update(self, error, dt):
        self.integral += error * dt
        derivative = 0.0 if dt <= 0 else (error - self.last_error) / dt
        self.last_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class BoardController:
    """
    单块板控制器：
    - 打开串口
    - 接收 P:pos1,pos2,pos3,pos4
    - 根据 target 做 PID
    - 发送 C:p1,p2,p3,p4
    兼容旧接口：
    - connect() 返回 bool
    - get_pos()
    - update_pid_params()
    """

    def __init__(self, name, port=None, logger=print, baudrate=115200):
        self.name = name
        self.port = port
        self.logger = logger
        self.baudrate = baudrate

        self.ser = None
        self.running = False
        self.thread = None

        self.pos = [0, 0, 0, 0]
        self.target = [0, 0, 0, 0]
        self.pid = [PIDController() for _ in range(4)]

        # 编码器方向修正：默认把每对手指中的“后/下”电机翻转。
        # 若某一路方向仍不对，可按实测改成 1 或 -1。
        # 板子A: [小上, 小后, 无上, 无后]
        # 板子B: [中上, 中后, 食上, 食后]
        self.enc_signs = [-1, -1, -1, -1]

        self.deadzone = 30
        self.min_pwm = 0

        self.lock = threading.Lock()

    def connect(self, port=None):
        try:
            if port is not None:
                self.port = port
            if not self.port:
                raise ValueError(f"{self.name} 未设置串口")

            if self.ser is not None and self.running:
                self.logger(f"[{self.name}] 串口已连接")
                return True

            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
            self.running = True
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            self.logger(f"[{self.name}] 已连接 {self.port}")
            return True
        except Exception as e:
            self.logger(f"[{self.name}] 连接失败: {e}")
            self.running = False
            if self.ser is not None:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
            return False

    def disconnect(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

        if self.ser is not None:
            try:
                self.stop_all()
                time.sleep(0.05)
                self.ser.close()
            except Exception:
                pass
            self.ser = None

        self.logger(f"[{self.name}] 已断开")
        return True

    def stop_all(self):
        self.send_pwm([0, 0, 0, 0])

    def reset_encoder(self):
        if self.ser is None:
            return
        self.ser.write(b"R\n")
        self.logger(f"[{self.name}] 已发送复位命令 R")

    def set_target(self, motor_idx, value):
        with self.lock:
            self.target[motor_idx] = int(value)

    def set_targets(self, values):
        if len(values) != 4:
            raise ValueError("values 长度必须为 4")
        with self.lock:
            self.target = [int(v) for v in values]

    def get_position(self, motor_idx):
        with self.lock:
            return self.pos[motor_idx]

    # 旧接口别名
    def get_pos(self, motor_idx):
        return self.get_position(motor_idx)

    def get_positions(self):
        with self.lock:
            return list(self.pos)

    def update_pid_params(self, kp, ki, kd, deadzone=None, min_pwm=None):
        for c in self.pid:
            c.kp = float(kp)
            c.ki = float(ki)
            c.kd = float(kd)
        if deadzone is not None:
            self.deadzone = int(deadzone)
        if min_pwm is not None:
            self.min_pwm = int(min_pwm)
        self.logger(
            f"[{self.name}] PID更新 kp={kp:.3f} ki={ki:.3f} kd={kd:.3f} deadzone={self.deadzone} min_pwm={self.min_pwm}"
        )

    def send_pwm(self, pwm_list):
        if self.ser is None:
            return
        pwm_list = [int(x) for x in pwm_list]
        cmd = "C:{},{},{},{}\n".format(*pwm_list)
        self.ser.write(cmd.encode("utf-8"))

    def _read_position_line(self, line):
        if not line.startswith("P:"):
            return
        body = line[2:].strip()
        parts = body.split(",")
        if len(parts) != 4:
            return
        try:
            vals = [int(x) for x in parts]
        except ValueError:
            return

        vals = [vals[i] * self.enc_signs[i] for i in range(4)]

        with self.lock:
            self.pos = vals

    def _apply_deadzone_min_pwm(self, u):
        if abs(u) <= self.deadzone:
            return 0
        if u > 0:
            return max(u, self.min_pwm)
        return min(u, -self.min_pwm)

    def _loop(self):
        last_t = time.time()

        while self.running and self.ser is not None:
            try:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self._read_position_line(line)

                now = time.time()
                dt = now - last_t
                last_t = now

                with self.lock:
                    errors = [self.target[i] - self.pos[i] for i in range(4)]

                pwm = []
                for i in range(4):
                    u = self.pid[i].update(errors[i], dt)
                    u = int(max(min(u, 255), -255))
                    u = self._apply_deadzone_min_pwm(u)
                    pwm.append(u)

                self.send_pwm(pwm)
                time.sleep(0.03)

            except Exception as e:
                self.logger(f"[{self.name}] 循环异常: {e}")
                time.sleep(0.1)


class RobotController:
    """
    高层控制器：
    - 管两块板
    - 管手指映射
    - 管 press / rest
    - 管 preset
    兼容旧接口：
    - __init__(log_func=...)
    - connect_all() 返回 bool
    - global_reset()
    """

    def __init__(
        self,
        board_a_port="COM11",
        board_b_port="COM8",
        motor_config_path="motor_config.json",
        preset_config_path="preset_config.json",
        logger=None,
        log_func=None,
    ):
        # logger / log_func 二选一，兼容旧代码
        self.logger = logger or log_func or print

        self.board_a = BoardController("板子A(小/无)", board_a_port, logger=self.logger)
        self.board_b = BoardController("板子B(中/食)", board_b_port, logger=self.logger)

        self.finger_map = {
            "pinky": ("board_a", 0, 1),
            "ring": ("board_a", 2, 3),
            "middle": ("board_b", 0, 1),
            "index": ("board_b", 2, 3),
        }

        self.finger_alias = {
            "小指": "pinky",
            "无名指": "ring",
            "中指": "middle",
            "食指": "index",
            "pinky": "pinky",
            "ring": "ring",
            "middle": "middle",
            "index": "index",
        }

        self.motor_config_path = motor_config_path
        self.preset_config_path = preset_config_path

        self.motor_cfg = self.load_motor_config()
        self.preset_cfg = self.load_preset_config()

    def normalize_finger(self, finger: str):
        finger = finger.strip().lower()
        return self.finger_alias.get(finger, finger)

    def load_motor_config(self):
        template = {
            "pinky": {"press": {"up": 2000, "down": -2000}, "rest": {"up": 0, "down": 0}},
            "ring": {"press": {"up": 2000, "down": -2000}, "rest": {"up": 0, "down": 0}},
            "middle": {"press": {"up": 2000, "down": -2000}, "rest": {"up": 0, "down": 0}},
            "index": {"press": {"up": 5000, "down": -5000}, "rest": {"up": 0, "down": 0}},
        }

        if not os.path.exists(self.motor_config_path):
            with open(self.motor_config_path, "w", encoding="utf-8") as f:
                json.dump(template, f, ensure_ascii=False, indent=2)
            return template

        with open(self.motor_config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def load_preset_config(self):
        template = {
            "relax": {
                "pinky": {"up": 0, "down": 0},
                "ring": {"up": 0, "down": 0},
                "middle": {"up": 0, "down": 0},
                "index": {"up": 0, "down": 0},
            },
            "pinch": {
                "index": {"up": 4000, "down": -4000},
                "middle": {"up": 1500, "down": -1500},
            },
        }

        if not os.path.exists(self.preset_config_path):
            with open(self.preset_config_path, "w", encoding="utf-8") as f:
                json.dump(template, f, ensure_ascii=False, indent=2)
            return template

        with open(self.preset_config_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def connect_all(self):
        ok_a = self.board_a.connect()
        ok_b = self.board_b.connect()
        return bool(ok_a and ok_b)

    def disconnect_all(self):
        self.board_a.disconnect()
        self.board_b.disconnect()
        return True

    def global_reset(self):
        self.board_a.reset_encoder()
        self.board_b.reset_encoder()
        for board in (self.board_a, self.board_b):
            board.set_targets([0, 0, 0, 0])
        self.logger("[global] 已执行 RESET")
        return True

    def _get_board_and_motors(self, finger):
        finger = self.normalize_finger(finger)
        if finger not in self.finger_map:
            raise ValueError(f"未知手指: {finger}")
        board_name, up_idx, down_idx = self.finger_map[finger]
        board = getattr(self, board_name)
        return finger, board, up_idx, down_idx

    def set_finger_target(self, finger, up_value, down_value):
        finger, board, up_idx, down_idx = self._get_board_and_motors(finger)
        board.set_target(up_idx, up_value)
        board.set_target(down_idx, down_value)
        self.logger(f"[{finger}] target -> up={up_value}, down={down_value}")

    def motor(self, finger, state):
        finger = self.normalize_finger(finger)
        if finger not in self.motor_cfg:
            raise ValueError(f"motor_config 中无此手指配置: {finger}")
        if state not in self.motor_cfg[finger]:
            raise ValueError(f"motor_config 中无此状态配置: {state}")

        cfg = self.motor_cfg[finger][state]
        self.set_finger_target(finger, cfg["up"], cfg["down"])

    def run_preset(self, preset_name):
        if preset_name not in self.preset_cfg:
            raise ValueError(f"不存在预设: {preset_name}")

        preset = self.preset_cfg[preset_name]
        for finger, cfg in preset.items():
            self.set_finger_target(finger, cfg["up"], cfg["down"])

        self.logger(f"[preset] 已执行 {preset_name}")

    def get_all_positions(self):
        return {
            "board_a": self.board_a.get_positions(),
            "board_b": self.board_b.get_positions(),
        }
