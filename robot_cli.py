import argparse
import time

from robot_core import RobotController

DIGIT_TO_FINGER = {
    "2": "index",
    "3": "middle",
    "4": "ring",
    "5": "pinky",
}

FINGER_TO_CN = {
    "index": "食指",
    "middle": "中指",
    "ring": "无名指",
    "pinky": "小指",
}


def build_parser():
    p = argparse.ArgumentParser(
        description=(
            "紧凑动作串控制器。\n"
            "示例: python robot_cli.py 2345 1234\n"
            "表示依次执行 食指1秒、中指2秒、无名指3秒、小指4秒。"
        ),
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("sequence", help="动作序列，只能包含 2/3/4/5；2=食指，3=中指，4=无名指，5=小指")
    p.add_argument("durations", help="与动作序列等长的时长串，每位表示该步演示秒数，范围 0~9")
    p.add_argument("--porta", default="COM11", help="板子A串口")
    p.add_argument("--portb", default="COM8", help="板子B串口")
    p.add_argument("--gap", type=float, default=0.2, help="每一步结束后额外停顿时间（秒）")
    p.add_argument("--tail", type=float, default=0.5, help="全部动作完成后回到 relax 前的停顿时间（秒）")
    return p


def validate_args(sequence: str, durations: str):
    if len(sequence) == 0:
        raise ValueError("sequence 不能为空")
    if len(sequence) != len(durations):
        raise ValueError("前后两段长度必须一致")
    bad_seq = [ch for ch in sequence if ch not in DIGIT_TO_FINGER]
    if bad_seq:
        raise ValueError(f"sequence 中只能出现 2/3/4/5，非法字符: {''.join(bad_seq)}")
    bad_dur = [ch for ch in durations if ch < '0' or ch > '9']
    if bad_dur:
        raise ValueError(f"durations 中只能出现 0~9，非法字符: {''.join(bad_dur)}")



def run_sequence(rc: RobotController, sequence: str, durations: str, gap: float, tail: float):
    print(f"开始执行: {sequence} {durations}")
    for step_idx, (finger_digit, dur_digit) in enumerate(zip(sequence, durations), start=1):
        finger = DIGIT_TO_FINGER[finger_digit]
        finger_cn = FINGER_TO_CN[finger]
        duration = int(dur_digit)

        print(f"[step {step_idx}] {finger_digit}->{finger_cn}, 演示 {duration} 秒")
        rc.motor(finger, "press")
        time.sleep(duration)
        rc.motor(finger, "rest")
        if gap > 0:
            time.sleep(gap)

    if tail > 0:
        time.sleep(tail)
    try:
        rc.run_preset("relax")
    except Exception:
        pass
    print("动作串执行完成")



def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        validate_args(args.sequence, args.durations)
    except ValueError as e:
        parser.error(str(e))

    rc = RobotController(board_a_port=args.porta, board_b_port=args.portb, log_func=print)

    try:
        if not rc.connect_all():
            print("连接失败：请检查串口号和硬件连接")
            return
        time.sleep(0.2)
        run_sequence(rc, args.sequence, args.durations, args.gap, args.tail)
    finally:
        rc.disconnect_all()


if __name__ == "__main__":
    main()
