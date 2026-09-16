"""One-shot UM982/G90 serial configuration utility."""
import argparse
import re
import time


def _send(ser, command):
    ser.write((command.rstrip("\r\n") + "\r\n").encode("ascii"))
    ser.flush()


def _read(ser, seconds=1.0):
    end = time.monotonic() + seconds
    data = bytearray()
    while time.monotonic() < end:
        n = ser.in_waiting
        if n:
            data.extend(ser.read(n))
        else:
            time.sleep(0.02)
    return data.decode("ascii", errors="replace")


def main(argv=None):
    p = argparse.ArgumentParser(description="Configure and verify Wheeltec G90/UM982 UART output")
    p.add_argument("--port", default="/dev/wheeltec_gnss")
    p.add_argument("--current-baud", type=int, default=115200)
    p.add_argument("--baud", type=int, default=460800, help="new COM baud rate")
    p.add_argument("--output-port", default="COM1")
    p.add_argument("--rate", type=float, default=10.0)
    args = p.parse_args(argv)
    try:
        import serial
    except ImportError:
        p.error("python3-serial is required")

    period = f"{1.0 / max(1.0, min(20.0, args.rate)):.2f}".rstrip("0").rstrip(".")
    commands = ["UNLOGGSV", "UNLOGGSA", "UNLOGRMC"]
    for prefix in ("", f"{args.output_port} "):
        commands += [f"PVTSLNA {prefix}{period}", f"GNHPR {prefix}{period}",
                     f"BESTNAVA {prefix}{period}", f"GNGGA {prefix}{period}"]
    commands.append(f"CONFIG {args.output_port} {args.baud}")

    print(f"连接 {args.port} @ {args.current_baud}，发送 G90 配置…")
    with serial.Serial(args.port, args.current_baud, timeout=0.1) as ser:
        for command in commands:
            _send(ser, command)
            time.sleep(0.05)
    # CONFIG takes effect immediately; reopen using the new host baud.
    time.sleep(0.5)
    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        _send(ser, "SAVECONFIG")
        time.sleep(0.3)
        _send(ser, "CONFIG")
        response = _read(ser, 1.5)

    baud_ok = bool(re.search(rf"{re.escape(args.output_port)}\s+{args.baud}(?:\s|$)", response, re.I))
    if baud_ok:
        print(f"配置成功：{args.output_port} = {args.baud}，SAVECONFIG 已执行")
        return 0
    print("未在 CONFIG 回显中确认目标波特率。请检查串口是否确实能以新速率通信，")
    print("并手动执行 CONFIG 查询；设备可能不回显查询结果，但 SAVECONFIG 已发送。")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
