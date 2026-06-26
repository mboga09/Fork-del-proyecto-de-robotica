from __future__ import annotations

import argparse
import json
import time
from pathlib import Path
from typing import Any

import serial
import yaml


BAUD_RATE = 115200
SERVO2_AT_Q2_ZERO_DEG = 45.0
SERVO3_AT_Q3_ZERO_DEG = 90.0
Q2_MIN_DEG = -30.0
Q2_MAX_DEG = 30.0
Q3_MIN_DEG = -45.0
Q3_MAX_DEG = 45.0
TOOL_HOME_DEG = 90.0
TOOL_ASPIRATE_DEG = 180.0
TOOL_DISPENSE_DEG = 0.0

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_WORKSPACE_CONFIG = PROJECT_ROOT / "python" / "config" / "workspace_config.yaml"


def load_workspace_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as fh:
        return yaml.safe_load(fh)


def servo_from_joint(theta2_deg: float, theta3_deg: float) -> tuple[float, float]:
    servo2 = SERVO2_AT_Q2_ZERO_DEG + theta2_deg
    servo3 = SERVO3_AT_Q3_ZERO_DEG + theta3_deg

    if not 0.0 <= servo2 <= 180.0:
        raise ValueError(f"servo2 fuera de rango: {servo2:.3f} deg para q2={theta2_deg:.3f} deg")

    if not 0.0 <= servo3 <= 180.0:
        raise ValueError(f"servo3 fuera de rango: {servo3:.3f} deg para q3={theta3_deg:.3f} deg")

    return servo2, servo3


def move_command(name: str, theta2_deg: float, theta3_deg: float, z_dir: int = 0, z_time_s: float = 0.0) -> dict[str, Any]:
    servo2, servo3 = servo_from_joint(theta2_deg, theta3_deg)
    return {
        "cmd": "MOVE_ACT",
        "name": name,
        "z_dir": z_dir,
        "z_time_s": round(float(z_time_s), 3),
        "s2_deg": round(servo2, 3),
        "s3_deg": round(servo3, 3),
    }


def tool_move_command(name: str, tool_deg: float) -> dict[str, Any]:
    if not 0.0 <= tool_deg <= 180.0:
        raise ValueError(f"tool_deg fuera de rango: {tool_deg:.3f}")
    return {"cmd": "TOOL_MOVE", "name": name, "tool_deg": round(tool_deg, 3)}


def q_entry_to_move(name: str, q_entry: dict[str, Any]) -> dict[str, Any]:
    return move_command(
        name=name,
        theta2_deg=float(q_entry["theta2_deg"]),
        theta3_deg=float(q_entry["theta3_deg"]),
    )


def build_sequence(config: dict[str, Any], include_tool: bool, jog_z: bool) -> list[dict[str, Any]]:
    sequence: list[dict[str, Any]] = []

    sequence.append({"cmd": "PING", "name": "PING"})
    sequence.append({"cmd": "ARM_TEST", "name": "ARM_TEST"})

    q_safe = config["q_safe"]
    sequence.append(q_entry_to_move("SAFE_START", q_safe))

    sequence.extend(
        [
            move_command("Q2_MIN", Q2_MIN_DEG, 0.0),
            move_command("Q2_ZERO", 0.0, 0.0),
            move_command("Q2_MAX", Q2_MAX_DEG, 0.0),
            move_command("Q2_ZERO_RETURN", 0.0, 0.0),
            move_command("Q3_MIN", 0.0, Q3_MIN_DEG),
            move_command("Q3_ZERO", 0.0, 0.0),
            move_command("Q3_MAX", 0.0, Q3_MAX_DEG),
            move_command("Q3_ZERO_RETURN", 0.0, 0.0),
        ]
    )

    if jog_z:
        sequence.append(move_command("Z_JOG_FORWARD_SHORT", 0.0, 0.0, z_dir=1, z_time_s=0.25))
        sequence.append(move_command("Z_JOG_REVERSE_SHORT", 0.0, 0.0, z_dir=-1, z_time_s=0.25))

    sequence.append(q_entry_to_move("SOURCE_APPROACH", config["source"]["approach_q"]))
    sequence.append(q_entry_to_move("SAFE_AFTER_SOURCE", q_safe))

    for well_id in ["A1", "A2", "A3", "B1", "B2", "B3"]:
        sequence.append(q_entry_to_move(f"WELL_{well_id}_APPROACH", config["wells"][well_id]["approach_q"]))

    sequence.append(q_entry_to_move("SAFE_END", q_safe))

    if include_tool:
        sequence.append(tool_move_command("TOOL_HOME_START", TOOL_HOME_DEG))
        sequence.append({"cmd": "TOOL_ASPIRATE", "name": "TOOL_ASPIRATE_180"})
        sequence.append({"cmd": "TOOL_DISPENSE", "name": "TOOL_DISPENSE_0"})
        sequence.append({"cmd": "TOOL_HOME", "name": "TOOL_HOME_END"})

    return sequence


def terminal_statuses_for(command: dict[str, Any]) -> set[str]:
    cmd = command["cmd"]
    if cmd == "PING":
        return {"READY"}
    if cmd == "ARM_TEST":
        return {"HOMED"}
    if cmd == "CONFIG":
        return {"CONFIG"}
    if cmd in {"MOVE_ACT", "TOOL_ASPIRATE", "TOOL_DISPENSE", "TOOL_HOME", "TOOL_MOVE"}:
        return {"IDLE", "STOPPED", "ESTOPPED", "ERROR"}
    return {"IDLE", "STOPPED", "ESTOPPED", "ERROR"}


def read_json_line(port: serial.Serial, timeout_s: float) -> dict[str, Any] | None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        raw = port.readline()
        if not raw:
            continue

        text = raw.decode("utf-8", errors="replace").strip()
        if not text:
            continue

        print(f"RX {text}")
        try:
            return json.loads(text)
        except json.JSONDecodeError:
            print("RX line was not JSON; ignoring")

    return None


def send_and_wait(port: serial.Serial, command: dict[str, Any], timeout_s: float) -> None:
    payload = json.dumps(command, separators=(",", ":"), ensure_ascii=False)
    print(f"TX {payload}")
    port.write((payload + "\n").encode("utf-8"))
    port.flush()

    ack_seen = False
    terminal_statuses = terminal_statuses_for(command)
    deadline = time.monotonic() + timeout_s

    while time.monotonic() < deadline:
        message = read_json_line(port, timeout_s=0.25)
        if message is None:
            continue

        msg_type = message.get("type")
        status = message.get("status")
        cmd = message.get("cmd")

        if msg_type == "ack" and cmd == command["cmd"]:
            ack_seen = True
            if not message.get("ok", False):
                raise RuntimeError(f"Command rejected: {message}")

        if msg_type == "error":
            raise RuntimeError(f"Firmware error: {message}")

        if status in terminal_statuses:
            if status in {"ERROR", "ESTOPPED"}:
                raise RuntimeError(f"Terminal error status: {message}")
            if ack_seen or command["cmd"] == "PING":
                return

    raise TimeoutError(f"Timeout waiting for {command['cmd']} terminal status")


def main() -> None:
    parser = argparse.ArgumentParser(description="Send a safe ESP32 hardware test sequence over JSON serial.")
    parser.add_argument("--port", required=True, help="Serial port, for example COM8 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=BAUD_RATE)
    parser.add_argument("--config", type=Path, default=DEFAULT_WORKSPACE_CONFIG)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--dry-run", action="store_true", help="Print commands without opening serial")
    parser.add_argument("--include-tool", action="store_true", help="Move tool servo to home, aspirate, dispense, home")
    parser.add_argument("--jog-z", action="store_true", help="Run two very short q1 jogs of 0.25 s each")
    args = parser.parse_args()

    config = load_workspace_config(args.config)
    sequence = build_sequence(config=config, include_tool=args.include_tool, jog_z=args.jog_z)

    if args.dry_run:
        for command in sequence:
            print(json.dumps(command, ensure_ascii=False))
        return

    with serial.Serial(args.port, args.baud, timeout=0.1) as port:
        time.sleep(2.0)
        while port.in_waiting:
            read_json_line(port, timeout_s=0.1)

        for command in sequence:
            send_and_wait(port, command, timeout_s=args.timeout)

    print("Hardware test sequence completed.")


if __name__ == "__main__":
    main()
