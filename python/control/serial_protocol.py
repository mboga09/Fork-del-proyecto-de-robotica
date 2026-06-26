"""
Serial JSON protocol definitions for the PRR robotic pipetting project.

Protocol:
    - One JSON object per line.
    - Python/HMI sends commands using the "cmd" field.
    - Firmware responds with messages using the "type" field.

Protocol version: 0.2
"""

from enum import StrEnum


BAUD_RATE = 115200
ENCODING = "utf-8"
LINE_ENDING = "\n"
READ_TIMEOUT_S = 0.1


class Command(StrEnum):
    PING = "PING"
    HOME = "HOME"
    STOP = "STOP"
    ESTOP = "ESTOP"

    MOVE_ACT = "MOVE_ACT"

    TOOL_ASPIRATE = "TOOL_ASPIRATE"
    TOOL_DISPENSE = "TOOL_DISPENSE"


class MessageType(StrEnum):
    ACK = "ack"
    STATUS = "status"
    ERROR = "error"


class RobotStatus(StrEnum):
    READY = "READY"
    IDLE = "IDLE"
    HOMING = "HOMING"
    HOMED = "HOMED"
    MOVING = "MOVING"
    STOPPED = "STOPPED"
    ESTOPPED = "ESTOPPED"
    ERROR = "ERROR"


FIELD_COMMAND = "cmd"
FIELD_TYPE = "type"
FIELD_STATUS = "status"
FIELD_OK = "ok"
FIELD_MESSAGE = "message"

FIELD_Z_DIR = "z_dir"
FIELD_Z_TIME_S = "z_time_s"
FIELD_SERVO_2_DEG = "s2_deg"
FIELD_SERVO_3_DEG = "s3_deg"


def make_command(command: Command) -> dict:
    return {
        FIELD_COMMAND: command.value
    }


def make_ping_command() -> dict:
    return make_command(Command.PING)


def make_home_command() -> dict:
    return make_command(Command.HOME)


def make_stop_command() -> dict:
    return make_command(Command.STOP)


def make_estop_command() -> dict:
    return make_command(Command.ESTOP)


def make_tool_aspirate_command() -> dict:
    return make_command(Command.TOOL_ASPIRATE)


def make_tool_dispense_command() -> dict:
    return make_command(Command.TOOL_DISPENSE)


def make_move_act_command(
    z_dir: int,
    z_time_s: float,
    s2_deg: float,
    s3_deg: float,
) -> dict:
    z_dir = int(z_dir)
    z_time_s = float(z_time_s)
    s2_deg = float(s2_deg)
    s3_deg = float(s3_deg)

    if z_dir not in (-1, 0, 1):
        raise ValueError("z_dir must be -1, 0, or 1.")

    if z_time_s < 0.0:
        raise ValueError("z_time_s must be non-negative.")

    if not 0.0 <= s2_deg <= 180.0:
        raise ValueError("s2_deg must be between 0 and 180 degrees.")

    if not 0.0 <= s3_deg <= 180.0:
        raise ValueError("s3_deg must be between 0 and 180 degrees.")

    return {
        FIELD_COMMAND: Command.MOVE_ACT.value,
        FIELD_Z_DIR: z_dir,
        FIELD_Z_TIME_S: round(z_time_s, 3),
        FIELD_SERVO_2_DEG: round(s2_deg, 2),
        FIELD_SERVO_3_DEG: round(s3_deg, 2),
    }


def is_ack_message(message: dict) -> bool:
    return message.get(FIELD_TYPE) == MessageType.ACK.value


def is_status_message(message: dict) -> bool:
    return message.get(FIELD_TYPE) == MessageType.STATUS.value


def is_error_message(message: dict) -> bool:
    return message.get(FIELD_TYPE) == MessageType.ERROR.value


def is_ok_message(message: dict) -> bool:
    return bool(message.get(FIELD_OK, False))


def get_message_text(message: dict) -> str:
    status = message.get(FIELD_STATUS)
    text = message.get(FIELD_MESSAGE)

    if status and text:
        return f"{status}: {text}"

    if text:
        return text

    return str(message)