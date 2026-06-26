from robot.control.json_motion_sender import JsonMotionSender
from robot.control.actuator_mapping import ActuatorTarget


class FakeSerialController:
    def __init__(self):
        self.commands = []

    def send_command(self, command: dict) -> None:
        self.commands.append(command)
        print(command)


def main():
    fake_serial = FakeSerialController()
    sender = JsonMotionSender(fake_serial)

    target = ActuatorTarget(
        z_delta_m=0.010,
        z_revolutions=5.0,
        z_direction=1,
        z_duration_s=6.667,
        servo2_deg=45.0,
        servo3_deg=90.0,
    )

    sender.send_actuator_target(target)
    sender.aspirate()
    sender.dispense()
    sender.home()
    sender.stop()
    sender.estop()


if __name__ == "__main__":
    main()