import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from subprocess import call

class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.model_name = 'bluerov2'
        """
        Callback function to handle joystick messages.
        This function maps the joystick axis to a Gazebo command.
        """
    def joy_callback(self, joy):
            # Forward and backward
            if joy.axes[1] > 0.1:  # Threshold to ignore small movements
                # Forward
                self.execute_command('forward',joy.axes[1])
            elif joy.axes[1] < -0.1:
                # Backward
                self.execute_command('backward',joy.axes[1])

            # Left and right
            if joy.axes[0] > 0.1:
                # Left
                self.execute_command('left',joy.axes[0])
                
            elif joy.axes[0] < -0.1:
                # Right
                self.execute_command('right',joy.axes[0])

            # Up and down
            if joy.axes[4] > 0.1:
                # Up
                self.execute_command('up',joy.axes[4])
            elif joy.axes[4] < -0.1:
                # Down
                self.execute_command('down',joy.axes[4])

            # Rotations
            if joy.axes[3] < -0.1:  # Assume button index 4 for clockwise rotation
                self.execute_command('clockwise',joy.axes[3])
            if joy.axes[3] > 0.1:  # Assume button index 5 for counterclockwise rotation
                self.execute_command('counterclockwise',joy.axes[3])

            # Stop
            if joy.buttons[0]:  # Assume button index 6 for stop
                self.execute_command('stop',joy.axes[0])
                
    def execute_command(self, command, value):
            cmd_base = "gz topic -t /model/{}/joint/".format(self.model_name)
            scale_factor = 5
            scaled_value = scale_factor * abs(value)  # Scale the joystick input

            commands = {
                'forward': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'"
                ],
                'backward': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'"
                ],
                'left': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'"
                ],
                'right': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'"
                ],
                'clockwise': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'"
                ],
                'counterclockwise': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'"
                ],
                'down': [
                    cmd_base + f"thruster5_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'",
                    cmd_base + f"thruster6_joint/cmd_thrust -m gz.msgs.Double -p 'data: {scaled_value}'"
                ],
                'up': [
                    cmd_base + f"thruster5_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'",
                    cmd_base + f"thruster6_joint/cmd_thrust -m gz.msgs.Double -p 'data: {-scaled_value}'"
                ],
                'stop': [
                    cmd_base + f"thruster1_joint/cmd_thrust -m gz.msgs.Double -p 'data: {0}'",
                    cmd_base + f"thruster2_joint/cmd_thrust -m gz.msgs.Double -p 'data: {0}'",
                    cmd_base + f"thruster3_joint/cmd_thrust -m gz.msgs.Double -p 'data: {0}'",
                    cmd_base + f"thruster4_joint/cmd_thrust -m gz.msgs.Double -p 'data: {0}'",
                    cmd_base + f"thruster5_joint/cmd_thrust -m gz.msgs.Double -p 'data: {0}'",
                    cmd_base + f"thruster6_joint/cmd_thrust -m gz.msgs.Double -p 'data: {0}'"
                ],
            }

            for cmd in commands.get(command, []):
                call(cmd, shell=True)


def main(args=None):
    rclpy.init(args=args)

    joy_teleop = JoyTeleop()

    rclpy.spin(joy_teleop)

    joy_teleop.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
        main()
