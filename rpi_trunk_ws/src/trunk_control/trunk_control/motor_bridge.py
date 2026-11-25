"""motor_bridge: the link between ROS 2 and the Arduino Due.

Runs on the Raspberry Pi, where the Due is plugged in (Native USB port).

Subscribes
    trunk/motor_cmd    std_msgs/Int32MultiArray   target position of each motor [steps]
Publishes
    trunk/motor_steps  std_msgs/Int32MultiArray   position reported by the Arduino [steps]

Talks to trunk_due.ino with one text line per message:
    to the Due:    T <s1> <s2> <s3>     move the motors to these step positions
    from the Due:  P <p1> <p2> <p3>     where the motors are now (20 times a second)
                   OK / ERR ...         reply to each command
"""

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class MotorBridge(Node):

    def __init__(self):
        super().__init__('motor_bridge')

        # Settings that can be changed from the command line, with defaults.
        # /dev/trunk_due is the fixed name our udev rule gives the Due.
        port = self.declare_parameter('port', '/dev/trunk_due').value
        self.max_steps = self.declare_parameter('max_steps', 800).value

        # Open the USB serial port. timeout=0 means "never wait for data".
        self.ser = serial.Serial(port, 115200, timeout=0)
        self.rx_buf = b''

        # What this node listens to, what it publishes, and what it does on a clock.
        self.create_subscription(Int32MultiArray, 'trunk/motor_cmd', self.on_cmd, 10)
        self.pub_steps = self.create_publisher(Int32MultiArray, 'trunk/motor_steps', 10)
        self.create_timer(0.02, self.read_feedback)

        self.get_logger().info(f'Connected to the Arduino on {port}')

    def on_cmd(self, msg):
        """Runs every time a message arrives on trunk/motor_cmd."""
        if len(msg.data) != 3:
            self.get_logger().warn('motor_cmd needs exactly 3 numbers')
            return
        steps = [max(-self.max_steps, min(self.max_steps, s)) for s in msg.data]
        line = 'T {} {} {}\n'.format(*steps)        # 'T' = target, as trunk_due.ino expects
        self.ser.write(line.encode('ascii'))

    def read_feedback(self):
        """Runs 50 times a second: collect whatever the Arduino has sent."""
        waiting = self.ser.in_waiting
        if waiting:
            self.rx_buf += self.ser.read(waiting)

        # The buffer may hold several lines, or only part of one.
        while b'\n' in self.rx_buf:
            raw, self.rx_buf = self.rx_buf.split(b'\n', 1)
            parts = raw.decode('ascii', errors='ignore').split()
            if len(parts) == 4 and parts[0] == 'P':
                try:
                    steps = [int(p) for p in parts[1:]]
                except ValueError:
                    continue                      # a damaged line: skip it
                self.pub_steps.publish(Int32MultiArray(data=steps))
            elif parts and parts[0] != 'OK':      # 'OK' after every command: not worth logging
                self.get_logger().info('Arduino says: ' + ' '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = MotorBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()