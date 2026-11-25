"""motor_sliders: a small window with one slider per stepper motor.

Publishes
    trunk/motor_cmd   std_msgs/Int32MultiArray   target position of each motor [steps]

Each slider is the motor's angle in degrees from zero. Moving one winds that
motor's tendon in (positive) or lets it out (negative).
"""

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class MotorSliders(Node):

    def __init__(self):
        super().__init__('motor_sliders')

        self.steps_per_rev = self.declare_parameter('steps_per_rev', 200).value
        max_deg = self.declare_parameter('max_deg', 180.0).value

        self.pub = self.create_publisher(Int32MultiArray, 'trunk/motor_cmd', 10)
        self.last_steps = [0, 0, 0]

        # --- the window ---
        self.window = tk.Tk()
        self.window.title('Trunk motors')
        self.sliders = []
        for i in range(3):
            slider = tk.Scale(
                self.window, label=f'Motor {i + 1}  [degrees]',
                from_=-max_deg, to=max_deg, resolution=360.0 / self.steps_per_rev,
                orient=tk.HORIZONTAL, length=420, command=self.on_change)
            slider.pack(padx=12, pady=6)
            self.sliders.append(slider)
        tk.Button(self.window, text='All to zero', command=self.all_to_zero).pack(pady=10)

    def on_change(self, _value=None):
        """Runs whenever a slider moves: degrees -> steps -> publish."""
        steps = [round(s.get() / 360.0 * self.steps_per_rev) for s in self.sliders]
        if steps != self.last_steps:             # only send when something changed
            self.pub.publish(Int32MultiArray(data=steps))
            self.last_steps = steps

    def all_to_zero(self):
        for slider in self.sliders:
            slider.set(0)
        self.on_change()


def main(args=None):
    rclpy.init(args=args)
    node = MotorSliders()

    # Tk runs the window; every 50 ms we give ROS a turn as well.
    def give_ros_a_turn():
        if not rclpy.ok():                       # Ctrl+C was pressed
            node.window.destroy()
            return
        rclpy.spin_once(node, timeout_sec=0.0)
        node.window.after(50, give_ros_a_turn)

    node.window.after(50, give_ros_a_turn)
    try:
        node.window.mainloop()                   # returns when the window is closed
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()