"""motor_sliders: a small window with one slider per stepper motor.

Publishes
    trunk/motor_cmd   std_msgs/Int32MultiArray   target position of each motor [steps]

Each slider is the motor's angle in degrees from zero. Moving one to the right
winds that motor's tendon in.

Start position: at start_deg each tendon is just tight and the trunk is still
straight. The window moves the motors there when it opens, and the
"Start position" button brings them back. Above it, the trunk bends.
"""

import tkinter as tk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class MotorSliders(Node):

    def __init__(self):
        super().__init__('motor_sliders')

        self.steps_per_rev = self.declare_parameter('steps_per_rev', 200).value
        max_deg = self.declare_parameter('max_deg', 2160.0).value
        # Where each tendon is just tight (trunk straight). Measured on the real robot.
        self.start_deg = list(self.declare_parameter('start_deg', [673.0, 702.0, 907.0]).value)

        self.pub = self.create_publisher(Int32MultiArray, 'trunk/motor_cmd', 10)
        self.last_steps = None

        # --- the window ---
        self.window = tk.Tk()
        self.window.title('Trunk motors')
        self.sliders = []
        for i in range(3):
            slider = tk.Scale(
                self.window, label=f'Motor {i + 1}  [degrees]',
                from_=0.0, to=max_deg, resolution=360.0 / self.steps_per_rev,
                orient=tk.HORIZONTAL, length=600, command=self.on_change)
            slider.pack(padx=12, pady=6)
            self.sliders.append(slider)

        buttons = tk.Frame(self.window)
        buttons.pack(pady=10)
        tk.Button(buttons, text='Start position', command=self.to_start).pack(side=tk.LEFT, padx=8)
        tk.Button(buttons, text='All to zero (slack)', command=self.all_to_zero).pack(side=tk.LEFT, padx=8)

        # Open at the start position, so the tendons are tight from the beginning.
        self.to_start()

    def on_change(self, _value=None):
        """Runs whenever a slider moves: degrees -> steps -> publish."""
        steps = [round(s.get() / 360.0 * self.steps_per_rev) for s in self.sliders]
        if steps != self.last_steps:             # only send when something changed
            self.pub.publish(Int32MultiArray(data=steps))
            self.last_steps = steps

    def to_start(self):
        """Move every motor to its start position (tendon just tight)."""
        for slider, deg in zip(self.sliders, self.start_deg):
            slider.set(deg)
        self.on_change()

    def all_to_zero(self):
        """Back to 0: the tendons go slack."""
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

    # Send the start position again after 1 s, in case the bridge was not connected yet.
    node.window.after(1000, lambda: (setattr(node, 'last_steps', None), node.on_change()))
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