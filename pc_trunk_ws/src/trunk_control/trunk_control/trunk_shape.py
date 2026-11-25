"""trunk_shape: shows in RViz the shape that the three tendons give the trunk.

Subscribes
    trunk/motor_steps  std_msgs/Int32MultiArray  position of each motor [steps]
Publishes
    joint_states       sensor_msgs/JointState    every bend and squeeze joint in the model

All three tendons end at the last (red) ring. Pulling them bends the trunk
as one smooth arc, and the part they pull in common also shortens it.

Matching the real robot: the ideal model turns every millimetre of tendon into
bend, but the real trunk loses part of it (tendon stretch, friction, the lattice
squeezing). So the bend is scaled so that ONE motor at full_steps gives exactly
max_bend_deg, and no combination of motors ever bends it further than that.
Both can be changed while it runs, to calibrate against the camera:
    ros2 param set /trunk_shape full_steps 800

Slack: each tendon is loose until its motor reaches slack_steps (the start
position where it becomes just tight). Below that the trunk stays straight;
only the steps above it bend the trunk.
    ros2 param set /trunk_shape slack_steps "[374, 390, 504]"
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

UNITS = ('unit1', 'unit2', 'unit3')      # must match the names in the URDF
SLICES_PER_UNIT = 8


def tendons_to_shape(pulls, radius, angles):
    """Forward kinematics: three tendon pulls [m] -> (bx, by, compression).

    pulls[i] > 0 means tendon i was wound in, < 0 means it was let out.
    Tendons cannot stretch, and the trunk cannot get longer than its rest length.
    """
    ux = [math.cos(a) for a in angles]
    uy = [math.sin(a) for a in angles]
    mean = sum(pulls) / 3.0

    # Case 1: the tendons pulled in more than they let out in total.
    # All three stay tight: the shared part shortens the trunk,
    # the differences between them bend it.
    if mean >= 0.0:
        k = 2.0 / (3.0 * radius)
        bx = k * sum(p * x for p, x in zip(pulls, ux))
        by = k * sum(p * y for p, y in zip(pulls, uy))
        return bx, by, mean

    # Case 2: more was let out than pulled in. The trunk keeps its full length,
    # some tendons go slack, and it takes the smallest bend the tight ones allow.
    def fits(bx, by):
        return all(radius * (bx * x + by * y) >= p - 1e-12
                   for p, x, y in zip(pulls, ux, uy))

    options = [(0.0, 0.0)]
    for i in range(3):                       # one tendon tight
        if pulls[i] > 0.0:
            s = pulls[i] / radius
            options.append((s * ux[i], s * uy[i]))
    for i, j in ((0, 1), (0, 2), (1, 2)):    # two tendons tight
        det = ux[i] * uy[j] - ux[j] * uy[i]
        bx = (pulls[i] * uy[j] - pulls[j] * uy[i]) / (radius * det)
        by = (ux[i] * pulls[j] - ux[j] * pulls[i]) / (radius * det)
        options.append((bx, by))

    bx, by = min((o for o in options if fits(*o)), key=lambda o: math.hypot(*o))
    return bx, by, 0.0


def hinge_angles(bx, by, hinges):
    """Angles for the two hinges of every slice, sharing the bend equally."""
    theta = math.hypot(bx, by)
    phi = math.atan2(by, bx)
    alpha = theta / hinges                       # each slice takes an equal share
    about_x = math.atan2(-math.sin(phi) * math.sin(alpha), math.cos(alpha))
    about_y = math.asin(math.cos(phi) * math.sin(alpha))
    return about_x, about_y


class TrunkShape(Node):

    def __init__(self):
        super().__init__('trunk_shape')

        # --- settings ---
        self.tendon_radius = self.declare_parameter('tendon_radius', 0.0194).value
        angles_deg = self.declare_parameter('tendon_angles_deg', [90.0, 210.0, 330.0]).value
        self.tendon_angles = [math.radians(a) for a in angles_deg]
        spool_radius = self.declare_parameter('spool_radius', 0.010).value
        steps_per_rev = self.declare_parameter('steps_per_rev', 200).value
        self.directions = self.declare_parameter('motor_directions', [1, 1, 1]).value
        self.max_compression = self.declare_parameter('max_compression', 0.05).value
        # Real robot: one motor at full_steps bends the trunk max_bend_deg, never more.
        self.max_bend_deg = self.declare_parameter('max_bend_deg', 90.0).value
        self.full_steps = self.declare_parameter('full_steps', 800).value
        # Start position of each motor: the tendon is just tight here (673, 702, 907 deg on the sliders).
        self.slack_steps = list(self.declare_parameter('slack_steps', [374, 390, 504]).value)
        rate = self.declare_parameter('publish_rate', 30.0).value

        # One turn of the spool winds in one circumference of tendon.
        self.metres_per_step = 2.0 * math.pi * spool_radius / steps_per_rev
        self.update_gain()
        self.add_on_set_parameters_callback(self.on_params)

        # The joint names, in the same pattern as the URDF: 3 per slice.
        self.joint_names = []
        for unit in UNITS:
            for i in range(1, SLICES_PER_UNIT + 1):
                self.joint_names.append(f'{unit}_seg{i}_squeeze')
                self.joint_names.append(f'{unit}_seg{i}_bend_x')
                self.joint_names.append(f'{unit}_seg{i}_bend_y')
        self.slices = len(UNITS) * SLICES_PER_UNIT

        self.steps = [0, 0, 0]          # latest motor positions

        # --- ROS interfaces ---
        self.create_subscription(Int32MultiArray, 'trunk/motor_steps', self.on_steps, 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / rate, self.on_timer)

    def update_gain(self):
        """Scale factor: the ideal bend of one tendon pulled full_steps, brought down to max_bend_deg."""
        self.max_bend = math.radians(self.max_bend_deg)
        full_pull = self.full_steps * self.metres_per_step
        ideal_full_bend = 2.0 * full_pull / (3.0 * self.tendon_radius)
        self.bend_gain = self.max_bend / ideal_full_bend
        self.get_logger().info(
            f'One motor at {self.full_steps} steps above its start bends {self.max_bend_deg:.0f} deg '
            f'(bend gain {self.bend_gain:.3f})')

    def on_params(self, params):
        """Runs when a setting is changed with 'ros2 param set' while the node runs."""
        for p in params:
            if p.name == 'full_steps':
                if p.value <= 0:
                    return SetParametersResult(successful=False, reason='full_steps must be > 0')
                self.full_steps = p.value
            elif p.name == 'slack_steps':
                if len(p.value) != 3:
                    return SetParametersResult(successful=False, reason='slack_steps needs 3 numbers')
                self.slack_steps = list(p.value)
            elif p.name == 'max_bend_deg':
                if p.value <= 0:
                    return SetParametersResult(successful=False, reason='max_bend_deg must be > 0')
                self.max_bend_deg = p.value
        self.update_gain()
        return SetParametersResult(successful=True)

    def on_steps(self, msg):
        if len(msg.data) == 3:
            self.steps = list(msg.data)

    def on_timer(self):
        # motor steps -> tendon pulls [m] -> bend and shortening -> joint values
        # Only the steps above each tendon's start position pull it; below that it is slack.
        pulls = [max(0, s * d - slack) * self.metres_per_step
                 for s, d, slack in zip(self.steps, self.directions, self.slack_steps)]
        bx, by, compression = tendons_to_shape(pulls, self.tendon_radius, self.tendon_angles)
        bx *= self.bend_gain                             # match the real robot
        by *= self.bend_gain
        bend = math.hypot(bx, by)
        if bend > self.max_bend:                         # never bend past the real maximum
            bx *= self.max_bend / bend
            by *= self.max_bend / bend
        compression = min(compression, self.max_compression)
        about_x, about_y = hinge_angles(bx, by, self.slices)
        squeeze = -compression / self.slices            # each slice moves closer by an equal share

        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = self.joint_names
        joints.position = [squeeze, about_x, about_y] * self.slices
        self.pub_joints.publish(joints)


def main(args=None):
    rclpy.init(args=args)
    node = TrunkShape()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()