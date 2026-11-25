"""trunk_sim: a MuJoCo simulation that stands in for the real robot.

Subscribes
    trunk/motor_cmd    std_msgs/Int32MultiArray   target position of each motor [steps]
Publishes
    trunk/motor_steps  std_msgs/Int32MultiArray   simulated motor positions [steps]
    joint_states       sensor_msgs/JointState     the simulated shape, for RViz

It uses the same topics as motor_bridge, so the sliders and RViz work
unchanged: the simulation simply takes the Arduino's place.
"""

import math

import mujoco
import mujoco.viewer
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

from trunk_sim.model import SLICES_PER_UNIT, UNITS, build_mjcf

TICK = 0.02                 # how often the node runs [s]


class TrunkSim(Node):

    def __init__(self):
        super().__init__('trunk_sim')

        # --- settings: motors and spools, the same as on the real robot ---
        spool_radius = self.declare_parameter('spool_radius', 0.010).value
        steps_per_rev = self.declare_parameter('steps_per_rev', 200).value
        self.directions = self.declare_parameter('motor_directions', [1, 1, 1]).value
        self.pretension = self.declare_parameter('pretension', 0.005).value
        self.max_speed = self.declare_parameter('max_speed', 200.0).value    # steps/s
        show_viewer = self.declare_parameter('viewer', True).value

        # --- settings: how stiff the soft trunk is ---
        model_xml = build_mjcf(
            bend_stiffness=self.declare_parameter('bend_stiffness', 0.05).value,
            squeeze_stiffness=self.declare_parameter('squeeze_stiffness', 300.0).value,
            slice_mass=self.declare_parameter('slice_mass', 0.004).value,
        )

        # --- the simulation ---
        self.model = mujoco.MjModel.from_xml_string(model_xml)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)

        self.metres_per_step = 2.0 * math.pi * spool_radius / steps_per_rev
        self.motor_ids = [self.model.actuator(f'motor{k}').id for k in (1, 2, 3)]
        tendon_ids = [self.model.tendon(f'tendon{k}').id for k in (1, 2, 3)]
        self.rest_length = [float(self.data.ten_length[t]) for t in tendon_ids]
        self.substeps = max(1, round(TICK / self.model.opt.timestep))

        # The joints to report, with the same names as in the URDF.
        self.joint_names = []
        for unit in UNITS:
            for i in range(1, SLICES_PER_UNIT + 1):
                for part in ('squeeze', 'bend_x', 'bend_y'):
                    self.joint_names.append(f'{unit}_seg{i}_{part}')
        self.qpos_index = [self.model.jnt_qposadr[self.model.joint(n).id]
                           for n in self.joint_names]

        # --- state ---
        self.target = [0, 0, 0]         # commanded motor positions [steps]
        self.motor = [0.0, 0.0, 0.0]    # simulated motor positions [steps]

        # --- ROS interfaces ---
        self.create_subscription(Int32MultiArray, 'trunk/motor_cmd', self.on_cmd, 10)
        self.pub_steps = self.create_publisher(Int32MultiArray, 'trunk/motor_steps', 10)
        self.pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(TICK, self.on_timer)

        # --- MuJoCo's own 3D window (optional) ---
        self.viewer = None
        if show_viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

        self.get_logger().info(
            f'Simulation ready: {self.substeps} physics steps per tick, '
            f'tendon rest length {self.rest_length[0] * 1000:.1f} mm')

    def on_cmd(self, msg):
        if len(msg.data) == 3:
            self.target = list(msg.data)

    def on_timer(self):
        # 1. Motors move towards their targets at limited speed, like the real ones.
        max_move = self.max_speed * TICK
        self.motor = [m + max(-max_move, min(max_move, t - m))
                      for m, t in zip(self.motor, self.target)]

        # 2. Each motor sets how long its tendon should be.
        for k in range(3):
            pull = self.motor[k] * self.directions[k] * self.metres_per_step + self.pretension
            self.data.ctrl[self.motor_ids[k]] = self.rest_length[k] - pull

        # 3. Let the physics run for one tick.
        for _ in range(self.substeps):
            mujoco.mj_step(self.model, self.data)

        # 4. Report back: motor positions and the shape of the trunk.
        self.pub_steps.publish(Int32MultiArray(data=[round(m) for m in self.motor]))
        joints = JointState()
        joints.header.stamp = self.get_clock().now().to_msg()
        joints.name = self.joint_names
        joints.position = [float(self.data.qpos[i]) for i in self.qpos_index]
        self.pub_joints.publish(joints)

        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()


def main(args=None):
    rclpy.init(args=args)
    node = TrunkSim()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.viewer is not None:
            node.viewer.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()