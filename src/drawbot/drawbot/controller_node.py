"""Motion controller for the drawbot.

Turns pen strokes (polylines in the base frame) into joint position commands
for the two steppers and the pen-lift servo.

  /drawbot/stroke   geometry_msgs/PoseArray   one polyline, drawn pen-down
  /drawbot/home     std_srvs/Trigger          lift pen and park clear of the paper
  /drawbot/stop     std_srvs/Trigger          drop queued strokes, lift pen
  /drawbot/busy     std_msgs/Bool (latched)   true while there is motion queued
"""

import math
from collections import deque

import rclpy
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger

from drawbot import kinematics as kin


class Controller(Node):

    def __init__(self):
        super().__init__('drawbot_controller')
        self.rate = self.declare_parameter('rate', 100.0).value
        self.draw_speed = self.declare_parameter('draw_speed', 0.04).value      # m/s
        self.travel_speed = self.declare_parameter('travel_speed', 2.0).value   # rad/s
        self.travel_accel = self.declare_parameter('travel_accel', 12.0).value  # rad/s^2
        self.microsteps = self.declare_parameter('microsteps', 16).value
        self.pen_up_angle = self.declare_parameter('pen_up_angle', 0.35).value  # rad
        self.pen_dwell = self.declare_parameter('pen_dwell', 0.25).value        # s
        self.elbow_up = self.declare_parameter('elbow_up', False).value
        # Parked pose (shoulder, elbow): folded back beside the base, clear of the paper.
        self.home_q = list(self.declare_parameter('home_joints', [-2.16, 1.99]).value)

        # Commanded state. Gazebo spawns the arm straight out with the pen down.
        self.q = [0.0, 0.0]
        self.pen = 0.0
        self.queue = deque()   # ('pen', angle) | ('joint', q) | ('line', (x, y))
        self.joint_speed = 0.0   # current speed along a 'joint' move, rad/s
        self.dwell_ticks = 0
        self.busy = None

        self.pub_shoulder = self.create_publisher(Float64, '/drawbot/shoulder/cmd_pos', 10)
        self.pub_elbow = self.create_publisher(Float64, '/drawbot/elbow/cmd_pos', 10)
        self.pub_pen = self.create_publisher(Float64, '/drawbot/pen_lift/cmd_pos', 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_busy = self.create_publisher(Bool, '/drawbot/busy', latched)

        self.create_subscription(PoseArray, '/drawbot/stroke', self.on_stroke, 100)
        self.create_service(Trigger, '/drawbot/home', self.on_home)
        self.create_service(Trigger, '/drawbot/stop', self.on_stop)

        self.queue_home()
        self.create_timer(1.0 / self.rate, self.tick)

    # ---------------------------------------------------------------- inputs
    def queue_home(self):
        self.queue.append(('pen', self.pen_up_angle))
        self.queue.append(('joint', self.home_q))

    def on_home(self, _request, response):
        self.queue_home()
        response.success = True
        return response

    def on_stop(self, _request, response):
        self.queue.clear()
        self.joint_speed = 0.0
        self.queue.append(('pen', self.pen_up_angle))
        response.success = True
        return response

    def on_stroke(self, msg):
        points = [(p.position.x, p.position.y) for p in msg.poses]
        if not points:
            return
        for x, y in points:
            if kin.inverse(x, y, self.elbow_up) is None:
                self.get_logger().warn(
                    f'Skipping stroke: ({x:.3f}, {y:.3f}) is out of reach')
                return
        self.queue.append(('pen', self.pen_up_angle))
        self.queue.append(('joint', kin.inverse(*points[0], self.elbow_up)))
        self.queue.append(('pen', 0.0))
        self.queue.extend(('line', p) for p in points[1:])
        self.queue.append(('pen', self.pen_up_angle))

    # ---------------------------------------------------------------- motion
    def tick(self):
        dt = 1.0 / self.rate
        if self.dwell_ticks > 0:
            self.dwell_ticks -= 1
        elif self.queue:
            kind, target = self.queue[0]
            if kind == 'pen':
                done = self.step_pen(target)
            elif kind == 'joint':
                done = self.step_joint(target, dt)
            else:
                done = self.step_line(target, self.draw_speed * dt)
            if done:
                self.queue.popleft()

        self.publish_commands()
        busy = bool(self.queue) or self.dwell_ticks > 0
        if busy != self.busy:
            self.busy = busy
            self.pub_busy.publish(Bool(data=busy))

    def step_pen(self, angle):
        if not math.isclose(angle, self.pen):
            self.pen = angle
            self.dwell_ticks = round(self.pen_dwell * self.rate)
        return True

    def step_joint(self, target, dt):
        """Move both joints toward target so that they arrive together.

        Speed follows a trapezoidal profile; the motors cannot stop the arm
        dead from travel speed, and it would overshoot onto the paper.
        """
        delta = [target[0] - self.q[0], target[1] - self.q[1]]
        remaining = max(abs(delta[0]), abs(delta[1]))
        self.joint_speed = min(self.joint_speed + self.travel_accel * dt,
                               self.travel_speed,
                               math.sqrt(2.0 * self.travel_accel * remaining))
        step = self.joint_speed * dt
        if remaining <= max(step, 1e-6):
            self.q = list(target)
            self.joint_speed = 0.0
            return True
        scale = step / remaining
        self.q = [self.q[0] + delta[0] * scale, self.q[1] + delta[1] * scale]
        return False

    def step_line(self, target, max_step):
        """Move the pen tip in a straight line toward target."""
        x, y = kin.forward(*self.q)
        dx, dy = target[0] - x, target[1] - y
        dist = math.hypot(dx, dy)
        done = dist <= max_step
        if not done:
            target = (x + dx * max_step / dist, y + dy * max_step / dist)
        q = kin.inverse(*target, self.elbow_up)
        if q is None:   # endpoints were checked, so only a line cutting the inner limit
            self.get_logger().warn('Line leaves the workspace, skipping segment')
            return True
        self.q = list(q)
        return done

    def publish_commands(self):
        self.pub_shoulder.publish(Float64(data=kin.quantize(self.q[0], self.microsteps)))
        self.pub_elbow.publish(Float64(data=kin.quantize(self.q[1], self.microsteps)))
        self.pub_pen.publish(Float64(data=self.pen))


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
