"""Command line front end: ros2 run drawbot draw <shape> [options]."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from drawbot import kinematics as kin
from drawbot import shapes

SERVICES = {'home': '/drawbot/home', 'stop': '/drawbot/stop', 'clear': '/drawbot/clear'}


def call_trigger(node, name):
    client = node.create_client(Trigger, name)
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(f'{name} is not available - is the simulation running?')
        return False
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    return future.done()


def send_strokes(node, strokes, wait):
    for stroke in strokes:
        for x, y in stroke:
            if kin.inverse(x, y) is None:
                node.get_logger().error(f'({x:.3f}, {y:.3f}) is out of the arm\'s reach')
                return False

    pub = node.create_publisher(PoseArray, '/drawbot/stroke', 100)
    deadline = time.monotonic() + 5.0
    while pub.get_subscription_count() == 0:
        if time.monotonic() > deadline:
            node.get_logger().error('No controller listening - is the simulation running?')
            return False
        rclpy.spin_once(node, timeout_sec=0.1)

    state = {'busy': None, 'started': False}

    def on_busy(msg):
        state['busy'] = msg.data
        state['started'] = state['started'] or msg.data

    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Bool, '/drawbot/busy', on_busy, latched)

    for stroke in strokes:
        msg = PoseArray()
        msg.header.frame_id = 'base_link'
        for x, y in stroke:
            pose = Pose()
            pose.position.x, pose.position.y = float(x), float(y)
            msg.poses.append(pose)
        pub.publish(msg)
    node.get_logger().info(f'Sent {len(strokes)} stroke(s)')

    # Let the messages go out, then (optionally) block until the arm is idle.
    sent = time.monotonic()
    while rclpy.ok():
        elapsed = time.monotonic() - sent
        if not wait and elapsed > 1.0:
            break
        if state['started'] and state['busy'] is False:
            break
        if not state['started'] and elapsed > 5.0:
            node.get_logger().error('The controller never started drawing')
            return False
        rclpy.spin_once(node, timeout_sec=0.1)
    return True


def main():
    parser = argparse.ArgumentParser(description='Draw with the drawbot.')
    parser.add_argument('what', choices=[*shapes.SHAPES, 'demo', *SERVICES])
    parser.add_argument('--size', type=float, default=0.05, help='shape size in metres')
    parser.add_argument('--x', type=float, default=kin.PAPER_CENTER[0], help='centre x (m)')
    parser.add_argument('--y', type=float, default=kin.PAPER_CENTER[1], help='centre y (m)')
    parser.add_argument('--no-wait', action='store_true', help="don't wait for the drawing")
    args = parser.parse_args()

    rclpy.init()
    node = Node('drawbot_draw')
    try:
        if args.what in SERVICES:
            ok = call_trigger(node, SERVICES[args.what])
        else:
            if args.what == 'demo':
                strokes = shapes.demo()
            else:
                strokes = shapes.place(shapes.SHAPES[args.what](args.size), args.x, args.y)
            ok = send_strokes(node, strokes, not args.no_wait)
    except KeyboardInterrupt:
        ok = False
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
