"""Draws the ink the pen leaves behind.

The pen tip is tracked from the simulated joint states (not the commands), so
the trail shows what the arm really did. Ink is laid down whenever the pen is
lowered over the paper, and is shown both in RViz (MarkerArray on
/drawbot/trail) and in the Gazebo GUI (through its /marker_array service).

  /drawbot/clear   std_srvs/Trigger   wipe the paper
"""

import math
import queue
import subprocess
import threading

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from drawbot import kinematics as kin

INK_Z = 0.0108           # just above the paper surface, base frame
GZ_NAMESPACE = 'drawbot_trail'
GZ_CHUNK = 150           # points per Gazebo marker, keeps requests small


class Trail(Node):

    def __init__(self):
        super().__init__('drawbot_trail')
        self.pen_down_angle = self.declare_parameter('pen_down_angle', 0.03).value
        self.min_spacing = self.declare_parameter('min_spacing', 0.0004).value
        self.line_width = self.declare_parameter('line_width', 0.0009).value
        self.use_gazebo = self.declare_parameter('gazebo_markers', True).value

        self.strokes = []        # list of [(x, y), ...]
        self.drawing = False
        self.gz_sent = {}        # (stroke index, chunk index) -> points already sent
        self.gz_queue = queue.Queue()

        self.pub = self.create_publisher(MarkerArray, '/drawbot/trail', 1)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_service(Trigger, '/drawbot/clear', self.on_clear)
        self.create_timer(0.2, self.publish_rviz)
        if self.use_gazebo:
            self.create_timer(0.5, self.queue_gazebo)
            threading.Thread(target=self.gazebo_worker, daemon=True).start()

    # ---------------------------------------------------------------- tracking
    def on_joints(self, msg):
        try:
            pos = dict(zip(msg.name, msg.position))
            shoulder, elbow, lift = pos['shoulder_joint'], pos['elbow_joint'], pos['pen_lift_joint']
        except KeyError:
            return
        x, y = kin.forward(shoulder, elbow)
        on_paper = (kin.PAPER_X[0] <= x <= kin.PAPER_X[1]
                    and kin.PAPER_Y[0] <= y <= kin.PAPER_Y[1])
        if lift > self.pen_down_angle or not on_paper:
            self.drawing = False
            return
        if not self.drawing:
            self.strokes.append([(x, y)])
            self.drawing = True
            return
        last = self.strokes[-1][-1]
        if math.hypot(x - last[0], y - last[1]) >= self.min_spacing:
            self.strokes[-1].append((x, y))

    def on_clear(self, _request, response):
        self.strokes = []
        self.drawing = False
        self.gz_sent = {}
        self.pub.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))
        if self.use_gazebo:
            self.gz_queue.put(f'marker {{ ns: "{GZ_NAMESPACE}" action: DELETE_ALL }}')
        response.success = True
        return response

    # ---------------------------------------------------------------- RViz
    def publish_rviz(self):
        array = MarkerArray()
        for i, stroke in enumerate(self.strokes):
            if len(stroke) < 2:
                continue
            m = Marker()
            m.header.frame_id = 'base_link'
            m.ns = 'trail'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.pose.orientation.w = 1.0
            m.scale.x = self.line_width
            m.color.r, m.color.g, m.color.b, m.color.a = 0.05, 0.10, 0.60, 1.0
            m.points = [Point(x=x, y=y, z=INK_Z) for x, y in stroke]
            array.markers.append(m)
        if array.markers:
            self.pub.publish(array)

    # ---------------------------------------------------------------- Gazebo
    def queue_gazebo(self):
        """Queue every marker chunk that has grown since it was last sent."""
        if not self.gz_queue.empty():   # GUI is slow or absent, don't pile up
            return
        markers = []
        for i, stroke in enumerate(self.strokes):
            # Chunks overlap by one point so the line stays continuous.
            for c, start in enumerate(range(0, len(stroke) - 1, GZ_CHUNK)):
                points = stroke[start:start + GZ_CHUNK + 1]
                if self.gz_sent.get((i, c)) == len(points):
                    continue
                self.gz_sent[(i, c)] = len(points)
                markers.append(self.ribbon_marker(i * 1000 + c + 1, points))
        if markers:
            self.gz_queue.put(' '.join(markers))

    def ribbon_marker(self, marker_id, points):
        """A flat strip of the pen's line width, as gz.msgs.Marker text."""
        half = self.line_width / 2.0
        verts = []
        for k, (x, y) in enumerate(points):
            ax, ay = points[max(k - 1, 0)]
            bx, by = points[min(k + 1, len(points) - 1)]
            norm = math.hypot(bx - ax, by - ay) or 1.0
            nx, ny = -(by - ay) / norm * half, (bx - ax) / norm * half
            verts.append(f'point {{ x: {x + nx:.5f} y: {y + ny:.5f} z: {INK_Z} }}')
            verts.append(f'point {{ x: {x - nx:.5f} y: {y - ny:.5f} z: {INK_Z} }}')
        ink = 'r: 0.05 g: 0.10 b: 0.60 a: 1'
        return (f'marker {{ ns: "{GZ_NAMESPACE}" id: {marker_id} action: ADD_MODIFY '
                f'type: TRIANGLE_STRIP '
                f'material {{ ambient {{ {ink} }} diffuse {{ {ink} }} emissive {{ {ink} }} }} '
                f'{" ".join(verts)} }}')

    def gazebo_worker(self):
        while True:
            request = self.gz_queue.get()
            try:
                subprocess.run(
                    ['gz', 'service', '-s', '/marker_array',
                     '--reqtype', 'gz.msgs.Marker_V', '--reptype', 'gz.msgs.Boolean',
                     '--timeout', '1000', '--req', request],
                    capture_output=True, timeout=5.0, check=False)
            except (OSError, subprocess.TimeoutExpired):
                pass


def main():
    rclpy.init()
    node = Trail()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
