#!/usr/bin/env python3
"""Only this node consumes simulator ground truth. Interpolate at image timestamp."""
import bisect
import csv
import os
import threading
from collections import deque
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from lar_gazebo.geometry import wrap


def yaw(p):
    q = p.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class Evaluation:
    def __init__(self):
        self.finished = False
        self.closed = False
        self.lock = threading.RLock()
        self.truth = deque(maxlen=2000)
        self.pending = deque(maxlen=100)
        path = rospy.get_param('~csv')
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        self.file = open(path, 'w', buffering=1)
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time', 'x', 'y', 'yaw', 'gt_x', 'gt_y', 'gt_yaw', 'error_position_m', 'error_yaw_rad'])
        rospy.Subscriber('/evaluation/ground_truth', Odometry, self.ground_truth, queue_size=100)
        rospy.Subscriber('/top_view/pose', PoseStamped, self.vision, queue_size=100)
        rospy.Subscriber('/top_view/done', Bool, self.done)
        rospy.on_shutdown(self.close)

    def ground_truth(self, msg):
        with self.lock:
            if self.finished or self.closed:
                return
            self.truth.append((msg.header.stamp.to_sec(), msg.pose.pose))
            self.flush()

    def vision(self, msg):
        with self.lock:
            if self.finished or self.closed:
                return
            self.pending.append(msg)
            self.flush()

    def done(self, msg):
        if msg.data:
            with self.lock:
                if self.finished or self.closed:
                    return
                self.finished = True
                self.flush()
                self.pending.clear()
                self.file.flush()

    def close(self):
        """Finalize the CSV once, while serializing with ROS callbacks."""
        with self.lock:
            if self.closed:
                return
            self.finished = True
            self.flush()
            self.pending.clear()
            self.file.flush()
            self.file.close()
            self.closed = True

    def flush(self):
        if len(self.truth) < 2:
            return
        times = [p[0] for p in self.truth]
        while self.pending:
            msg = self.pending[0]
            t = msg.header.stamp.to_sec()
            if t > times[-1]:
                break
            self.pending.popleft()
            j = min(bisect.bisect_right(times, t), len(times)-1)
            if j == 0 or j >= len(times) or times[j]-times[j-1] > .1:
                continue
            a, b = self.truth[j-1][1], self.truth[j][1]
            f = (t-times[j-1])/(times[j]-times[j-1])
            gx = a.position.x + f*(b.position.x-a.position.x)
            gy = a.position.y + f*(b.position.y-a.position.y)
            gyaw = wrap(yaw(a)+f*wrap(yaw(b)-yaw(a)))
            p = msg.pose
            eyaw = wrap(yaw(p)-gyaw)
            self.writer.writerow([t, p.position.x, p.position.y, yaw(p), gx, gy, gyaw,
                                  ((p.position.x-gx)**2+(p.position.y-gy)**2)**.5, eyaw])


if __name__ == '__main__':
    rospy.init_node('evaluate_top_view')
    Evaluation()
    rospy.spin()
