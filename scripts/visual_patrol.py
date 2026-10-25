#!/usr/bin/env python3
"""Waypoint controller: pose comes exclusively from the overhead camera."""
import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from lar_gazebo.geometry import wrap


class Patrol:
    def __init__(self):
        self.pose = None
        self.index = 0
        self.points = rospy.get_param('~waypoints', [[5.65, 3.0], [5.65, 3.8], [4.65, 3.8], [4.65, 3.0]])
        self.pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=1)
        self.done = rospy.Publisher('/top_view/done', Bool, queue_size=1, latch=True)
        self.done.publish(False)
        rospy.Subscriber('/top_view/pose', PoseStamped, self.receive)
        rospy.on_shutdown(lambda: self.pub.publish(Twist()))
        self.timer = rospy.Timer(rospy.Duration(.05), self.step)

    def receive(self, pose):
        self.pose = pose

    def step(self, event):
        cmd = Twist()
        if self.index >= len(self.points):
            self.done.publish(True)
        elif self.pose is not None and 0 <= (rospy.Time.now() - self.pose.header.stamp).to_sec() < .5:
            p = self.pose.pose
            x, y = self.points[self.index]
            dx, dy = x-p.position.x, y-p.position.y
            q = p.orientation
            yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
            error = wrap(math.atan2(dy, dx)-yaw)
            if math.hypot(dx, dy) < .12:
                self.index += 1
                rospy.loginfo('Waypoint %s/%s', self.index, len(self.points))
            else:
                cmd.angular.z = max(-.5, min(.5, 1.5*error))
                if abs(error) < .35:
                    cmd.linear.x = min(.2, .6*math.hypot(dx, dy))
        self.pub.publish(cmd)


if __name__ == '__main__':
    rospy.init_node('visual_patrol')
    Patrol()
    rospy.spin()
