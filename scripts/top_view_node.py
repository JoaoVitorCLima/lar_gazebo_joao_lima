#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_from_matrix
from lar_gazebo.geometry import estimate, estimate_on_plane


class Vision:
    def __init__(self):
        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.parameters = cv2.aruco.DetectorParameters_create()
        # The robot outline is close to the tag outline: retain both candidates.
        self.parameters.minMarkerDistanceRate = 0.01
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.gamma_lut = np.uint8(np.sqrt(np.arange(256) / 255.0) * 255)
        self.pub = rospy.Publisher('/top_view/pose', PoseStamped, queue_size=10)
        self.image_pub = rospy.Publisher('/top_view/annotated', Image, queue_size=1)
        self.world_camera = np.array(rospy.get_param('~world_camera'), dtype=float).reshape(4, 4)
        self.base_tag = np.eye(4)
        self.base_tag[:3, 3] = rospy.get_param('~tag_translation', [0, 0, 0.55])
        self.tag_height = rospy.get_param('~tag_world_height', 0.742)
        self.size = rospy.get_param('~tag_size', 0.40)
        image = message_filters.Subscriber('/overhead/image_raw', Image, queue_size=1, buff_size=16777216)
        info = message_filters.Subscriber('/overhead/camera_info', CameraInfo, queue_size=1)
        self.sync = message_filters.ApproximateTimeSynchronizer([image, info], 5, 0.02)
        self.sync.registerCallback(self.callback)

    def callback(self, msg, info):
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids, _ = cv2.aruco.detectMarkers(cv2.LUT(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), self.gamma_lut), self.dictionary, parameters=self.parameters)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            for c, tag_id in zip(corners, ids.ravel()):
                if tag_id != 42:
                    continue
                try:
                    T, residual = estimate(c, np.array(info.K).reshape(3, 3), np.array(info.D),
                                           self.size, self.world_camera, self.base_tag)
                    if residual > 2.0:
                        continue
                    T = estimate_on_plane(c, np.array(info.K).reshape(3, 3), np.array(info.D),
                                          self.world_camera, self.base_tag, self.tag_height)
                    pose = PoseStamped()
                    pose.header.stamp = msg.header.stamp
                    pose.header.frame_id = 'world'
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = T[:3, 3]
                    q = quaternion_from_matrix(T)
                    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
                    self.pub.publish(pose)
                    cv2.putText(image, 'AprilTag 42  x=%.3f y=%.3f  reproj=%.2f px' %
                                (T[0, 3], T[1, 3], residual), (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2)
                except (ValueError, cv2.error) as exc:
                    rospy.logwarn_throttle(5, str(exc))
        annotated = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        annotated.header = msg.header
        self.image_pub.publish(annotated)


if __name__ == '__main__':
    rospy.init_node('top_view')
    Vision()
    rospy.spin()
