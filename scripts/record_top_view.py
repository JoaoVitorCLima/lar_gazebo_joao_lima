#!/usr/bin/env python3
"""Record real annotated camera frames, finishing after visual patrol completes."""
import os
import threading
import shutil
import subprocess
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class Recorder:
    def __init__(self):
        self.writer = None
        self.closed = False
        self.lock = threading.RLock()
        self.bridge = CvBridge()
        self.path = rospy.get_param('~path')
        self.frames = 0
        os.makedirs(os.path.dirname(os.path.abspath(self.path)), exist_ok=True)
        rospy.Subscriber('/top_view/annotated', Image, self.frame, queue_size=2)
        rospy.Subscriber('/top_view/done', Bool, self.done)
        rospy.on_shutdown(self.close)

    def frame(self, msg):
        with self.lock:
            if self.closed:
                return
            image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            if self.writer is None:
                h, w = image.shape[:2]
                self.writer = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*'mp4v'), 15, (w, h))
                if not self.writer.isOpened():
                    raise RuntimeError('Cannot open video output')
            cv2.putText(image, 'LaR - localizacao visual | tempo simulado %.2f s' % msg.header.stamp.to_sec(),
                        (20, 70), cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 0, 255), 2)
            self.writer.write(image)
            self.frames += 1

    def done(self, msg):
        if msg.data and self.frames:
            rospy.signal_shutdown('Patrol finished')

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
            if self.writer is not None:
                self.writer.release()
                self.writer = None
                if shutil.which('ffmpeg'):
                    converted = self.path + '.h264.mp4'
                    result = subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', self.path,
                                             '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '23',
                                             '-pix_fmt', 'yuv420p', '-movflags', '+faststart', converted])
                    if result.returncode == 0:
                        os.replace(converted, self.path)
                    else:
                        rospy.logwarn('H.264 conversion failed; original MPEG-4 video preserved')


if __name__ == '__main__':
    rospy.init_node('record_top_view')
    Recorder()
    rospy.spin()
