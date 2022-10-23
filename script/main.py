#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import cv2 as cv
from scipy.spatial.transform import Rotation as R
import numpy as np

from submodule.mono_vslam_py_prototype.app.route_tracker import RouteTracker


MAX_VELOCITY = 2.0 # m/s
MAX_ROTATION_VELOCITY = 1.0 # rad/s
GAIN_VELOCITY = 4.0
GAIN_ROTATION_VELOCITY = 0.5

class VisionRouteTracker:
    def __init__(self, src, route_dir):
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, tcp_nodelay=True, queue_size=10)


        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.routeTracker = RouteTracker(route_dir)

        self.route_dir = route_dir
        self.save_frame_num = 0

        cv.namedWindow('plane')
        cv.namedWindow('current_keyframe')
        self.paused = False

    def arrange_in_range(self, value, abs_value):
        return min(abs_value, max(-abs_value, value))

    def run(self):
        cv_vis_images = []
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            self.frame = frame.copy()

            vis = self.frame.copy()
            p2k, did_finish, rotation = self.routeTracker.get_cmd(self.frame)
            if p2k:
                r = R.from_matrix(p2k.pose[0])

                for (x0, y0), (x1, y1) in zip(np.int32(p2k.p0), np.int32(p2k.p1)):
                    cv.circle(vis, (x1, y1), 2, (255, 255, 255))
                    cv.line(vis, (x0, y0), (x1, y1), (255, 255, 255))
            else:
                print('no p2k')

            cv.imshow('plane', vis)
            current_keyframe = self.routeTracker.current_keyframe.copy()
            cv.imshow('current_keyframe', current_keyframe)
            twist = Twist()
            if not self.paused and rotation:

                print(rotation)

                twist.linear.x = self.arrange_in_range( 1.0 / (abs(rotation) + 1.0) * GAIN_VELOCITY, MAX_VELOCITY )
                twist.angular.z = self.arrange_in_range( rotation * GAIN_ROTATION_VELOCITY, MAX_ROTATION_VELOCITY )

            self.twist_publisher.publish(twist)

            ch = cv.waitKey(100)
            if ch == 27:
                break
            if ch == ord('p'):
                self.paused = not self.paused
            if did_finish:
                print('finish')
                break



if __name__ == "__main__":
    rospy.init_node("vision_route_tracker", disable_signals=True)
    visionRouteTracker = VisionRouteTracker(2, './route_imgs/').run()
