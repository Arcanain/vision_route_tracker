#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import cv2 as cv

from submodule.mono_vslam_py_prototype.app.navigator import Navigator


MAX_VELOCITY = 2.0 # m/s
MAX_ROTATION_VELOCITY = 1.0 # rad/s
GAIN_VELOCITY = 4.0
GAIN_ROTATION_VELOCITY = 0.5

class VisionRouteTracker:
    def __init__(self, src, route_dir):
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, tcp_nodelay=True, queue_size=10)


        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.navigator = Navigator(route_dir)

        self.route_dir = route_dir
        self.save_frame_num = 0

        cv.namedWindow('plane')
        cv.namedWindow('current_keyframe')
        self.paused = False

    def run(self):
        cv_vis_images = []
        while True:

            ch = cv.waitKey(1)
            if ch == 27:
                break
            if ch == ord('p'):
                self.paused = not self.paused

            if self.paused:
                continue

            ret, frame = self.cap.read()
            if not ret:
                break
            self.frame = frame.copy()

            vis = self.frame.copy()
            velocity, rotation = self.navigator.get_velocity_and_rotation( self.frame )

            cv.imshow('plane', vis)
            current_keyframe = self.navigator.keyframes_on_route[-1].keyframe.copy()
            cv.imshow('current_keyframe', current_keyframe)
            twist = Twist()
            if not self.paused:

                twist.linear.x = velocity
                twist.angular.z = rotation

            self.twist_publisher.publish(twist)

            did_finish = False

            if did_finish:
                print('finish')
                break


if __name__ == "__main__":
    rospy.init_node("vision_route_tracker", disable_signals=True)
    visionRouteTracker = VisionRouteTracker(2, './route_imgs/').run()
