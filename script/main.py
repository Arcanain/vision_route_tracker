#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

import cv2 as cv
import numpy as np

from submodule.mono_vslam_py_prototype.app.navigator import Navigator


MAX_VELOCITY = 2.0 # m/s
MAX_ROTATION_VELOCITY = 1.0 # rad/s
GAIN_VELOCITY = 4.0
GAIN_ROTATION_VELOCITY = 0.5

class VisionRouteTracker:
    def __init__(self, src, route_dir, debug=False):
        self.twist_publisher = rospy.Publisher("cmd_vel", Twist, tcp_nodelay=True, queue_size=10)


        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.navigator = Navigator(route_dir, debug=debug)

        self.route_dir = route_dir
        self.save_frame_num = 0

        cv.namedWindow('plane')
        self.paused = False

    def run(self):
        cv_vis_images = []
        rate = rospy.Rate(5)
        start_time = rospy.get_rostime()

        while not rospy.is_shutdown():

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

            print('a')
            print((rospy.get_rostime() - start_time).to_sec())

            start_time = rospy.get_rostime()

            velocity, rotation = self.navigator.get_velocity_and_rotation( self.frame )

            print((rospy.get_rostime() - start_time).to_sec())

            vis = self.frame.copy()
            if self.navigator.keyframes_on_route[0].value_available:

                p2k = self.navigator.keyframes_on_route[0].last_p2k

                for (x0, y0), (x1, y1) in zip(np.int32(p2k.p0), np.int32(p2k.p1)):
                    cv.circle(vis, (x1, y1), 2, (255, 255, 255))
                    cv.line(vis, (x0, y0), (x1, y1), (255, 255, 255))
            cv.imshow('plane', vis)

            twist = Twist()
            if not self.paused:

                twist.linear.x = velocity
                twist.angular.z = rotation

            self.twist_publisher.publish(twist)

            did_finish = False

            if did_finish:
                print('finish')
                break

            rate.sleep()


if __name__ == "__main__":
    import sys
    try:
        debug = bool(sys.argv[1])
        print(type(debug))
    except:
        debug = False

    rospy.init_node("vision_route_tracker", disable_signals=True)
    visionRouteTracker = VisionRouteTracker(0, './route_imgs/', debug=debug).run()
