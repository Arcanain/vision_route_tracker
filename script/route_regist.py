#! /usr/bin/env python3
import rospy

import cv2 as cv
import numpy as np

from submodule.mono_vslam_py_prototype.app.route_register import RouteRegister

class RouteRegist:

    def __init__(self, src, route_dir, path_to_camera_mat):

        self.cap = cv.VideoCapture(src)
        self.frame = None
        self.routeRegister = RouteRegister(route_dir, path_to_camera_mat)

        self.route_dir = route_dir

        cv.namedWindow('plane')
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

            self.routeRegister.update_frame(self.frame)

            vis = self.frame.copy()
            if self.routeRegister.last_frame.value_available:

                p2k = self.routeRegister.last_frame.last_p2k

                if not self.paused:
                    print('')
                    print(p2k.pose[1])
                    print(p2k.is_close)

                for (x0, y0), (x1, y1) in zip(np.int32(p2k.p0), np.int32(p2k.p1)):
                    cv.circle(vis, (x1, y1), 2, (255, 255, 255))
                    cv.line(vis, (x0, y0), (x1, y1), (255, 255, 255))

            cv.imshow('plane', vis)

if __name__ == "__main__":
    rospy.init_node("route_regist", disable_signals=True)
    routeRegist = RouteRegist(0, './route_imgs/', './camera_data/3_6_mm_cam.npy').run()

