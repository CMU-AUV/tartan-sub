#!/usr/bin/env python
import rospy

from task import Task

import cv2
import numpy as np

path_vision_options = {
    'debugging':[ False],
    'canny1':[ 50, 0, 200],
    'canny2':[ 150, 0, 200],
    'min_angle_diff':[ np.pi/8, 0, np.pi*2],
    'min_line_hough_length':[45, 0, 500],
    'color_dist_min':[ 0, 0, 255],
    'color_dist_max':[ 20, 0, 255],
    'min_angle_range':[ 35, 0, 180],
    'max_angle_range':[ 55, 0, 180],
}


class Path(Task):
    tracked_lines = []
    def __init__(self, sub_controller, run_config, visualize=False):
        self.mover = sub_controller.mover
        self.config = run_config

        self.camera_sub = rospy.Subscriber(self.config.camera_topic, Image, self.image_callback)
        self.bridge = CvBridge()
        self.visualize = visualize

    def angle(self, x1, y1, x2, y2):
        a = atan( (x2-x1) / (y2-y1) )
        return a

    def angle_diff(self, a1, a2):
        a = atan(sin(a1-a2)/cos(a1-a2))
        return abs(a)

    def threshold(self, mat):
        threshes = {}

        lab = cv2.cvtColor(mat, cv2.COLOR_BGR2LAB)

        dist_from_orange = np.linalg.norm(lab[:, :, :].astype(int) - [90, 144, 131], axis=2).astype(int)

        orange_threshed = cv2.inRange(dist_from_orange, self.options["color_dist_min"], self.options["color_dist_max"])

        morphed = orange_threshed
        threshes['morphed'] = morphed

        # hack hack hack for simulator
        sat = cv2.split(cv2.cvtColor(mat, cv2.COLOR_BGR2HSV))[1]
        morphed = cv2.inRange(sat, 20, 255)

        edges = cv2.Canny(morphed,threshold1=self.options['canny1'],threshold2=self.options['canny2'],apertureSize=3) #self.options['canny_aperture_size'])
        threshes["edges"] = edges

        return threshes

    def find_lines(self, mat, thresh):
        minLineLength = self.options['min_line_hough_length']

        theta_res = radians(2)

        lines = cv2.HoughLines(thresh,1,theta_res,int(minLineLength))

        if lines is None:
            return []

        if True:
            line_image = np.copy(mat)
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000*(-b))
                y1 = int(y0 + 1000*(a))
                x2 = int(x0 - 1000*(-b))
                y2 = int(y0 - 1000*(a))
                cv2.line(line_image,(x1,y1),(x2,y2),(0,255,0),10)

        return lines

    #A list of segment_infos sorted in order of best
    def average_lines(self,lines,mat):
        if lines is None:
            return lines

        info = []
        final = []

        for l in lines:
            rho, theta = l[0]
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a * rho
            y0 = b * rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            angle = self.angle(x1, y1, x2, y2)
            length = sqrt((y2-y1)**2 + (x2-x1)**2)

            x = line_info(x1,y1,x2,y2, angle, length, 0)
            info.append(x)

        # Pick big lines first
        info.sort(key=lambda x: x.length, reverse=True)

        # Put into buckets
        buckets = []
        for line in info:
            for bucket in buckets:
                if self.angle_diff(line.angle, bucket[0].angle) < self.options['min_angle_diff']:
                    bucket.append(line)
                    break
            else:
                buckets.append([line])

        # Average each bucket
        final = [line_info(*[sum([line[x] for line in bucket]) / len(bucket) for x in range(len(bucket[0]))]) for bucket in buckets]

        if self.visualize:
            line_image = np.copy(mat)

            for line in final:
                x1,y1,x2,y2 = line.x1,line.y1,line.x2,line.y2
                cv2.line(line_image,(int(x1),int(y1)),(int(x2),int(y2)),(0, 0, 255),15)
            for line in info:
                x1,y1,x2,y2 = line.x1,line.y1,line.x2,line.y2
                cv2.line(line_image,(int(x1),int(y1)),(int(x2),int(y2)),(0, 255, 0),5)

        return final

    def get_intersection(self, lines):
        x1 = lines[0].x1
        x2 = lines[0].x2
        y1 = lines[0].y1
        y2 = lines[0].y2

        x3 = lines[1].x1
        x4 = lines[1].x2
        y3 = lines[1].y1
        y4 = lines[1].y2

        px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
        py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )

        return [px, py]


    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image = cv_image
            ang1, ang2 = self.process(cv_image)
        except CvBridgeError as e:
            print(e)
            cv2.waitKey(10)

    def process(self, mat):

        image_size = mat.shape[0]*mat.shape[1]

        threshes = self.threshold(mat)

        lineMat = np.copy(mat)

        lines = self.find_lines(mat, threshes["edges"])

        error = namedtuple("error", ["lines", "error"])

        linesI = self.average_lines(lines, mat)

        path_angle = []

        for line in linesI:
          path_angle.append(line.angle)

        second_line = False
        flipped = False

        if len(linesI) == 2:
          px, py = self.get_intersection(linesI)

        if len(linesI) == 2:
          old_angle_1 = shm.path_results.angle_1.get()
          old_angle_2 = shm.path_results.angle_2.get()

          angle_diff_1 = self.angle_diff(old_angle_1, path_angle[0])
          angle_diff_2 = self.angle_diff(old_angle_2, path_angle[0])

          angle_diff_3 = self.angle_diff(old_angle_1, path_angle[1])
          angle_diff_4 = self.angle_diff(old_angle_2, path_angle[1])

          angle_diffs = [angle_diff_1, angle_diff_2, angle_diff_3, angle_diff_4]

          angle_diffs.sort()

          if angle_diffs[0] == angle_diff_2 or angle_diffs[0] == angle_diff_3 :
            path_angle[0], path_angle[1] = path_angle[1], path_angle[0]
            flipped = True

          shm.path_results.angle_1.set(path_angle[0])
          shm.path_results.angle_2.set(path_angle[1])
          shm.path_results.visible_1.set(True)
          shm.path_results.visible_2.set(True)
          shm.path_results.center_x.set(self.normalized(px,1))
          shm.path_results.center_y.set(self.normalized(py,0))
          shm.path_results.num_lines.set(2)

        elif len(linesI) == 1:
          old_angle_1 = shm.path_results.angle_1.get()
          old_angle_2 = shm.path_results.angle_2.get()
          angle_diff_1 = self.angle_diff(old_angle_1, path_angle[0])
          angle_diff_2 = self.angle_diff(old_angle_2, path_angle[0])

          if angle_diff_1 < angle_diff_2:
            shm.path_results.angle_1.set(path_angle[0])
            shm.path_results.visible_1.set(True)
            shm.path_results.visible_2.set(False)
          elif angle_diff_1 > angle_diff_2:
            shm.path_results.angle_2.set(path_angle[0])
            shm.path_results.visible_1.set(False)
            shm.path_results.visible_2.set(True)
            second_line = True

          shm.path_results.num_lines.set(1)

        else:
          shm.path_results.visible_1.set(False)
          shm.path_results.visible_2.set(False)
          shm.path_results.num_lines.set(0)


        line_image = np.copy(mat)
        if flipped:
            linesI[0], linesI[1] = linesI[1], linesI[0]

        for i, line in enumerate(linesI):
          x1,y1,x2,y2 = line.x1,line.y1,line.x2,line.y2
          if i == 0 and not second_line:
            cv2.line(line_image,(int(x1),int(y1)),(int(x2),int(y2)),(255,0,0),5)
          elif i == 1 or second_line:
            cv2.line(line_image,(int(x1),int(y1)),(int(x2),int(y2)),(0,255,0),5)
