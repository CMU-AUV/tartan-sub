import sys
import rospy
import roslib
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, MagneticField, FluidPressure
import cv2
import numpy as np
#from matplotlib import pyplot as plt
import time
import math

class circle(object):
    rod_s = cv2.imread('rod_img_small.png', 0)
    # rod_s = cv2.imread('/home/roboclub/catkin_ws/src/qualify_task/rod_img_small.png')
    rod_s = np.float32(rod_s)/255
    # the distance between the 2 legs of the gate is estimated as 322 pixels in the picture
    gate = np.zeros((rod_s.shape[0], 322+rod_s.shape[1]-1), dtype=np.float32)
    gate[:, 0:rod_s.shape[1]] = rod_s
    gate[:, -rod_s.shape[1]:] = rod_s
    poll_4 = cv2.resize(rod_s, None, fx=0.25, fy=0.25)
    poll_2 = cv2.resize(rod_s, None, fx=0.5, fy=0.5)

    camera_width = 768
    camera_height = 492
    camera_hfov = 1.5125
    camera_f = (0.5*camera_width)/math.tan(camera_hfov/2)
    camera_center = np.array([camera_width/2, camera_height/2])

    # origin 
    X_origin = np.array([0.0, 0.0, 0.0, 0.0])
    # top end of the poll
    X_poll_t = np.array([-15.7, -29.1, -46.0, 0.0]) - X_origin
    # bottom end of the poll
    X_poll_b = np.array([-15.7, -29.1, -47.3, 0.0]) - X_origin
    # bottom end of the gate's left leg
    X_gate_l = np.array([-5.0, -31.0, -45-1.45, 0.0]) - X_origin
    # bottom end of the gate's right leg
    X_gate_r = np.array([-5.0, -27.95, -45-1.45, 0.0]) - X_origin
    # bottom of the gate's center
    X_gate_c = np.array([-5.0, (-31.0-27.95)/2, -45-1.45, 0.0]) - X_origin
    # height of the gate
    X_gate_h = 1.55
    # camera position
    X_camera = np.array([1.15, 0.0, 0.4, 0.0])
    # pressure sensor position
    X_pressure = np.array([-1.32, 0.5, 0.85, 0.0])

    # minVal and maxVal for Canny
    edge_minVal = 100
#    edge_maxVal = 200
    edge_maxVal = 150
    # max diameter of the rod
    max_dia = 40
    # sea surface pressure
    pressure_sea = 101.325
    pressure_kPaPerM = 9.80638

    def __init__(self):
        self.pub = rospy.Publisher("/rexrov/cmd_vel", Twist, queue_size=1)
        self.image_sub = rospy.Subscriber("/rexrov/rexrov/camera/camera_image",Image, self.callbackImage)
        self.mag_sub = rospy.Subscriber("/rexrov/magnetometer", MagneticField, self.callbackMag)
        self.pres_sub = rospy.Subscriber("/rexrov/pressure", FluidPressure, self.callbackPres)
        self.bridge = CvBridge()
        self.deltaT = 0.5
        self.speed = 0.15
        # the observation of the gate or the poll
        self.Z_isGate = True
        self.Z_g = np.array([0.0, 0.0])
        self.Z_gg = np.array([0.0, 0.0])   # for look() only
        # the orientation from magnetometer
        self.theta_mag = 0.0
        # orientation toward the gate
        self.theta_gate = math.pi
        # the depth in meter calculated from the pressure sensor
        self.depth = 0.0
        self.depth2Dive = -44.5
        # the pose derived from motion and camera and other sensor observation
        self.X = np.array([0.0, -30.0, -45.0, 165.0%(2*math.pi)])
        # the pose derived from motion
        self.X_ = np.copy(self.X)
        # the 'believe' matrix used in localize()
        self.KH = np.zeros((4, 4))
        # the command queue and the event queue
        self.cmds = []
        self.evnt = []

    def callbackImage(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # Object detection methods
            # self.template_matching(cv_image)
            self.edge_detecting(cv_image)
        except CvBridgeError as e:
            print(e)

    def callbackMag(self, msg):
        vect = np.array([-msg.magnetic_field.x, msg.magnetic_field.y])
        # theta_mag = angle between +x and vect
        self.theta_mag = np.arccos(-msg.magnetic_field.x/np.linalg.norm(vect))

    def callbackPres(self, msg):
        # the pressure is in kPa, not pascals as said in ROS document
        # At the sea surface, the pressure is 101.325 kPa. It increases 9.80638 kPa per meter in uuv_simulator.
        # These 2 constants can be different in the real environment. So, they are defined as constants.
        self.depth = -((msg.fluid_pressure-self.pressure_sea)/self.pressure_kPaPerM+self.X_pressure[2])

    def template_matching(self, src):
        img_g = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        img_g = np.float32(img_g)/255
        result = cv2.matchTemplate(img_g, self.gate, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, gate_loc = cv2.minMaxLoc(result)
        h, w = self.gate.shape
        img_in_l = img_g[gate_loc[1]:, gate_loc[0]:gate_loc[0]+w/2]
        img_in_r = img_g[gate_loc[1]:, gate_loc[0]+w/2:gate_loc[0]+w]
        result_l = cv2.matchTemplate(img_in_l, self.rod_s, cv2.TM_CCOEFF)
        result_r = cv2.matchTemplate(img_in_r, self.rod_s, cv2.TM_CCOEFF)
        _, _, _, l_loc = cv2.minMaxLoc(result_l)
        _, _, _, r_loc = cv2.minMaxLoc(result_r)
        hs, ws = self.rod_s.shape
        l_loc = (l_loc[0]+gate_loc[0], l_loc[1]+gate_loc[1])
        r_loc = (r_loc[0]+gate_loc[0]+w/2, r_loc[1]+gate_loc[1])
        self.Z_g[0] = (l_loc[0]+r_loc[0]+ws)/2
        self.Z_g[1] = (l_loc[1]+r_loc[1])/2+hs
        
    def edge_detecting(self, src):
        img_g = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(img_g, self.edge_minVal, self.edge_maxVal)
        loc = np.where(edges>0)
        ux, cx = np.unique(loc[1], return_counts=True)
        uy, cy = np.unique(loc[0], return_counts=True)
        mean_cx = np.mean(cx)
        std_cx = np.std(cx)
        mean_cy = np.mean(cy)
        std_cy = np.std(cy)
        legs = filter(lambda ucx: ucx[1] > mean_cx+std_cx, zip(ux, cx))
        bar = filter(lambda ucy: ucy[1] > mean_cy+std_cy, zip(uy, cy))
        self.Z_g[0] = 0
        self.Z_g[1] = 0
        if self.Z_isGate:
            if len(bar) > 0 and len(legs) > 0:
                legs = zip(*legs)
                x_list = legs[0]
                x_l = x_list[0]
                x_r = x_list[-1]
                bar = zip(*bar)
                y_list = bar[0]
                y_list = [y for y in y_list if y < y_list[0]+self.max_dia]
                self.Z_g[0] = (x_l+x_r)/2
                self.Z_g[1] = (y_list[0]+y_list[-1])/2 + (x_r-x_l)/(self.X_gate_r[1]-self.X_gate_l[1])*self.X_gate_h
        else:
            lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold=self.max_dia, maxLineGap=self.max_dia/2)
            if len(legs) > 0 and legs[-1][0]-legs[0][0] < self.max_dia and lines is not None and len(lines) > 0:
                legs = zip(*legs)
                x_list = legs[0]
                x_l = x_list[0]
                x_r = x_list[-1]
                x1, y1, x2, y2 = x_l, 0, x_l, 0
                for line in lines:
                    xx1, yy1, xx2, yy2 = line[0]
                    if (x1-x2)**2 + (y1-y2)**2 < (xx1-xx2)**2 + (yy1-yy2)**2:
                        x1, y1, x2, y2 = xx1, yy1, xx2, yy2
                self.Z_g[0] = (x_l+x_r)/2
                self.Z_g[1] = min(y1, y2)

    def toPict(self, X_p, X):
        w = (X_p[1]-X[1])*math.cos(X[3]) - (X_p[0]-X[0])*math.sin(X[3])-self.X_camera[1]
        h = X_p[2]-X[2]-self.X_camera[2]
        dist = (X_p[0]-X[0])*math.cos(X[3]) + (X_p[1]-X[1])*math.sin(X[3])-self.X_camera[0]
        return np.array([-self.camera_f*w/dist+self.camera_width/2, -self.camera_f*h/dist+self.camera_height/2])

    def spin1(self, speed_x, speed_yaw):
        msg = Twist()
        msg.linear.x = speed_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = speed_yaw
        self.pub.publish(msg)
        # motion equation
        r = abs(speed_x/speed_yaw)
        deltaX = r*(math.cos(self.X_[3]+speed_yaw*self.deltaT)-math.cos(self.X_[3]))
        deltaY = r*(math.sin(self.X_[3]+speed_yaw*self.deltaT)-math.sin(self.X_[3]))
        self.X_ = self.X_+np.array([deltaX, deltaY, 0.0, speed_yaw*self.deltaT])
        self.X_[3] = self.X_[3]%(2*math.pi)

    def spin2(self, speed_y, speed_yaw):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = speed_y
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = speed_yaw
        self.pub.publish(msg)
        # motion equation
        r = abs(speed_y/speed_yaw)
        deltaX = r*(math.cos(self.X_[3]+speed_yaw*self.deltaT)-math.cos(self.X_[3]))
        deltaY = r*(math.sin(self.X_[3]+speed_yaw*self.deltaT)-math.sin(self.X_[3]))
        self.X_ = self.X_+np.array([deltaX, deltaY, 0.0, speed_yaw*self.deltaT])
        self.X_[3] = self.X_[3]%(2*math.pi)

    def moveX(self, speed_x):
        msg = Twist()
        msg.linear.x = speed_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        # motion equation
        r = speed_x*self.deltaT
        self.X_ = self.X_+np.array([r*math.cos(self.X_[3]), r*math.sin(self.X_[3]), 0.0, 0.0])

    def moveY(self, speed_y):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = speed_y
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        # motion equation
        r = speed_y*self.deltaT
        self.X_ = self.X_+np.array([-r*math.sin(self.X_[3]), r*math.cos(self.X_[3]), 0.0, 0.0])

    def moveZ(self, speed_z):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = speed_z
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)
        # motion equation
        self.X_ = self.X_+np.array([0.0, 0.0, speed_z*self.deltaT, 0.0])

    def halt(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.pub.publish(msg)

    def start(self):
        self.cmds.append(('halt', ('search4gate',)))

    def search4Gate(self):
        print 'depth =', self.depth
        if self.depth2Dive < 0.0:
            self.goZ(self.depth2Dive-self.depth)
        rad2turn = self.theta_gate-self.theta_mag
        n = int(rad2turn/(self.speed*self.deltaT))
        for i in range(n):
            self.cmds.append(('spin2', 0.0, self.speed))
        self.cmds.append(('spin2', 0.0, rad2turn-n*self.speed*self.deltaT, ('gox', -1.0, ('lookagain',))))
        self.cmds.append(('halt', ))
        self.cmds.append(('halt', ('look',)))

    def goDist(self, dist, evnt, cmd):
        sign = 1.0
        if dist < 0.0:
            sign = -1.0
        n = int(abs(dist/(self.speed*self.deltaT)))
        for i in range(n):
            self.cmds.append((cmd, sign*self.speed))
        rest = dist-n*sign*self.speed*self.deltaT
        if abs(rest) > 0:
            self.cmds.append((cmd, rest))
        if evnt != None:
            self.cmds.append(('halt',))
            self.cmds.append(('halt', evnt))
        
    def goX(self, distance, event):
        self.goDist(distance, event, 'movex')

    def goY(self, distance):
        self.goDist(distance, None, 'movey')

    def goZ(self, distance):
        self.goDist(distance, None, 'movez')

    def look(self):
        self.Z_gg[0] = self.Z_g[0]
        self.Z_gg[1] = self.Z_g[1]
        if self.Z_isGate:
            print 'gate observed:', self.Z_gg, 'estimated:', self.toPict(self.X_gate_c, self.X_)
        else:
            print 'poll observed:', self.Z_gg, 'estimated:', self.toPict(self.X_poll_t, self.X_)

    def localize(self):
        if (self.Z_gg[0] == self.Z_g[0] and self.Z_gg[1] == self.Z_g[1]) or (self.Z_gg[0] == 0 and self.Z_gg[1] == 0) or (self.Z_g[0] == 0 and self.Z_g[1] == 0):
            print 'invalid observation'
            self.X_[2] = self.depth
            self.X_[3] = self.theta_mag
            self.X = np.copy(self.X_)
            return
        # Now, we have 
        # self.Z_gg = (-w'+width/2, -h'+height/2) = (-f*w/dist+width/2, -f*h/dist+height/2)
        # self.Z_g = (-w''+width/2, -h''+height/2) = (-f*w/(dist+1)+width/2, -f*h/(dist+1)+height/2)
        # camera_center = (width/2, height/2)
        # self.Z_gg-camera_center = (-f*w/dist, -f*h/dist)
        # self.Z_g-camera_center = (-f*w/(dist+1), -f*h/(dist+1))
        # (self.Z_gg-camera_center)/(self.Z_g-camera_center) = ((dist+1)/dist, (dist+1)/dist) = (1+1/dist, 1+1/dist)
        gg_ = self.Z_gg-self.camera_center
        g_ = self.Z_g-self.camera_center
        epsilon = 2
        if abs(gg_[0]-g_[0]) > epsilon and abs(gg_[1]-g_[1]) > epsilon:
            dist = (float(g_[0])/(gg_[0]-g_[0])+float(g_[1])/(gg_[1]-g_[1]))/2
        elif abs(gg_[1]-g_[1]) > epsilon:
            dist = float(g_[1])/(gg_[1]-g_[1])
        elif abs(gg_[0]-g_[0]) > epsilon:
            dist = float(g_[0])/(gg_[0]-g_[0])
        else:
            if abs(gg_[1]-g_[1]) > abs(gg_[0]-g_[0]):
                dist = float(g_[1])/(gg_[1]-g_[1])
            else:
                dist = float(g_[0])/(gg_[0]-g_[0])
        w_h = gg_*dist/(-self.camera_f)
        w = w_h[0]
        h = w_h[1]
        print 'w=', w, 'h=', h, 'dist=', dist
        # w = (y_gate-y_sub)*cos(theta_sub) - (x_gate-x_sub)*sin(theta_sub) - X_camera[1]
        # h = z_gate-z_sub - self.X_camera[2]
        # dist = (x_gate-x_sub)*cos(theta_sub) + (y_gate-y_sub)*sin(theta_sub) - X_camera[0]
        # X_gate_c = (x_gate, y_gate, z_gate, 0)
        # calculate X_sub
        # z_sub = z_gate-0.4-h
        # R: rotation matrix
        # (w, dist+1.15) = R * (x_gate-x_sub, y_gate-y_sub)
        # (x_sub, y_sub) = (x_gate, y_gate) - R^-1 * (w, dist+1.15)
        if self.Z_isGate:
            X_p = self.X_gate_c
        else:
            X_p = self.X_poll_t
        z = X_p[2]-self.X_camera[2]-h
        print 'z(from camera)=', z, ', z(from pressure)=', self.depth
        z = self.depth
        theta = self.theta_mag
        R = np.array([[-math.sin(theta), math.cos(theta)], [math.cos(theta), math.sin(theta)]])
        gxy = np.array([X_p[0], X_p[1]])
        xy = gxy - np.matmul(np.linalg.inv(R), np.array([w+self.X_camera[1], dist+self.X_camera[0]]))
        # went backward 1m
        xy = xy - np.array([math.cos(theta), math.sin(theta)])
        X_sensor = np.array([xy[0], xy[1], z, theta])
        # X_: from motion equation
        # X_sensor: from camera and magnetometer
        # KH: diagnal matrix, each number between 0 and 1, weight of X_sensor
        # X = (1-KH)*X_ + KH*X_sensor = X_ + KH*(X_sensor - X_)     
        self.X = self.X_ + np.matmul(self.KH, X_sensor-self.X_)
        self.X_ = np.copy(self.X)
        if self.Z_isGate:
            print 'localize observed:', self.Z_g, 'estimated:', self.toPict(self.X_gate_c, self.X_)
        else:
            print 'localize observed:', self.Z_g, 'estimated:', self.toPict(self.X_poll_t, self.X_)

    def lookAgain(self):
        # Starting point unknown
        self.KH = np.identity(4)
        self.localize()
        # mid point 1: (X_gate_c[0]+3, X_gate_c[1], X_gate_c[2], pi)
        print 'going to mid point 1'
        self.goY(-(self.X_gate_c[1]-self.X[1]))
        self.goZ(self.X_gate_c[2]-self.X[2])
        self.goX(-(self.X_gate_c[0]+3-self.X[0]), ('crossgate',))

    def crossGate(self):
        self.X = np.copy(self.X_)
        # mid point 2: (X_gate_c[0]-6, X_poll_t[1], X_gate_c[2]+1.0, pi)
        print 'going to mid point 2'
        self.goX(-(-6), None)
        self.Z_isGate = False
        self.goY(-(self.X_poll_t[1]-self.X[1]))
        self.goZ(1.0)
        self.goX(-(-3), ('look',))
        self.cmds.append(('halt', ('gox', -1.0, ('go2poll',))))

    def go2Poll(self):
        # self.KH = np.zeros((4, 4))  # without localization
        self.KH = np.identity(4)*np.array([0.5, 0.5, 0.5, 1.0])
        self.localize()
        # mid point 3: (X_poll_t[0]+3, X_poll_t[1], X_gate_c[2], pi)
        print 'going to mid point 3'
        self.goY(-(self.X_poll_t[1]-self.X[1]))
        self.goZ(self.X_gate_c[2]-self.X[2])
        self.goX(-(self.X_poll_t[0]+3-self.X[0]), ('goaroundpoll',))

    def goAroundPoll(self):
        # mid point 4: (X_poll_t[0]+3, X_poll_t[1], X_gate_c[2], pi)
        print 'going to mid point 4'
        r = 3
        n = int(2*math.pi/((self.speed/r)*self.deltaT))
        for i in range(n):
            self.cmds.append(('spin2', -self.speed, self.speed/r))
        self.cmds.append(('spin2', -self.speed, 2*math.pi-n*(self.speed/r)*self.deltaT))
        self.cmds.append(('halt',))
        # mid point 4: (X_poll_t[0]+3, X_poll_t[1], X_gate_c[2], 0)
        n = int(math.pi/(self.speed*self.deltaT))
        for i in range(n):
            self.cmds.append(('spin2', 0.0, self.speed))
        self.cmds.append(('spin2', 0.0, math.pi-n*self.speed*self.deltaT))
        self.cmds.append(('halt',))
        self.cmds.append(('halt', ('go2gate',)))
        self.Z_isGate = True

    def go2Gate(self):
        self.X = np.copy(self.X_)
        # mid point 5: (X_gate_c[0]-5, X_gate_c[1], X_gate_c[2]+1.0, 0)
        print 'going to mid point 5'
        self.goY(self.X_gate_c[1]-self.X[1])
        self.goZ(1.0)
        self.goX(self.X_gate_c[0]-5-self.X[0], ('look',))
        self.cmds.append(('halt', ('gox', -1.0, ('gohome',))))

    def goHome(self):
        # self.KH = np.zeros((4, 4))  # without localization
        self.KH = np.identity(4)*np.array([0.5, 0.5, 0.5, 1.0])
        self.localize()
        # end point: (X_gate_c[0]+5, X_gate_c[1], X_gate_c[2], 0)
        print 'going Home'
        self.goY(self.X_gate_c[1]-self.X[1])
        self.goZ(self.X_gate_c[2]-self.X[2])
        self.goX(self.X_gate_c[0]+5-self.X[0], None)

    def execute(self):
        while(not rospy.is_shutdown()):
            # handle an event if there is any
            # an event is a tuple, (id, parameter 1, parameter 2, ..., parameter n)
            # id is required, parameters are event dependent
            if len(self.evnt) > 0:
                e = self.evnt.pop(0)
                if e[0] == 'start':
                    self.start()
                elif e[0] == 'search4gate':
                    self.search4Gate()
                elif e[0] == 'gox':
                    self.goX(e[1], e[2])
                elif e[0] == 'look':
                    self.look()
                elif e[0] == 'lookagain':
                    self.lookAgain()
                elif e[0] == 'crossgate':
                    self.crossGate()
                elif e[0] == 'go2poll':
                    self.go2Poll()
                elif e[0] == 'goaroundpoll':
                    self.goAroundPoll()
                elif e[0] == 'go2gate':
                    self.go2Gate()
                elif e[0] == 'gohome':
                    self.goHome()
            # execute a command if there is any
            # a command is a tuple, (id, parameter 1, parameter 2, ..., parameter n, event to trigger)
            # id is required, parameters are command dependent, event is optional
            if len(self.cmds) == 0:
                self.halt();
            else:
                c = self.cmds.pop(0)
                # print c
                if c[0] == 'spin1':
                    self.spin1(c[1], c[2])
                    if len(c) == 4:
                        self.evnt.append(c[3])
                elif c[0] == 'spin2':
                    self.spin2(c[1], c[2])
                    if len(c) == 4:
                        self.evnt.append(c[3])
                elif c[0] == 'movex':
                    self.moveX(c[1])
                    if len(c) == 3:
                        self.evnt.append(c[2])
                elif c[0] == 'movey':
                    self.moveY(c[1])
                    if len(c) == 3:
                        self.evnt.append(c[2])
                elif c[0] == 'movez':
                    self.moveZ(c[1])
                    if len(c) == 3:
                        self.evnt.append(c[2])
                elif c[0] == 'halt':
                    self.halt()
                    if len(c) == 2:
                        self.evnt.append(c[1])
                else:
                    self.halt()
            time.sleep(self.deltaT)

def main(args):
    rospy.init_node('circle', anonymous=True)
    c = circle()
    # starting from the start event
    c.evnt.append(('start',))
    c.execute()
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
