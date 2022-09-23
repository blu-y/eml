#-*- coding: utf-8 -*-
import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar
from djitellopy import tello
import time
import matplotlib.pyplot as plt
import torch
import pandas as pd
import os
from os import path
import sys
#sys.path.append(path(__file__))

'''
기본 명령
drone.move_up(50)
drone.send_control_command("down {}".format(20))
drone.send_command_without_return("rc {} {} {} {}".format(a,b,c,d))
#                                       a b c d : 좌우 ?��?�� ?��?�� yaw -100~100
'''

def puttext(img, txt, org, clr=(0,0,0)):
    cv2.putText(img, txt, org, cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
    return img

class qr():
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
    def detect(self, img):
        qrimg = np.copy(img)
        qr_value = ''
        for code in pyzbar.decode(qrimg):
            qr_value = code.data.decode('utf-8')
        if qr_value == '': qr_value, bbox, qrimg = self.detect_cv(qrimg)
        #if qr_value != '': print('QR detected: ', qr_value)
        return qr_value
    def detect_cv(self, img):
        qrimg = np.copy(img)
        try:
            value, bbox, _ = self.detector.detectAndDecode(img)
            bbox = bbox.astype(int)
            cv2.drawContours(qrimg, bbox, 0,(0,255,255), 3)
            return value, bbox, qrimg
        except: return '', '', qrimg

class drone_tello():
    def __init__(self, takeoff = 1, debug = 0):
        '''
        Tello drone initialize:
        '''
        self.tstart = time.time()
        self.tello = tello.Tello()
        self.tello.connect()
        #self.tello.set_video_resolution(tello.Tello.RESOLUTION_480P)
        self.tello.streamon()
        print('battery: ', self.tello.get_battery(), '%\n')
        self.tello.send_command_without_return("rc {} {} {} {}".format(0,0,0,0))
        self.takeoff = takeoff
        self.debug = debug
        if self.takeoff == 1 : self.tello.takeoff()
        self.qrd = qr()
    def control(self, left=0, right=0, front=0, back=0, up=0, down=0, yaw=0):
        #drone.send_command_without_return("rc {} {} {} {}".format(a, b, c, d))
        #a b c d : 좌우 ?��?�� ?��?�� yaw -100~100
        if self.takeoff == 1: 
            self.tello.send_command_without_return("rc {} {} {} {}".format(right-left, front-back, up-down, yaw))
    def com(self, command):
        if self.takeoff == 1: 
            self.tello.send_command_without_return(command)
    def control_r(self, left=0, right=0, front=0, back=0, up=0, down=0, yaw=0):
        #drone.send_control_command("rc {} {} {} {}".format(a, b, c, d))
        #a b c d : 좌우 ?��?�� ?��?�� yaw -100~100
        if self.takeoff == 1: 
            return self.tello.send_control_command("rc {} {} {} {}".format(right-left, front-back, up-down, yaw))
        else: return False
    def get_frame(self):
        self.frame_ = self.tello.get_frame_read().frame
        self.frame = cv2.resize(self.frame_, (360, 240))
        #self.frame = cv2.GaussianBlur(self.frame, (3,3), 1, 1)
        return self.frame
    def up(self, d):
        if self.takeoff == 1: self.tello.move_up(d)
    def down(self, d):
        if self.takeoff == 1: self.tello.move_down(d)
    def left(self, d):
        if self.takeoff == 1: self.tello.move_left(d)
    def right(self, d):
        if self.takeoff == 1: self.tello.move_right(d)
    def forward(self, d):
        if self.takeoff == 1: self.tello.move_forward(d)
    def back(self, d):
        if self.takeoff == 1: self.tello.move_back(d)
    def land(self):
        if self.takeoff == 1: self.tello.land()
        self.tend = time.time()
        self.time = self.tend - self.tstart
        print(f'Time : {self.time:.3f}')
        print('battery: ', self.tello.get_battery(), '%\n')
        if self.debug == 1:
            plt.figure('frame')
            plt.imshow(self.frame)
            plt.figure('original')
            plt.imshow(self.frame_)
            plt.figure('hsv')
            plt.imshow(cv2.cvtColor(self.frame_, cv2.COLOR_BGR2HSV))
            plt.show()
    def qr(self, show = 1):
        value = self.qrd.detect(self.frame_)
        if value != '':
            if show == 1: print('QR detected : ', value)
        return value
    def hw(self, show = 1):
        value = self.hwd.detect(self.frame_)
        if show == 1: print('Number detected : ', value)
        return value

def imageGP(img):
    qrd=qr()
    src = np.copy(img)
    height, width, channel = src.shape

    srcPoint1 = np.array([[0, 0], [width-200, 0], [width-200, height], [0, height]], dtype=np.float32)
    srcPoint2 = np.array([[0, 0], [width - 100, 0], [width - 100, height], [0, height]], dtype=np.float32)
    srcPoint3 = np.array([[200, 0], [width, 0], [width, height], [200, height]], dtype=np.float32)
    srcPoint4 = np.array([[100, 0], [width, 0], [width, height], [100, height]], dtype=np.float32)
    dstPoint = np.array([[0, 0], [width, 0], [width, height], [0, height]], dtype=np.float32)

    matrix1 = cv2.getPerspectiveTransform(srcPoint1, dstPoint)
    dst1 = cv2.warpPerspective(src, matrix1, (width, height))
    matrix2 = cv2.getPerspectiveTransform(srcPoint2, dstPoint)
    dst2 = cv2.warpPerspective(src, matrix2, (width, height))
    matrix3 = cv2.getPerspectiveTransform(srcPoint3, dstPoint)
    dst3 = cv2.warpPerspective(src, matrix3, (width, height))
    matrix4 = cv2.getPerspectiveTransform(srcPoint4, dstPoint)
    dst4 = cv2.warpPerspective(src, matrix4, (width, height))
    
    qr_value = qrd.detect(src)
    if qr_value != '':
        return qr_value
    qr_value = qrd.detect(dst1)
    if qr_value != '':
        return qr_value
    qr_value = qrd.detect(dst2)
    if qr_value != '':
        return qr_value
    qr_value = qrd.detect(dst3)
    if qr_value != '':
        return qr_value
    qr_value = qrd.detect(dst4)
    if qr_value != '':
        return qr_value
    return ''    

def clr(img, pts, r):
    pix = np.uint8([[img[pts[1],pts[0]]]])
    hsv = cv2.cvtColor(pix, cv2.COLOR_BGR2HSV).squeeze()
    #print(hsv)
    h = hsv[0]
    #s = hsv[1]
    v = hsv[2]
    if v > 100: return 'X'
    if (h < r or h > 180-r): return 'R' #and s < 210+r and s > 210-r and v < 140+r and v > 140-r
    if h < 68+r and h > 68-r: return 'G' #and s < 138+r and s > 138-r and v < 90+r and v > 90-r
    if h < 108+r and h > 108-r: return 'B' #and s < 240+r and s > 240-r and v < 115+r and v > 115-r
    return 'X'

def clr_bin(img, clr, r=20, s=150, v=100):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    img_h = hsv[:,:,0]
    img_s = hsv[:,:,1]
    img_v = hsv[:,:,2]
    img_b_r = cv2.inRange(img_h, 180-r, 180)
    img_b_r = img_b_r + cv2.inRange(img_h, 0, r)
    img_b_r[img_v>v+40] = 0 #+40??? ?��?�� ?��?��
    img_b_r[img_s<s+60] = 0
    img_b_g = cv2.inRange(img_h, 80-r, 80+r)
    img_b_g[img_v>v] = 0
    img_b_g[img_s<s] = 0
    img_b_b = cv2.inRange(img_h, 120-r, 120+r)
    img_b_b[img_v>v] = 0
    img_b_b[img_s<s] = 0
    # black �?�? ?��?�� ?��?��
    img_b_k = cv2.inRange(img_v, 0, 30)
    if clr == 'R': return img_b_r
    if clr == 'G': return img_b_g
    if clr == 'B': return img_b_b
    if clr == 'K': return img_b_k
    return hsv

def circle(img, img_bin):
    img_c = np.copy(img)
    #img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    c = cv2.HoughCircles(img_bin, cv2.HOUGH_GRADIENT, 1, 20)
    #print(c)
    if type(c) == type(None):
        return img_c, c
    c = np.uint16(np.around(c))
    if c[0][0].ndim == 1:
        for i in c[0,:]:
            cv2.circle(img_c,(i[0],i[1]),i[2],(0,255,0),2)
            cv2.circle(img_c,(i[0],i[1]),2,(0,0,255),3)
    return img_c, c

def box_det(img, r=20):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, img_bin = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU) # white+else so, binary
    cnt, _ = cv2.findContours(img_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    img_con = np.copy(img)
    rect = []
    color = []
    cent = []
    area = img.shape[0]*img.shape[1]
    for pts in cnt:
        if cv2.contourArea(pts)<2000 or cv2.contourArea(pts)>area*0.8:
            continue
        eps = 0.05*cv2.arcLength(pts, True)
        app = cv2.approxPolyDP(pts, eps, True)
        cor = len(app)
        if cor == 4: # rectangular
            cent = np.mean(app, axis=0).astype(np.uint16).squeeze()
            color_ = clr(img,cent, r)
            cv2.drawContours(img_con, [app], 0,(0,255,255), 3)
            if color_ != 'X':
                cv2.circle(img_con, cent, 5, (0,255,255), 2)
                cv2.putText(img_con, clr(img,cent, r), cent+10, cv2.FONT_HERSHEY_PLAIN, 1, (0,255,255))
                rect.append(cent)
                color.append(color_)
    #cv2.imshow('Bin', img_bin)
    #cv2.imshow('Con', img_con)
    #cv2.imshow('canny', canny)
    if len(color) == 0:
        color = None
    else: color = color[0]
    return cent, color, img_con

def blob(image):
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 200
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.filterByConvexity = False
    params.minConvexity = 0.95
    params.filterByInertia = True
    params.minInertiaRatio = 0.3
    
    image = 255 - image
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(image)
    image_with_keypoints = cv2.drawKeypoints(image , keypoints , np.array([]) , (0 , 0 , 255) , cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #print(keypoints)
    #cv2.imshow('all blobs' , image_with_keypoints)
    #cv2.imwrite('all_blobs.jpg' , image_with_keypoints)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    circle = []
    for kp in keypoints:
        circle.append([round(kp.pt[0]),round(kp.pt[1]),round(kp.size)])

    """
    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(image)
    image_with_keypoints = cv2.drawKeypoints(image , keypoints , np.array([]) , (0 , 0 , 255) , cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow('With red circular blobs' , image_with_keypoints)
    #cv2.imwrite('circular blobs.jpg' , image_with_keypoints)
    #cv2.waitKey()
    #cv2.destroyAllWindows()
    """
    return image_with_keypoints, circle, keypoints

def circle_bin_detection(img, r=20, s=150, v=130, flag = ['K', 'R', 'B']):
    img_b = dict()
    img_c = dict()
    cir = dict()
    img_4 = img
    for i in flag:
        img_b[i] = clr_bin(img, i, r, s, v)
        img_c[i], cir[i], _ = blob(img_b[i])
    img_4 = np.vstack((np.hstack((img_4, img_c[flag[0]])), np.hstack((img_c[flag[1]], img_c[flag[2]]))))
    cv2.putText(img_4, 'Original', (10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
    cv2.putText(img_4, flag[0], (img.shape[1]+10, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
    cv2.putText(img_4, flag[1], (10, img.shape[0]+20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
    cv2.putText(img_4, flag[2], (img.shape[1]+10, img.shape[0]+20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
    img_b['4'] = img_4
    return img_b, img_c, cir

def calc(formula):
    num1 = int(formula[0])
    num2 = int(formula[2])
    oper = formula[1]
    if oper == '+':
        return num1+num2
    elif oper == '-':
        return num1-num2
    elif oper == '*':
        return num1*num2
    elif oper == '/':
        return num1/num2
    else: return 'X'
    
def mission_action(i):
    #i = calc(s) # string >> calculating
    a1 = ""
    a2 = ""
    a3 = ""
    a4 = ""
    if i == 1:
        a1 = "up {}".format(30)
        a2 = "down {}".format(30)
        print('answer = 1, up 30, down 30')
    if i == 2:
        a1 = "back {}".format(30)
        a2 = "flip {}".format("f")
        print('answer = 2, flip forward')
    if i == 3:
        a1 = "down {}".format(30)
        a2 = "up {}".format(30)
        print('answer = 3, down 30, up 30')
    if i == 4:
        a1 = "right {}".format(20)
        a2 = "flip {}".format("l")   
        print('answer = 4, flip left')
    if i == 5:
        a1 = "cw {}".format(360)   
        print('answer = 5, clockwise 360')
    return a1, a2

def action(drone, i):
    if drone.takeoff == 1:
        if i == 1:
            print('back 30, front 30')
            drone.com("back {}".format(30))
            drone.com("front {}".format(30))
        if i == 2:
            print('left 30, right 30')
            drone.com("left {}".format(30))
            drone.com("right {}".format(30))
        if i == 3:
            print('clockwise 360')
            drone.com("cw {}".format(360))
        if i == 4:
            print('right 10, up 10, left 10, down 10') # ?���? ?��?��?
            drone.com("right {}".format(30))
            drone.com("up {}".format(30))
            drone.com("left {}".format(30))
            drone.com("down {}".format(30))
        if i == 5:
            print('flip backward')
            drone.com("flip {}".format("b"))
        if i == 6:
            print('up 30, flip, down 30')
            drone.com("up {}".format(30))
            drone.com("flip {}".format("b"))
            drone.com("down {}".format(30))
        if i == 7:
            print('flip left')
            drone.com("flip {}".format("l"))
        if i == 8:
            print('up 30, down 30')
            drone.com("up {}".format(30))
            drone.com("down {}".format(30))
        if i == 9:
            print('capture picture')
            cv2.imwrite('9.png', drone.frame_)
            print('9.png saved')

def mission(num, drone):
    if num == 1:
        drone.move_up(30)
        drone.move_down(30)
    elif num == 2:
        drone.flip_forward()
    elif num == 3:
        drone.move_down(30)
        drone.move_up(30)
    elif num == 4:
        drone.flip_left()
    elif num == 5:
        drone.rotate_clockwise(360)
    else: return 'X'

def circle_follow(cir, clr):
    #global al
    if len(cir[clr]) == 0:
        return "rc {} {} {} {}".format(0,0,0,50)
    if cir[clr][0][2] > 60:
        print(cir[clr], clr, 'detected')
    if abs(cir[clr][0][0]-180)>15:
        return "rc {} {} {} {}".format((cir[clr][0][0]-180)/2.6,0,0,0)
    if abs(cir[clr][0][1]-100)>30:
        return "rc {} {} {} {}".format(0,0,(100-cir[clr][0][1])/2,0)
    if cir[clr][0][2] < 70:#40:
        return "rc {} {} {} {}".format(0,(80-cir[clr][0][2])/1.5,0,0)
    return True

class YOLO():
    def __init__(self):
        #self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolo.pt', force_reload=False)
        self.model = torch.load('yolo.pt')
    def detect_letter(self, img):
        yolo = self.model(img)
        df = yolo.pandas().xyxy[0]
        img = yolo.render()[0]
        if df.shape[0]>0:
            ret = True
            letter = df['name'] # ?��?�� ?��?��
            print('letter = \n', letter)
            letter = int(letter[0])
        else: 
            ret = False
            letter = None
        return ret, letter, img
    def detect_object(self, img):
        yolo = self.model(img)
        df = yolo.pandas().xyxy[0]
        yolo.display(render=True)
        return df, img

def object_follow(df, label):
    x = df[label]['x']
    y = df[label]['y']
    size = None
    size = df[label]['size']
    if size == None:
        return "rc {} {} {} {}".format(0,0,0,50)
    if size > 60:
        print(label, 'detected')
    if abs(x-180)>15:
        return "rc {} {} {} {}".format((x-180)/2.6,0,0,0)
    if abs(y-100)>30:
        return "rc {} {} {} {}".format(0,0,(100-y)/2,0)
    if size < 70:#40:
        return "rc {} {} {} {}".format(0,(80-size)/1.5,0,0)
    return True
