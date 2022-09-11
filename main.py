import cv2
import numpy as np
import time
import pyzbar.pyzbar as pyzbar
from main_func import *
from time import sleep
from djitellopy import tello
import matplotlib.pyplot as plt

'''
기본 명령
drone.move_up(50)
drone.send_control_command("down {}".format(20))
drone.send_command_without_return("rc {} {} {} {}".format(a,b,c,d))
#                                       a b c d : 좌우 앞뒤 상하 yaw -100~100
'''

class drone():
    def __init__(self, t=1):
        # initialize with takeoff
        self.tstart = time.time()
        self.tello = tello.Tello()
        self.tello.connect()
        #self.tello.set_video_resolution(tello.Tello.RESOLUTION_480P)
        self.tello.streamon()
        print('battery: ', self.tello.get_battery(), '%\n')
        self.tello.send_command_without_return("rc {} {} {} {}".format(0,0,0,0))
        if t==1 : self.tello.takeoff()
        self.qrd = qr()
    def control(self, right, front, up, yaw):
        #drone.send_command_without_return("rc {} {} {} {}".format(a, b, c, d))
        #a b c d : 좌우 앞뒤 상하 yaw -100~100
        self.tello.send_command_without_return("rc {} {} {} {}".format(right, front, up, yaw))
    def control_r(self, right, front, up, yaw):
        #drone.send_command_without_return("rc {} {} {} {}".format(a, b, c, d))
        #a b c d : 좌우 앞뒤 상하 yaw -100~100
        ret = self.tello.send_control_command("rc {} {} {} {}".format(right, front, up, yaw))
        return ret
    def frame(self):
        self.frame = self.tello.get_frame_read().frame
        return self.frame
    def up(self, d):
        self.tello.move_up(d)
    def down(self, d):
        self.tello.move_down(d)
    def left(self, d):
        self.tello.move_left(d)
    def right(self, d):
        self.tello.move_right(d)
    def forward(self, d):
        self.tello.move_forward(d)
    def back(self, d):
        self.tello.move_back(d)
    def land(self):
        self.tello.land()
        self.tend = time.time() - self.tstart
        print(f'Time : {self.tend:.3f}')
    def qr(self):
        value = self.qrd.detect(self.frame)
        print('QR detected : ', value)
        return value

def main():
##### Initialize
    d = drone()
    [m1, m2, m3] = ['G', 'R', 'B']
    [q0, q1, q2, q3] = ['X', 'X', 'X', 'X']
    t0 = None
    while True:
        img = drone.get_frame_read().frame
        img_ = np.copy(img)
        img = cv2.resize(img, (360, 240))
        img = cv2.GaussianBlur(img, (3,3), 1, 1)
        # img = img.astype(np.uint8)
        cir = dict()
        img_4, img_b_r, img_b_g, img_b_b, img_c_r, img_c_g, img_c_b, cir['R'], cir['G'], cir['B'] = circle_bin_detection(img, s=100, v=130)
        #img_4, img_b_r, img_b_g, img_b_b, img_c_r, img_c_g, img_c_b, c_r, c_g, c_b = circle_bin_detection(img, s=120, v=130)
        cv2.imshow('circle', img_4)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.imwrite('fail_720p.jpg', img_)
            cv2.imwrite('fail_360p.jpg', img)
            cv2.imwrite('bin.jpg', img_4)
            break
##### Start: QR detection - Hover
        if q0 != 'O':
            qr_value = qrd.detect(img_)
            #qr_value = 'hover' ###임시
            if t0 != None:
                if (time.time() - t0) > 5:
                    q0 = 'O'
                    #sleep(1)
                    # 벽이랑 떨어져서 뒤돌기
                    drone.move_right(30)
                    drone.rotate_clockwise(100)
                    drone.move_forward(20)
                else: cv2.putText(img_4, f'hover: {time.time()-t0:.2f}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
            if t0 == None and qr_value != '':
                print('QR detected: ', qr_value)
                if qr_value == 'hover':
                    t0 = time.time()
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
##### Flag 1: Circle Detect and Follow
        if q0 == 'O' and m1 != 'O':
            com = circle_follow(cir, m1)
            if com == 'O':
                print('circle follow success')
                m1 = com
                sleep(1)
            else:
                drone.send_command_without_return(com)
##### Flag 1: QR detection and Mission
        if m1 == 'O' and q1 != 'O':
            qr_value = qrd.detect(img_)
            if qr_value != '':
                print('QR detected: ', qr_value)
                a1, a2 = mission_action(qr_value)
                print('battery: ', drone.get_battery(), '%\n')
                sleep(1)
                drone.send_control_command(a1)
                sleep(1)
                if a2 != '':
                    drone.send_control_command(a2)
                #drone.send_control_command("up {}".format(20))
                print('mission 1 complete')
                q1 = 'O'
                sleep(1)
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
##### Flag 2: Circle Detect and Follow 
        if q1 == 'O' and m2 != 'O':
            com = circle_follow(cir, m2)
            if com == 'O':
                print('circle follow success')
                m2 = com
                sleep(1)
            else:
                drone.send_command_without_return(com)
##### Flag 2: QR detection and Mission
        if m2 == 'O' and q2 != 'O':
            qr_value = qrd.detect(img_)
            if qr_value != '':
                print('QR detected: ', qr_value)
                a1, a2 = mission_action(qr_value)
                print('battery: ', drone.get_battery(), '%\n')
                sleep(1)
                drone.send_control_command(a1)
                sleep(1)
                if a2 != '':
                    drone.send_control_command(a2)
                #drone.send_control_command("up {}".format(20))
                print('mission 2 complete')
                q2 = 'O'
                sleep(1)
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
##### Flag 3: Circle Detect and Follow 
        if q2 == 'O' and m3 != 'O':
            com = circle_follow(cir, m3)
            if com == 'O':
                print('circle follow success')
                m3 = com
                sleep(1)
            else:
                drone.send_command_without_return(com)
##### Flag 3: QR detection and Mission 
        if m3 == 'O' and q3 != 'O':
            qr_value = qrd.detect(img_)
            if qr_value != '':
                print('QR detected: ', qr_value)
                a1, a2 = mission_action(qr_value)
                print('battery: ', drone.get_battery(), '%\n')
                sleep(1)
                drone.send_control_command(a1)
                sleep(1)
                if a2 != '':
                    drone.send_control_command(a2)
                print('mission 3 complete')
                q3 = 'O'
                sleep(1)
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
        if q3 == 'O':
            break
    sleep(0.5)
    drone.send_command_without_return("rc {} {} {} {}".format(0,0,0,0))
    print('\nbattery: ', drone.get_battery(), '%\n')
    drone.land()
    print("mission complete!")
    print(f"time: {time.time() - t:.2f}")
    cv2.destroyAllWindows()
##### for troubleshoot
    #plt.figure('img_original')
    #plt.imshow('img_')
    #plt.figure('hsv')
    #plt.imshow(clr_bin(img, 'X'))
    #plt.figure('bin')
    #plt.imshow(img_4)
    #plt.show()

''' 2차 예선
def main():
##### Initialize
    t = time.time()
    drone = tello.Tello()
    qrd = qr()
    drone.connect()
    # drone.set_video_resolution(tello.Tello.RESOLUTION_480P)
    drone.streamon()
    img = drone.get_frame_read().frame
    print('\nbattery: ', drone.get_battery(), '%\n')
    drone.send_command_without_return("rc {} {} {} {}".format(0,0,0,0))
    drone.takeoff()
    [m1, m2, m3] = ['G', 'R', 'B']
    [q0, q1, q2, q3] = ['X', 'X', 'X', 'X']
    t0 = None
    while True:
        img = drone.get_frame_read().frame
        img_ = np.copy(img)
        img = cv2.resize(img, (360, 240))
        img = cv2.GaussianBlur(img, (3,3), 1, 1)
        # img = img.astype(np.uint8)
        cir = dict()
        img_4, img_b_r, img_b_g, img_b_b, img_c_r, img_c_g, img_c_b, cir['R'], cir['G'], cir['B'] = circle_bin_detection(img, s=100, v=130)
        #img_4, img_b_r, img_b_g, img_b_b, img_c_r, img_c_g, img_c_b, c_r, c_g, c_b = circle_bin_detection(img, s=120, v=130)
        cv2.imshow('circle', img_4)
        key = cv2.waitKey(1)
        if key == 27:
            cv2.imwrite('fail_720p.jpg', img_)
            cv2.imwrite('fail_360p.jpg', img)
            cv2.imwrite('bin.jpg', img_4)
            break
##### Start: QR detection - Hover
        if q0 != 'O':
            qr_value = qrd.detect(img_)
            #qr_value = 'hover' ###임시
            if t0 != None:
                if (time.time() - t0) > 5:
                    q0 = 'O'
                    #sleep(1)
                    # 벽이랑 떨어져서 뒤돌기
                    drone.move_right(30)
                    drone.rotate_clockwise(100)
                    drone.move_forward(20)
                else: cv2.putText(img_4, f'hover: {time.time()-t0:.2f}', (10, 40), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))
            if t0 == None and qr_value != '':
                print('QR detected: ', qr_value)
                if qr_value == 'hover':
                    t0 = time.time()
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
##### Flag 1: Circle Detect and Follow
        if q0 == 'O' and m1 != 'O':
            com = circle_follow(cir, m1)
            if com == 'O':
                print('circle follow success')
                m1 = com
                sleep(1)
            else:
                drone.send_command_without_return(com)
##### Flag 1: QR detection and Mission
        if m1 == 'O' and q1 != 'O':
            qr_value = qrd.detect(img_)
            if qr_value != '':
                print('QR detected: ', qr_value)
                a1, a2 = mission_action(qr_value)
                print('battery: ', drone.get_battery(), '%\n')
                sleep(1)
                drone.send_control_command(a1)
                sleep(1)
                if a2 != '':
                    drone.send_control_command(a2)
                #drone.send_control_command("up {}".format(20))
                print('mission 1 complete')
                q1 = 'O'
                sleep(1)
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
##### Flag 2: Circle Detect and Follow 
        if q1 == 'O' and m2 != 'O':
            com = circle_follow(cir, m2)
            if com == 'O':
                print('circle follow success')
                m2 = com
                sleep(1)
            else:
                drone.send_command_without_return(com)
##### Flag 2: QR detection and Mission
        if m2 == 'O' and q2 != 'O':
            qr_value = qrd.detect(img_)
            if qr_value != '':
                print('QR detected: ', qr_value)
                a1, a2 = mission_action(qr_value)
                print('battery: ', drone.get_battery(), '%\n')
                sleep(1)
                drone.send_control_command(a1)
                sleep(1)
                if a2 != '':
                    drone.send_control_command(a2)
                #drone.send_control_command("up {}".format(20))
                print('mission 2 complete')
                q2 = 'O'
                sleep(1)
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
##### Flag 3: Circle Detect and Follow 
        if q2 == 'O' and m3 != 'O':
            com = circle_follow(cir, m3)
            if com == 'O':
                print('circle follow success')
                m3 = com
                sleep(1)
            else:
                drone.send_command_without_return(com)
##### Flag 3: QR detection and Mission 
        if m3 == 'O' and q3 != 'O':
            qr_value = qrd.detect(img_)
            if qr_value != '':
                print('QR detected: ', qr_value)
                a1, a2 = mission_action(qr_value)
                print('battery: ', drone.get_battery(), '%\n')
                sleep(1)
                drone.send_control_command(a1)
                sleep(1)
                if a2 != '':
                    drone.send_control_command(a2)
                print('mission 3 complete')
                q3 = 'O'
                sleep(1)
            else:
                drone.send_command_without_return("rc {} {} {} {}".format(0,0,-10,0))
        if q3 == 'O':
            break
    sleep(0.5)
    drone.send_command_without_return("rc {} {} {} {}".format(0,0,0,0))
    print('\nbattery: ', drone.get_battery(), '%\n')
    drone.land()
    print("mission complete!")
    print(f"time: {time.time() - t:.2f}")
    cv2.destroyAllWindows()
##### for troubleshoot
    #plt.figure('img_original')
    #plt.imshow('img_')
    #plt.figure('hsv')
    #plt.imshow(clr_bin(img, 'X'))
    #plt.figure('bin')
    #plt.imshow(img_4)
    #plt.show()
'''

if __name__ == '__main__':
    main()
