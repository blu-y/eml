#-*- coding: utf-8 -*-
import cv2
import numpy as np
import time
from main_func import *
import matplotlib.pyplot as plt

def main():
# Initialize
    #fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    #rec = cv2.VideoWriter('rec.avi', fourcc, 15, (h, w))
    yolo = YOLO('num.pt')
    pid = PID()
    takeoff = 1
    drone = drone_tello(takeoff=takeoff)
    drone.control()
    drone.back(40)
    drone.control()
    flag = ['K', 'R', 'B']
    hw = ['X', 'X', 'X']
    track = 'X'
    t = {'Ht':False, 'H':True, 'Hq': True, 'F1':False, 'H1':False, 'F2':False, 'H2':False, 'F3':False, 'H3':False, 'T':False, 'L':False}
    #t = {'Ht':False, 'H':False, 'Hq': True, 'F1':False, 'H1':False, 'F2':False, 'H2':False, 'F3':False, 'H3':True, 'T':False, 'L':False}
    while True:
        img = drone.get_frame()
        #cv2.imshow('original', drone.frame_)
        show_bin = t['H']+t['F1']+t['F2']+t['F3']
        if show_bin == 1: 
            img_b, img_c, cir = circle_bin_detection(img, flag=flag, s=100, v=130)
### 1. QR detection - hover 
        if t['H']:
            if t['Ht']:
                if time.time() - t['Ht'] > 5: 
                    print('hover 5s success')
                    drone.control()
                    t['H'] = False
                    t['F1'] = True
                else: 
                    img_b['4'] = puttext(img_b['4'], f"hover: {time.time()-t['Ht']:.2f}", (10, 40))
            elif drone.qr() == 'hover':
                if t['Hq']: 
                    t['Ht'] = time.time()
                    drone.control()
                    t['Hq']=False
            else: drone.control(back=5)
### 2.1 Circle follow and read number - black
        if t['F1']:
            com = pid.circle(cir, flag[0], img_b['4'])
            if com == True:
                print('circle follow success')
                cv2.imwrite('flag1.png', img_b['4'])
                drone.control()
                t['F1'] = False
                t['H1'] = True
            else: drone.com(com)
        if t['H1']:
            ret, letter, drone.frame_ = yolo.detect_letter(drone.frame_)
            if ret:
                action(drone, letter)
                drone.control()
                t['H1'] = False
                t['F2'] = True
            else: drone.control(down=10)
### 2.2 Circle follow and read number - red
        if t['F2']:
            com = pid.circle(cir, flag[1], img_b['4'])
            if com == True:
                print('circle follow success')
                cv2.imwrite('flag2.png', img_b['4'])
                drone.control()
                t['F2'] = False
                t['H2'] = True
            else: drone.com(com)
        if t['H2']:
            ret, letter, drone.frame_ = yolo.detect_letter(drone.frame_)
            if ret:
                action(drone, letter)
                drone.control()
                t['H2'] = False
                t['F3'] = True
            else: drone.control(down=10)
### 2.3 Circle follow and read number - blue
        if t['F3']:
            com = pid.circle(cir, flag[2], img_b['4'])
            if com == True:
                print('circle follow success')
                cv2.imwrite('flag3.png', img_b['4'])
                drone.control()
                t['F3'] = False
                t['H3'] = True
            else: drone.com(com)
        if t['H3']:
            ret, letter, drone.frame_ = yolo.detect_letter(drone.frame_)
            if ret:
                action(drone, letter)
                drone.control()
                t['H3'] = False
                t['T'] = True
                pid.set_center(480, 360, 250) #size 설정 필요
                yolo = YOLO('kau0.pt')
            else: drone.control(down=10)
### 3. Image tracking 1 - KAU
        if t['T']:
            fr = np.copy(drone.frame_)
            df, drone.frame_ = yolo.detect_object(drone.frame_)
            com = pid.object(df, fr, "'KAU'")
            #print(com)
            if com == True:
                yolo = YOLO('plane.pt')
                cv2.imwrite('KAUyolo.jpg', drone.frame_)
                drone.right(50)
                drone.ccw(30)
                t['T'] = False
                t['L'] = True
                t['Kt'] = time.time()
            else: drone.com(com)
### 4. Image tracking 2 - plane
        if t['L']:
            fr = np.copy(drone.frame_)
            df, drone.frame_ = yolo.detect_object(drone.frame_)
            com = pid.plane(df, drone.frame_)
            if com == True or (time.time()-t['Kt']) > 15:
                drone.control()
                drone.forward(150)
                time.sleep(5)
                t['L'] = False
                break
            else: drone.com(com)
### Visualize
        if show_bin == 1: 
            cv2.imshow('mission', img_b['4'])
            #(480, 720, 3)
        else: 
            drone.frame_ = cv2.resize(drone.frame_, (720, 480))
            cv2.imshow('mission', drone.frame_) #change to yolo
        key = cv2.waitKey(1)
### Abort
        if key == 27:
            cv2.imwrite('fail_720p.jpg', drone.frame_)
            cv2.imwrite('fail_360p.jpg', drone.frame)
            cv2.imwrite('fail_bin.jpg', img_b['4'])
            break
# Land
    drone.land()
    print("mission complete!")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
