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
    yolo = YOLO()
    debug = 1
    takeoff = 0
    drone = drone_tello(takeoff=takeoff, debug=debug)
    flag = ['K', 'R', 'B']
    hw = ['X', 'X', 'X']
    track = 'X'
    t = {'Ht':False, 'H':True, 'Hq': True, 'F1':False, 'H1':False, 'F2':False, 'H2':False, 'F3':False, 'H3':False, 'T':False, 'L':False}
    while True:
        img = drone.get_frame()
        #cv2.imshow('original', drone.frame_)
        show_bin = t['H']+t['F1']+t['H1']+t['F2']+t['H2']+t['F3']+t['H3']
        if show_bin == 1: 
            img_b, img_c, cir = circle_bin_detection(img, flag=flag, s=100, v=130)
### 1. QR detection - hover
        if t['H']:
            if t['Ht']:
                if time.time() - t['Ht'] > 5: 
                    print('hover 5s success')
                    t['H'] = False
                    t['F1'] = True
                else: 
                    img_b['4'] = puttext(img_b['4'], f"hover: {time.time()-t['Ht']:.2f}", (10, 40))
            elif drone.qr() == 'hover':
                if t['Hq']: 
                    t['Ht'] = time.time()
                    drone.control()
                    t['Ht']=True
            else: drone.control(right=20, down=10) # control 
### 2.1 Circle follow and read number - black
        if t['F1']:
            com = circle_follow(cir, flag[0])
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
                t['H1'] = False
                t['F2'] = True
            else: drone.control(down=10)
### 2.2 Circle follow and read number - red
        if t['F2']:
            com = circle_follow(cir, flag[1])
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
                t['H2'] = False
                t['F3'] = True
            else: drone.control(down=10)
### 2.3 Circle follow and read number - blue
        if t['F3']:
            com = circle_follow(cir, flag[2])
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
                t['H3'] = False
                t['T'] = True
            else: drone.control(down=10)
### 3. Image tracking 1 - KAU
        if t['T']:
            df, drone.frame_ = yolo.detect_object(drone.frame_)
            com = object_follow(df, 'KAU')
            if com == True:
                t['T'] = False
                t['L'] = True
            else: drone.com(com)
### 4. Image tracking 2 - plane
        if t['L']:
            df, cor, drone.frame_ = yolo.detect_object(drone.frame_)
            com = object_follow(df, 'F22')
            if com == True:
                t['L'] = False
                break
            else: drone.com(com)
### Visualize
        if show_bin == 1: 
            cv2.imshow('mission', img_b['4'])
        else: cv2.imshow('mission', drone.frame_) #change to yolo
        key = cv2.waitKey(1)
        
### Abort
        if key == 27:
            cv2.imwrite('fail_720p.jpg', drone.frame_)
            cv2.imwrite('fail_360p.jpg', img)
            cv2.imwrite('fail_bin.jpg', img_b['4'])
            break
# Land
    drone.land()
    print("mission complete!")
    cv2.destroyAllWindows()
# Debug
    if drone.debug == 1:
        plt.figure('img_original')
        plt.imshow(drone.frame_)
        plt.figure('hsv')
        plt.imshow(clr_bin(drone.frame_, 'X'))
        plt.figure('bin')
        plt.imshow(img_b['4'])
        plt.show()

if __name__ == '__main__':
    main()
