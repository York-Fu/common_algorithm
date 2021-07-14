#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import cv2
import numpy as np 

class ColorObject:
    def __init__(self, lower, upper, name='none'):
        self.lower_hsv = lower
        self.upper_hsv = upper
        self.obj = {'name': name, 'box_x': 0, 'box_y': 0, 'box_w': 0, 'box_h': 0, 'center_x': 0, 'center_y': 0}
        
    def show_img(self, win_name, img):
        cv2.imshow(win_name, img)
        k = cv2.waitKey(1)
        # if k == 27:
        #     cv2.destroyWindow()

    def detect(self, image, area_max=False):
        blurred = cv2.GaussianBlur(image, (5, 5), 0)  # 高斯模糊
        hsv_img = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV) # 转换颜色空间到HSV
        mask = cv2.inRange(hsv_img, self.lower_hsv, self.upper_hsv) # 对图片进行二值化处理
        mask = cv2.dilate(mask, None, iterations=2) # 膨胀操作
        mask = cv2.erode(mask, None, iterations=2) # 腐蚀操作
        contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]  # 寻找图中轮廓

        result = []
        if area_max == True:
            if len(contours) > 0: # 如果存在至少一个轮廓则进行如下操作
                contour = max(contours, key=cv2.contourArea) # 找到面积最大的轮廓
                box_x, box_y, box_w, box_h = cv2.boundingRect(contour)
                tmp_area = float(box_w * box_h) / float(mask.shape[0] * mask.shape[1])
                if tmp_area >= 0.02:
                    self.obj['box_x'], self.obj['box_y'], self.obj['box_w'],  self.obj['box_h'] = box_x, box_y, box_w, box_h
                    M = cv2.moments(contour)
                    self.obj['center_x'] = int(M['m10'] / M['m00'])
                    self.obj['center_y'] = int(M['m01'] / M['m00'])
                    result.append(self.obj)
        else:
            for contour in contours:
                box_x, box_y, box_w, box_h = cv2.boundingRect(contour)
                tmp_area = float(box_w * box_h) / float(mask.shape[0] * mask.shape[1])
                if tmp_area >= 0.02:
                    self.obj['box_x'], self.obj['box_y'], self.obj['box_w'],  self.obj['box_h'] = box_x, box_y, box_w, box_h
                    M = cv2.moments(contour)
                    self.obj['center_x'] = int(M['m10'] / M['m00'])
                    self.obj['center_y'] = int(M['m01'] / M['m00'])
                    result.append(self.obj)

        return self.result

    def put_visualization(self, image, result):
        if len(result) > 0:
            cv2.circle(image, center=(result[0]['center_x'], result[0]['center_y']), radius=1, color=(0, 0, 255), thickness=-1)
            x1, y1 = result[0]['box_x']+result[0]['box_w'], result[0]['box_y']+result[0]['box_h']
            cv2.rectangle(image, (result[0]['box_x'],result[0]['box_y']), (x1, y1), (0, 0, 255), 1)
            if result['name'] != 'none':
                cv2.putText(image, result['name'], (result[0]['box_x'], result[0]['box_y']-2), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

if __name__ == '__manin__':

    def put_text_info(img,fps,time):
        x, y = 10, 20
        text = 'fps: ' + '%5.2f'%fps
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        y = y + 30
        text = 'time: '+'%5.2f'%(time)
        cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    # HSV阈值
    lowerOrange = np.array([15, 100, 100])
    upperOrange = np.array([25, 255, 255])
    lowerCyan = np.array([80, 100, 100])
    upperCyan = np.array([95, 255, 255])
    lowerGreen = np.array([35, 100, 100])
    upperGreen = np.array([75, 255, 255])
    lowerRed = np.array([0, 224, 96])
    upperRed = np.array([16, 255, 240])
    # creat object
    ball = ColorObject(lowerRed, upperRed)
