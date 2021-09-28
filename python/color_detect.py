#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2 as cv
import math
import time


def rgb_to_hsv(rgb):
    """
    convert rgb to hsv value
    """
    rgb_pixel = np.uint8([[rgb]])
    hsv = cv.cvtColor(rgb_pixel, cv.COLOR_RGB2HSV)
    return hsv[0][0]


def color_range(hsv, delta=[0.1, 0.2, 0.6]):
    """
    color hsv upper and lower boundary
    """
    delta_h = int(delta[0] * 0.5 * 180.0)  # h
    delta_s = int(delta[1] * 0.5 * 255.0)  # s
    hsv_l = [hsv[0] - delta_h, hsv[1] - delta_s, int((1-delta[2]) * 0.5 * 255)]
    hsv_h = [hsv[0] + delta_h, hsv[1] + delta_s, int((1 - ((1-delta[2]) * 0.5)) * 255)]
    for i, v in enumerate(hsv_l):
        hsv_l[i] = hsv_l[i] if hsv_l[i] > 0 else 0
    for i, v in enumerate(hsv_h):
        hsv_h[i] = hsv_h[i] if hsv_h[i] < 255 else 255
    hsv_h[0] = hsv_h[0] if hsv_h[0] < 180 else hsv_h[0]
    return np.array(hsv_l), np.array(hsv_h)


class ColorDetect:
    def __init__(self, hsv, name='none'):
        self.lower_hsv = hsv[0]
        self.upper_hsv = hsv[1]
        self.obj_info = {'name': name, 'box_x': 0, 'box_y': 0, 'box_w': 0, 'box_h': 0, 'center': [0, 0]}

    def set_hsv_range(self, hsv):
        self.lower_hsv = hsv[0]
        self.upper_hsv = hsv[1]

    def detect(self, image, area_max=False):
        blurred = cv.GaussianBlur(image, (5, 5), 0)  # 高斯模糊
        hsv_img = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv_img, self.lower_hsv, self.upper_hsv)
        mask = cv.dilate(mask, None, iterations=2)  # 膨胀
        mask = cv.erode(mask, None, iterations=2)  # 腐蚀
        contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2]  # 寻找图中轮廓

        result = []
        if area_max == True:
            if len(contours) > 0:  # 如果存在至少一个轮廓
                contour = max(contours, key=cv.contourArea)  # 找到面积最大的轮廓
                box_x, box_y, box_w, box_h = cv.boundingRect(contour)
                tmp_area = float(box_w * box_h) / float(mask.shape[0] * mask.shape[1])
                if tmp_area >= 0.01:
                    self.obj_info['box_x'], self.obj_info['box_y'], self.obj_info['box_w'], self.obj_info['box_h'] = box_x, box_y, box_w, box_h
                    M = cv.moments(contour)
                    self.obj_info['center'] = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]
                    result.append(self.obj_info)
        else:
            for contour in contours:
                box_x, box_y, box_w, box_h = cv.boundingRect(contour)
                tmp_area = float(box_w * box_h) / float(mask.shape[0] * mask.shape[1])
                if tmp_area >= 0.01:
                    self.obj_info['box_x'], self.obj_info['box_y'], self.obj_info['box_w'], self.obj_info['box_h'] = box_x, box_y, box_w, box_h
                    M = cv.moments(contour)
                    self.obj_info['center'] = [int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])]
                    result.append(self.obj_info)

        return result


def put_visualization(image, result):
    img = image.copy()
    for res in result:
        cv.circle(img, center=(res['center'][0], res['center'][1]), radius=1, color=(0, 0, 255), thickness=-1)
        x1, y1 = res['box_x']+res['box_w'], res['box_y']+res['box_h']
        cv.rectangle(img, (res['box_x'], res['box_y']), (x1, y1), (0, 0, 255), 1)
        if res['name'] != 'none':
            cv.putText(img, res['name'], (res['box_x'], res['box_y']-2), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return img


if __name__ == "__main__":
    import rospy
    import image_manager as img

    rospy.init_node("image_detect")
    image = img.ImageManager()

    hsv = (0, 51, 51), (15, 204, 204)
    turntable = ColorDetect(hsv)
    hsv = color_range(rgb_to_hsv([5, 70, 90]), [0.1, 0.4, 0.7])
    pendulum_bob = ColorDetect(hsv)

    while not rospy.is_shutdown():
        result = turntable.detect(image.chin_image)
        result = pendulum_bob.detect(image.eye_image)
        cv.imshow("image detect", put_visualization(image.eye_image, result))
        cv.waitKey(1)
