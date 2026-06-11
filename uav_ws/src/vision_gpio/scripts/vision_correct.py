#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time

# 相机参数
# 本节点保留旧视觉节点的 x/y 对应关系：
#   dx = (pickup_x - obj_x) * VISION_SCALE / fx
#   dy = (pickup_y - obj_y) * VISION_SCALE / fy
#   msg_point.x = dy
#   msg_point.y = dx
#
# 与旧控制节点保持一致：
#   target_x = current_x - object_offset.x
#   target_y = current_y - object_offset.y
#
# 不再使用 boundingRect 宽度反推视觉 Z，避免目标轮廓宽度抖动污染 XY。
# /camera/object_offset.z 现在只表示目标面积，用作识别质量参考。

camera_matrix = np.array([
    [506.142358, 0, 308.46348627],
    [0, 503.54022763, 229.74875316],
    [0, 0, 1]
])
dist_coeffs = np.array([0.06507472, -0.20553698, -0.00187582, -0.00172221, 0.21452783])

# 固定视觉换算尺度，约等于你之前日志中视觉 Z 的 0.43~0.48m 区间
VISION_SCALE = 0.38

# 视觉输出一阶低通权重：越小越稳，越大越灵敏
ALPHA = 0.25

# 面积阈值，过滤小噪点
MIN_AREA = 800

# 丢失多少帧后重置滤波，避免重新识别时带入旧目标历史值
MISS_RESET_FRAMES = 3

# 上一时刻滤波数据
last_body_x = None
last_body_y = None


def main():
    global last_body_x, last_body_y

    rospy.init_node('vision_error_node')

    pub_point = rospy.Publisher('/camera/object_offset', Point, queue_size=10)
    pub_detected = rospy.Publisher('/camera/object_detected', Bool, queue_size=10)
    rate = rospy.Rate(30)

    cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
    if not cap.isOpened():
        rospy.logerr("Cannot open camera")
        return

    rospy.loginfo("Vision node started: fixed-scale XY output, old x/y mapping preserved.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 关闭自动曝光
    cap.set(cv2.CAP_PROP_EXPOSURE, 700)      # 手动曝光值

    frame_count = 0
    last_time = time.time()
    miss_count = 0

    kernel = np.ones((5, 5), np.uint8)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logwarn("Camera read failed")
            msg_detected = Bool()
            msg_detected.data = False
            pub_detected.publish(msg_detected)
            rate.sleep()
            continue

        frame_count += 1
        now = time.time()
        if now - last_time >= 1.0:
            frame_count = 0
            last_time = now

        frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

        height, width = frame.shape[:2]
        cx = width / 2.0
        cy = height / 2.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 红色 HSV 范围，两段相加
        lower_red1 = np.array([0, 43, 56])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 43, 56])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 形态学处理，减少小噪点和破碎轮廓
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0.0
        best_cnt = None

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < MIN_AREA:
                continue
            if area > max_area:
                max_area = area
                best_cnt = cnt

        detected = False

        if best_cnt is not None:
            detected = True
            miss_count = 0

            x, y, w, h = cv2.boundingRect(best_cnt)

            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]

            # 保留旧节点习惯：使用 boundingRect 中心，避免改变你已经验证过的方向对应关系
            obj_x = x + w / 2.0
            obj_y = y + h / 2.0

            pickup_x = cx
            pickup_y = cy

            # 固定尺度换算为米制 XY 偏差，不再使用视觉 Z
            dx = (pickup_x - obj_x) * VISION_SCALE / fx
            dy = (pickup_y - obj_y) * VISION_SCALE / fy

            # 保持旧节点最终发布方向：
            # 旧节点最终 msg_point.x = -body_x = dy
            # 旧节点最终 msg_point.y = -body_y = dx
            body_x = dy
            body_y = dx

            # 限制异常跳变，避免误检时给控制端过大的视觉误差
            body_x = float(np.clip(body_x, -0.20, 0.20))
            body_y = float(np.clip(body_y, -0.20, 0.20))

            # 一阶低通滤波
            if last_body_x is None:
                last_body_x = body_x
                last_body_y = body_y
            else:
                body_x = ALPHA * body_x + (1.0 - ALPHA) * last_body_x
                body_y = ALPHA * body_y + (1.0 - ALPHA) * last_body_y
                last_body_x = body_x
                last_body_y = body_y

            msg_point = Point()
            msg_point.x = body_x
            msg_point.y = body_y
            msg_point.z = max_area     # z 只表示目标面积/识别质量，不表示深度
            pub_point.publish(msg_point)

        if not detected:
            miss_count += 1
            if miss_count > MISS_RESET_FRAMES:
                last_body_x = None
                last_body_y = None

        msg_detected = Bool()
        msg_detected.data = detected
        pub_detected.publish(msg_detected)

        rate.sleep()

    cap.release()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
