# Direk görevi
# Direk görevi
import cv2
import numpy as np
import math
import motor_function
import control
import time
import support


# adina feriha koydum dan devam ediyorum
# Koeken
# 10.09.2021

# this function returns frame, center
def image_process(cap, out_i, fps_i="0"):
    ret, frame = cap.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    shape = frame.shape  # shape[0] = y coord , shape[1] = x coord
    # Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    range_center = shape[1] / 2, shape[0] / 2

    # copy the original frame
    original_one = frame.copy()

    # convert to hsv for masking
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # mask limits
    yellow_high = np.array([95, 255, 210])
    yellow_low = np.array([75, 190, 0])
    mask = cv2.inRange(hsv, yellow_low, yellow_high)
    # cv2.imshow("mask", mask)

    # to display the masked result
    # result = cv2.bitwise_and(frame, frame, mask=mask)
    # cv2.imshow("result", result)

    # find contours in the mask and initialize the current
    # (x, y) center
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = [None, None]

    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        str_radius = "radius: " + str(int(radius))
        cv2.putText(frame, str_radius, (0, 120), 1, 1, (0, 255, 0), 1)
        M = cv2.moments(c)  # used to calculate the center

        # area of the kuka contour
        area = "area: " + str(cv2.contourArea(c))
        cv2.putText(frame, area, (0, 135), 1, 1, (0, 255, 0), 1)

        # bounding rectangle around the contour
        box_x, box_y, box_w, box_h = cv2.boundingRect(c)
        bounding_box = "box width: " + str(int(box_w)) + " box height: " + str(int(box_h))
        cv2.rectangle(frame, (box_x, box_y), (box_x + box_w, box_y + box_h), (0, 255, 0), 2)
        cv2.putText(frame, bounding_box, (0, 150), 1, 1, (0, 255, 0), 1)

        # line filter, if width > height
        # this is a funking line
        if box_w > box_h:
            center = [None, None]
            support.video_record(original_one, out_i, fps_i)
            return range_center, center, 20, 0

        # to avoid (float/zero) condition
        if (M["m00"] == 0):
            center = [None, None]
            support.video_record(original_one, out_i, fps_i)
            return range_center, center, 20, 0
        else:
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print("center: ", center)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 3, (0, 0, 255), -1)
            # DETECTED!
            support.video_record(frame, out_i, fps_i)
            return range_center, center, 20, 0
        else:
            # radius is too small
            center = [None, None]
            support.video_record(original_one, out_i, fps_i)
            return range_center, center, 20, 0

    # there is no contour
    else:
        center = [None, None]
        support.video_record(original_one, out_i, fps_i)
        return range_center, center, 27, 0


def gudumlenme(center_2, pwm_g, frame_center, out_g, cap_gon, Area_g,angle, move=1, last=0, pid_i=0,kamera_position=292):
    # Merkezi Tolerans
    center_tol = 100
    pressure = 26
    # Kamera merkezinde y-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    # merkezini bu kümeye sokmaya çalışır.
    range2 = frame_center[1] - center_tol, frame_center[1] + center_tol
    # Kamera merkezinde x-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    # merkezini bu kümeye sokmaya çalışır.
    range3 = frame_center[0] - center_tol, frame_center[0] + center_tol

    #############################################--X_ekseni merkezleme---###########################################
    # Eğer nesneni merkezi kümenin solundaysa sol yengeç hareketi gerçekleştirilir
    if center_2[0] > range3[1]:
        # sola git
        motor_function.r_donus(pwm_g, 330)
        move = 1
        last = 0.0#error
        pid_i = 0.0
        angle = control.gyro()[3]
    # Eğer nesneni merkezi kümenin sağındaysa sağ yengeç hareketi gerçekleştirilir
    elif center_2[0] < range3[0]:
        # sağa git
        motor_function.l_donus(pwm_g, 330)
        move = 1
        last = 0.0#error
        pid_i = 0.0
        angle = control.gyro()[3]
    # Eğer nesnenin merkezi ,x-ekseni için belirlenen tolerans kümesi içindeyse araç ileri doğru gider.
    # -----------------------------------------------------------------------------------------------------------------#
    elif center_2[0] > range3[0] and center_2[0] < range3[1]:
        last, pid_i = motor_function.motor_fri(pwm_g, 340, last, pid_i, angle)
        move = 0
    return move, pressure, last, pid_i,angle,kamera_position
