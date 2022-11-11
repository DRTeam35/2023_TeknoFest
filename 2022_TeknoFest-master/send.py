import cv2.cv2
from cv2 import cv2 as cv
import numpy as np
import math
prevcircle=None

cap = cv.VideoCapture(0)

while True:

    ret, frame = cap.read()
    cv.waitKey(1)
    hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
    a_total=0
    b_total=0
    r_total=0
    yellow_high = np.array([150, 255, 255])
    yellow_low = np.array([75, 190, 0])
    mask = cv.inRange(hsv, yellow_low, yellow_high)
    res = cv.bitwise_and(frame, frame, mask=mask)
    res_blurred=cv.medianBlur(res,3)
    gray=cv.cvtColor(res_blurred,cv.COLOR_BGR2GRAY)
    gray_canny=cv.Canny(gray,20,60)
    gray_canny_backup=gray_canny
    detected_circles = cv.HoughCircles(gray_canny,
                                        cv.HOUGH_GRADIENT, 1, 20, param1=50,
                                        param2=40, minRadius=50, maxRadius=250)

    if detected_circles is not None:
        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))
        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]
            a_total+=a
            b_total +=b
            r_total += r

        a_total = int(a_total / len(detected_circles[0, :]))
        b_total = int(b_total / len(detected_circles[0, :]))
        r_total = int(r_total / len(detected_circles[0, :]))
        if r_total >= 20:
            # Draw the circumference of the circle.
            cv.circle(frame, (a_total, b_total), r_total, (0, 255, 0), 2)

            a_old = a_total
            b_old = b_total
            r_old = r_total
            # Draw a small circle (of radius 1) to show the center.
            cv.circle(frame, (a_total, b_total), 1, (0, 0, 255), 3)
            cv.circle(gray_canny_backup, (a_total, b_total), r_total, (0, 255, 0), 2)
            # writer.write(gray_canny_backup)

            a_total = 0
            b_total = 0
            r_total = 0


writer.release()


