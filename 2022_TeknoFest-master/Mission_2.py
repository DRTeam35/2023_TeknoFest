#Daire Görevi

##### Mission_1 icin kamera açısı 315
import cv2
import numpy as np
import math
import motor_function
import control
import time
import support

# image_process() returns the last version of the frame
# red_filter() filters red color, returns frame
# line() # returns 0 - no line or 1 - line
# pre_ellipse() # pre-process of ellipse, returns ellipse_check()
# ellipse_check() # returns 0 - no ellipse or 1 - ellipse
# gudumlenme
#############################################------------------------------------Görüntü işleme--------------------------------------###################################################

def image_process(cap, out_i, fps_i="0"):  # bura Ece nin kod
    center = [None,None]
    ret, frame = cap.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    shape = frame.shape  # shape[0] = y coord , shape[1] = x coord
    # Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    range_center = shape[1] / 2, shape[0] / 2
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    a_total = 0
    b_total = 0
    r_total = 0
    res_blurred = cv2.GaussianBlur(hsv, (5, 5), 0)
    removed = remove_lines(res_blurred)
    if removed is not None:
        blank = np.zeros(frame.shape[:2], dtype='uint8')
        contours, hierarchy = cv2.findContours(removed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for i in range(len(contours)):
            if cv2.contourArea(contours[i]) > 10 and cv2.contourArea(contours[i]) < 1000:
                cp = contours[i]
                cv2.drawContours(blank, [cp], 0, (255, 255, 255), 3)
        detected_circles = cv2.HoughCircles(blank,
                                           cv2.HOUGH_GRADIENT, 1, 20, param1=50,
                                           param2=15, minRadius=100, maxRadius=300)

        if detected_circles is not None:
            # Convert the circle parameters a, b and r to integers.
            detected_circles = np.uint16(np.around(detected_circles))
            for pt in detected_circles[0, :]:
                a, b, r = pt[0], pt[1], pt[2]
                a_total += a
                b_total += b
                r_total += r

            a_total = int(a_total / len(detected_circles[0, :]))
            b_total = int(b_total / len(detected_circles[0, :]))
            r_total = int(r_total / len(detected_circles[0, :]))
            # Draw the circumference of the circle.
            cv2.circle(frame, (a_total, b_total), r_total, (0, 255, 0), 2)
            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(frame, (a_total, b_total), 1, (0, 0, 255), 3)
            text = "X :" + str(a_total) + " " + "Y :" + str(b_total) + " " + "R :" + str(r_total)
            cv2.putText(frame, text, (0, 150), 1, 1, (0, 255, 0), 1)
            center = [a_total, b_total]
    cv2.rectangle(frame, (int((shape[1]/2)-50), int((shape[0]/2)-50)), (int((shape[1]/2) +70),int((shape[0]/2) + 30)), (0, 255, 0), 2)
    support.video_record(frame, out_i, fps_i)
    return range_center, center, 21, r_total
def remove_lines(image_):
    image = image_.copy()
    kernel = np.ones((5, 5), np.uint8)
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Use canny edge detection
    edges = cv2.Canny(gray, 10, 25, apertureSize=3)

    # Apply HoughLinesP method to
    # to directly obtain line end points
    lines_list = []
    lines = cv2.HoughLinesP(
        edges,  # Input edge image
        1,  # Distance resolution in pixels
        np.pi / 180,  # Angle resolution in radians
        threshold=50,  # Min number of votes for valid line
        minLineLength=50,  # Min allowed length of line
        maxLineGap=10  # Max allowed gap between line for joining them
    )
    if lines is not None:
        for points in lines:
            # Extracted points nested in the list
            x1, y1, x2, y2 = points[0]
            # Draw the lines joing the points
            # On the original image
            cv2.line(edges, (x1, y1), (x2, y2), (0, 0, 0), 10)

            # Maintain a simples lookup list for points
            lines_list.append([(x1, y1), (x2, y2)])
        return edges
    # Iterate over points
    else:
        return None
#############################################------------------------------------Gudumlenme--------------------------------------###################################################

def fatality(pwm_f,cap_on,out_f,press):#ön kamera objesi lazım,
    #ileri gideceği süre
    sec_ang=20
    start = time.time()
    #Anlık Zaman okunur
    stop = time.time()
    #Second_x Başlangıç zamanının anlık zamana göre farkının atanması için oluşturulur
    #Başlangıç olarak 0 atanır.
    Second_x=0
    last = 0.0
    pid_i = 0.0
    angle = control.gyro()[3]
    #While döngüsü fonksiyonun giriş değeri olan sec_ang ile geçen süre Second_x'i değerlendirir
    #Geçen süre, istenilen süreyi geçince fonksiyon durdurulur
    last, pid_i = motor_function.motor_fri(pwm_f, 340, last, pid_i, angle)
    center_g=[None,None]
    center_f=[None,None]
    
    move=0
    while Second_x <= float(12):
        last, pid_i = motor_function.motor_fri(pwm_f, 340, last, pid_i, angle)
        control.Pressure_Control(press,pwm_f)
        stop =time.time()
        #Başlangıç zamanının anlık zamana göre farkı alınarak Second_x değişkenine atanır
        Second_x=stop-start
    '''
    while Second_x <= sec_ang:
        #Buraya PID gelecek######
        #Görüntü işleme yapılıyor Geri döngü olarak hedefin merkezi alınıyor.
        #Geri döngü olarak işlenmiş görüntü ve hedef merkezi alınyor
        #frame_g=image , center_g[x,y]
        
        
        frame_center_circle,center_g,pressure,Area_on=image_process(cap_on,out_f)
        #Görüntü işleme eğer tespit yapamazsa None döndürür.
        #Eğer tespit yapılırsa güdümlenme döngüsüne girer
        last, pid_i = motor_function.motor_fri(pwm_f, 340, last, pid_i, angle)

        if center_g[1]!=None:
            motor_function.stop(pwm_f)
            while center_g[1]!=None:
                #merkez konumu değerlendirme amacıyla güdümlenme fonksiyonuna gönderilir
                move,press ,last,pid_i,angle,kamera_position=gudumlenme(center_g,pwm_f,frame_center_circle,out_f,cap_on,Area_on,move,last, pid_i,0)
                control.Pressure_Control(press,pwm_f)
                #kayıt alınır
                #Yeni merkez noktasını bulmak amacıyla görüntü işlenir.
                frame_center_circle,center_g,pressure,Area_on=image_process(cap_on,out_f)
                angle = control.gyro()[3]
            last, pid_i = motor_function.motor_fri(pwm_f, 340, last, pid_i, angle)

        
        #Anlık zaman değeri alınır
        stop =time.time()
        #Başlangıç zamanının anlık zamana göre farkı alınarak Second_x değişkenine atanır
        Second_x=stop-start
    '''
            
def gudumlenme(center_2,pwm_g,range_center_g,out_g,cap_gon,Area_g,angle,move=1,last=0.00,pid_i=0.00,activity=1,kamera_position=330):
    #Merkezi Tolerans
    center_tol=70
    center_tol_y=60
    
    last = 0.0
    pid_i = 0.0
    angle = control.gyro()[3]
    f = open('pressure.txt', "r")
    # Son kaydedilen basınc değeri ölçülür
    pressure = f.read()
    f.close()
    pressure=float(pressure)
    #Kamera merkezinde y-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range2 = range_center_g[1] -center_tol_y+10, range_center_g[1] +center_tol_y-30
    #Kamera merkezinde x-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    #merkezini bu kümeye sokmaya çalışır.
    range3= range_center_g[0]-center_tol+20,range_center_g[0]+center_tol
        
    #############################################--X_ekseni merkezleme---###########################################
    #Eğer nesneni merkezi kümenin solundaysa sol yengeç hareketi gerçekleştirilir
    if center_2[1]!=None:
        if center_2[1] > 120 and center_2[1] < range2[0]:
            
            motor_function.motor_u(pwm_g,290)
            pressure=0
        elif center_2[1] > 120 and center_2[1] > range2[1]:
            
            motor_function.motor_d(pwm_g,350)
            pressure=0# Basıınç sabitlemeyi kapatır,
        else:
            # pressure.txt belgesi 'w' formatında açılır
            sensor=control.gyro()
            pressure=float(sensor[1])
            f = open('pressure.txt', "w")
            # Yeni alına basınc bilgisi pressure.txt'ye yazdırılır
            f.write(sensor[1])
            # dosya kağatılır
            f.close()

        if center_2[0] > range3[1]: #and center_2[1] > range2[0] and center_2[1] < range2[1]:
            #sağa git
            motor_function.r_donus(pwm_g,327)
            move=1
        #Eğer nesneni merkezi kümenin sağındaysa sağ yengeç hareketi gerçekleştirilir
        elif center_2[0] < range3[0]:# and center_2[1] > range2[0] and center_2[1] > range2[1]:
            #sola git
            motor_function.l_donus(pwm_g,327)
            move=1
        elif center_2[0] > range3[0] and center_2[0] < range3[1] and center_2[1] > range2[0] and center_2[1] < range2[1]:
            last, pid_i = motor_function.motor_fri(pwm_g, 337, last, pid_i, angle)
            move=0
            if Area_g>200 and activity==1:
                fatality(pwm_g,cap_gon,out_g,pressure)
        elif center_2[0] > range3[0] and center_2[0] < range3[1]:
            motor_function.l_donus(pwm_g,318)
    return move,pressure, last, pid_i,angle,kamera_position
