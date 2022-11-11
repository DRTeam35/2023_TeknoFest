# Yere konma Görevi
# Mission_2 icin kamera acısı 292
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
    # İlk kamera işlemleri.
    area_i = 0
    ret, frame_ = cap.read()
    frame_ = cv2.flip(frame_, 0)
    frame_ = cv2.flip(frame_, 1)
    frame = cv2.cvtColor(frame_, cv2.COLOR_BGR2GRAY)
    
    shape = frame.shape  # shape[0] = y coord , shape[1] = x coord
    # Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    range_center = shape[1] / 2, shape[0] / 2

    # Set our filtering parameters
    # Initialize parameter setting using cv2.SimpleBlobDetector
    params = cv2.SimpleBlobDetector_Params()

    # Set Color filtering parameters
    # Use blobColor = 0 to extract dark blobs and blobColor = 255 to extract light blobs
    params.filterByColor = False
    params.blobColor = 0

    # Set Area filtering parameters
    params.filterByArea = True
    params.minArea = 3000
    params.maxArea = (frame.shape[0] * frame.shape[1]) / 2

    # Set threshold filtering parameters
    # params.minThreshold = 0
    # params.maxThreshold = 20000000

    # Set Circularity filtering parameters min 0, max 1
    params.filterByCircularity = True
    params.minCircularity = 0.55
    params.maxCircularity = 1

    # Set Convexity filtering parameters min 0, max 1
    params.filterByConvexity = True
    params.minConvexity = 0.3
    params.maxConvexity = 1

    # Set inertia filtering parameters min 0, max 1
    params.filterByInertia = True
    params.minInertiaRatio = 0.2
    params.maxInertiaRatio = 1

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Detect blobs
    keypoints = detector.detect(frame)

    # get coordinates and sizes of detected objects
    coordinates = [key_point.pt for key_point in keypoints]
    sizes = [key_point.size for key_point in keypoints]

    # check colors, eliminate blue ones
    eliminate_idx = []
    obj_color = 0
    green_color = 0
    for j in range(0, len(keypoints)):
        obj_color, green_color, a_ = frame_[int(coordinates[j][1]), int(coordinates[j][0])]  # image color code is BGR in my case, change channel if it is necessary
        print("B:"+str(obj_color)+"G:"+str(green_color)+"R:"+str(a_))
        if obj_color > 100 or green_color > 50:
            eliminate_idx.append(j)

    # remove some elements of a list
    keypoints = [i for j, i in enumerate(keypoints) if j not in eliminate_idx]
    coordinates = [i for j, i in enumerate(coordinates) if j not in eliminate_idx]
    sizes = [i for j, i in enumerate(sizes) if j not in eliminate_idx]
    if coordinates:
        center2 = [int(coordinates[0][0]), int(coordinates[0][1])]
        text = "X :" + str(int(coordinates[0][0])) +" "+ "Y :" + str(int(coordinates[0][1]))+" " + "R :" + str(sizes[0])
        cv2.putText(frame_, text, (0, 150), 1, 1, (0, 255, 0), 1)
        cv2.circle(frame_, (int(coordinates[0][0]), int(coordinates[0][1])), int(sizes[0]/2), (255, 255, 0), 3)
        area_i = int(sizes[0])
    else:
        area_i=0
        center2 = [None, None]
    # Draw blobs on our image as red circles
    #cv2.rectangle(frame_, (int((shape[1]/2)-80), int((shape[0]/2)-80)), (int((shape[1]/2) +80),int((shape[0]/2) + 20)), (0, 255, 0), 2)
    support.video_record(frame_, out_i, fps_i)
    return range_center, center2, 2, area_i  # son değerler sırayla sabitlenmek istenen basınç değeri,Area


# Bu kod RAL 3026 kırmızı rengi için filtreleme uygular.
# Eğer resimde kırmızı varsa bu kırmızı renginin orta merkezini bulur.
# geri döngü olarak frame ve renk yoğunluğunun merkezini döndürür.
def afk(cap_a, out_a, access=1):
    # Alt kameradan image alınır ve frame değişkenine atanır.
    # Burada testlerin durumuna göre kamera döndürülmesi veya ayna görüntüsünün alınması gerekebilir
    ret, frame = cap_a.read()
    frame = cv2.flip(frame, 0)
    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    shape = frame.shape  # shape[0] = y coord , shape[1] = x coord
    # Araç kamerası için Merkez noktası belirlenir range1=[x_center,y_center]
    frame_center = shape[1] / 2, shape[0] / 2
    # center2=[None,None] atanır.Bulunamazsa eğer geri bu değer döndürülür
    radius=0
    center2 = [None, None]  # this is for non-detected case, default
    # RAL3026 için üst treshold seviyesi
    red_high = np.array([156, 255, 69])
    # RAL3026 için alt treshold seviyesi
    red_low = np.array([104, 0, 0])
    # Kırmızı rengi için maskeleme yapılır.
    mask = cv2.inRange(hsv, red_low, red_high)
    result = cv2.bitwise_and(frame, frame, mask=mask)
    # maskelenen görüntüde contour çıkarılır.
    # find contours in the mask and initialize the current
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # (x, y) center
    center2_2 = None  # this is for detected case
    # Bu döngüde bulunan contourların alanları ölçülür. Alanı belli sviyenin üzerindeki contourların merkezi bulunur daha sonra
    # bu merkez center2 değişkenine atanır. En so olarak fonksiyon geriye işlenmiş görüntüyü ve kırmızı renginin merkezini döndürür
    # only proceed if at least one contour was found
    if len(cnts) > 0:
        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        # to avoid float / zero condition
        if (M["m00"] == 0):
            center2_2 = center2
        else:
            center2_2 = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print("center: ", center2_2)

        # only proceed if the radius meets a minimum size
        if radius > 10:
            # print("radius: ", radius)
            # draw the circle and centroid on the frame,
            # then update the list of tracked points
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, center2_2, 3, (0, 0, 255), -1)
            text="Radius:"+str(radius)
            cv2.putText(frame, text, (0, 150), 1, 1, (0, 0, 0), 1)

            if access == 1:
                support.video_record(frame, out_a)
            return frame_center, center2_2 ,radius
    if access == 1:
        support.video_record(frame, out_a)
    return frame_center, center2, radius


#############################################------------------------------------Gudumlenme--------------------------------------###################################################
'''
Öyle İşte
'''



def fatality(pwm_f, cap_on, out_f):  # ön kamera objesi lazım,
    # ileri gideceği süre
    
    sec_ang = 10
    start = time.time()
    # Anlık Zaman okunur
    stop = time.time()
    # Second_x Başlangıç zamanının anlık zamana göre farkının atanması için oluşturulur
    # Başlangıç olarak 0 atanır.
    Second_x = 0
    last=0.00
    pid_i=0.00
    angle_fatality=control.gyro()[3]
    # While döngüsü fonksiyonun giriş değeri olan sec_ang ile geçen süre Second_x'i değerlendirir
    # Geçen süre, istenilen süreyi geçince fonksiyon durdurulur
    #last,pid_i=motor_function.motor_fri(pwm_f, 330,last,pid_i,angle_fatality)
    center_g = [None, None]
    center_f = [None, None]
    move = 0
    pwm_f.set_pwm(3, 0, 225)
    check_afk = 0
    check_image = 0
    while Second_x <= sec_ang:
        # Buraya PID gelecek######
        # Görüntü işleme yapılıyor Geri döngü olarak hedefin merkezi alınıyor.
        # Geri döngü olarak işlenmiş görüntü ve hedef merkezi alınyor
        # frame_g=image , center_g[x,y]
        
        frame_center, center_f,radius = afk(cap_on, out_f, 0)
        control.Pressure_Control(2,pwm_f)
        # Görüntü işleme eğer tespit yapamazsa None döndürür.
        # Eğer tespit yapılırsa güdümlenme döngüsüne girer
        if center_f[1] != None:
            motor_function.stop(pwm_f)
            check_afk = 1
           

            while center_f[1] != None:
                

                # merkez konumu değerlendirme amacıyla güdümlenme fonksiyonuna gönderilir
                move, pres_f,last,pid_i,angle_fatality,kamera_position = gudumlenme(center_f, pwm_f, frame_center, out_f, cap_on, 0,
                                                                                  angle_fatality, move,last,pid_i,activity=0,kamera_position=225)
                control.Pressure_Control(2,pwm_f)
                # kayıt alınır
                # Yeni merkez noktasını bulmak amacıyla görüntü işlenir.
                frame_center, center_f,radius = afk(cap_on, out_f, 1)
                if radius<73:
                    check_afk=1
                    check_image=1
                    break
            motor_function.stop(pwm_f)
        elif center_f[1] == None:
            frame_center_circle, center_g, pressure, Area_on = image_process(cap_on, out_f)
            if center_g[1] != None:
                move, pres_f,last,pid_i,angle_fatality,kamera_position = gudumlenme(center_g, pwm_f, frame_center_circle, out_f, cap_on, 0,
                                                                                    angle_fatality, move,last,pid_i,activity=0,kamera_position=225)
                check_afk = 0
                check_image = 0
            else:
                last,pid_i=motor_function.motor_fri(pwm_f, 338,last,pid_i,angle_fatality)
                check_image = 1
                # Anlık zaman değeri alınır
        if check_afk == 1 and check_image == 1:
            motor_function.stop(pwm_f)
            break
        stop = time.time()
        # Başlangıç zamanının anlık zamana göre farkı alınarak Second_x değişkenine atanır
        Second_x = stop - start
    # Burada kırmızı rengi x koordinatında merkezlenebildiyse artık aracı dibe batırır ve motorları kapatır veeeeeee koduda kapatır.
    motor_function.stop(pwm_f)
    time.sleep(1)
    motor_function.motor_d(pwm_f, 353)
    while True:
        ret, frame_finish = cap_on.read()
        frame_finish = cv2.flip(frame_finish, 0)
        frame_finish = cv2.flip(frame_finish, 1)
        support.video_record(frame_finish, out_f)
        sensor = control.gyro()
        preassure = float(sensor[1])
        # 11 kPascal altıysa baatmıştıt artık be
        if preassure > float(40):
            motor_function.stop(pwm_f)
            time.sleep(1)
            # Motor Gücü kes
            motor_function.start(0)
            # programı kapat
            exit()


def gudumlenme(center_2, pwm_g, range_center_g, out_g, cap_gon, Area_g, angle, move=1, last=0.00, pid_i=0.00,
               activity=1,kamera_position = 290):
    # Merkezi Tolerans
    center_tol = 70
    center_tol_y = 20
    pressure = 1
    # Kamera merkezinde y-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    # merkezini bu kümeye sokmaya çalışır.
    range2 = range_center_g[1] - center_tol_y-60, range_center_g[1] + center_tol_y
    # Kamera merkezinde x-ekseni için 50 piksel toleranslık alan belirlenir. Araç tespit ettiği nesnenin
    # merkezini bu kümeye sokmaya çalışır.
    range3 = range_center_g[0] - center_tol, range_center_g[0] + center_tol
    if (center_2[1] < range2[0] and kamera_position < 395) and activity == 1 :
        kamera_position = kamera_position+1
        pwm_g.set_pwm(3, 0, kamera_position)
        # kamera yukarı
    elif (center_2[1] > range2[1] and kamera_position > 220) and activity == 1:
        kamera_position = kamera_position-2
        pwm_g.set_pwm(3, 0, kamera_position)
        #kamera aşağı 
    #############################################--X_ekseni merkezleme---###########################################
    # Eğer nesneni merkezi kümenin solundaysa sol yengeç hareketi gerçekleştirilir
    if center_2[0] > range3[1]:
        # sola git
        motor_function.r_donus(pwm_g, 327)
        move = 1
        #angle = control.gyro()[3]
    # Eğer nesneni merkezi kümenin sağındaysa sağ yengeç hareketi gerçekleştirilir
    elif center_2[0] < range3[0]:
        # sağa git
        motor_function.l_donus(pwm_g, 327)
        move = 1
        #angle = control.gyro()[3]
    # Eğer nesnenin merkezi ,x-ekseni için belirlenen tolerans kümesi içindeyse araç ileri doğru gider.
    # -----------------------------------------------------------------------------------------------------------------#
    elif center_2[0] > range3[0] and center_2[0] < range3[1]:
        angle=control.gyro()[3]
        if move == 1:
            motor_function.stop(pwm_g)
        else:
            last, pid_i= motor_function.motor_fri(pwm_g, 335, last, pid_i, angle)
        move = 0
        if Area_g > 210 and activity == 1:
            pwm_g.set_pwm(3, 0, 225)
            fatality(pwm_g,cap_gon,out_g)

    return move, pressure,last, pid_i,angle,kamera_position
