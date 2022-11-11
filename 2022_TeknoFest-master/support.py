import cv2
import control
import time
import motor_function
import matplotlib.pyplot as plt
# Mission_1 Square Detiction, Mission_2 elipse Detection, Mission_3 direk Detection
import Mission_3 as Mission
kamera_position = 300

'''
Bu modül içeriğinde:

sleep_function(sec_ang,choose,<cap_s>,<out_s>,<pwm_s>,check=1)
video_record(<cap>,<out>)

fonksiyonlarını bulundurur.

###########################################<3############################################################################################
#                                                                                                                                       #
#def video_record(<cap>,<out>):                                                                                                         #
#        Input paramater:2 object.<cap> kamera objesi ve <out> vide kayıt adresi objesini alır.                                         #
#                                                                                                                                       #
#def sleep_function(sec_ang,choose,<cap_s>,<out_s>,<pwm_s>,check=1):                                                                    #
#        input paramater:2 variable 3 object.                                                                                           #
#                        sec_angle:İstenilen açı veya istenile süre                                                                     #  
#                        choose:Fonksiyonun hangi parametreyi kullanıcağını belirler.Süre değişimi için 't',açı değilimiş için 'a'      #
#                        <cap> kamera objesi ve <out> vide kayıt adresi objesini alır.                                                  #
#                        <pwm> Motor komutlarına müdahale etmek amacıyla PCA9695'in adresini alır                                       #
#                                                                                                                                       #
###########################################<3############################################################################################
'''


class graph():
    def __init__(self,title="Deneme"):
        self.x = []
        self.y = []
        self.title=title

    def pull_data(self, error, time):
        self.x.append(time)
        self.y.append(error)

    def plot_graph(self):
        '''if self.title=='Leftmotor' or self.title=='Righmotor':
            plt.yticks(range(285,350,10))
        '''
        plt.xlabel('Time')
        plt.ylabel('Error')
        plt.title(self.title)
        plt.plot(self.x,self.y)
        plt.legend(["rightmotor", "error", "leftmotor", "gyro"], loc ="lower right")
        plt.savefig('Error_Graph_'+self.title+'.png')


def video_record(frame, out, fps="0"):
    line = control.gyro()  # Gyro değeri alınır değer olarak 4 elemanlı bir string array döner
    cv2.putText(frame, "V:" + line[0] + " Pressure" + line[1] + " Pitch:" + line[2] + " Yaw:" + line[3] + " FPS:" + fps,
                (0, 25), 1, 1, (255, 255, 255), 1)
    # oluşturulan son değer kayda basılır
    out.write(frame)



def fps(pre):
    # font which we will be using to display FPS 
    font = cv2.FONT_HERSHEY_SIMPLEX
    # time when we finish processing for this frame 
    new_frame_time = time.time()
    # Calculating the fps 
    # fps will be number of frame processed in given time frame 
    # since their will be most of time error of 0.001 second 
    # we will be subtracting it to get more accurate result 
    fps = 1 / (new_frame_time - pre)
    pre = new_frame_time
    # converting the fps into integer 
    fps = int(fps)
    # converting the fps to string so that we can display it on frame 
    # by using putText function 
    fps = str(fps)
    return pre, fps


def sleep_function(sec_ang, choose, cap_s, out_s, pwm_s, check=1, speed=340):#, cap_salt, check=1, speed=350):
    # Zamana bağlı tercih edilirse bu şartlı ifade geçerli sayılır
    global kamera_position
    if choose == 't':
        # Anlık zaman, başlangıç zamanı bilinmesi için dokunur
        start = time.time()
        # Anlık Zaman okunur
        stop = time.time()
        # Second_x Başlangıç zamanının anlık zamana göre farkının atanması için oluşturulur
        # Başlangıç olarak 0 atanır.
        Second_x = 0
        # While döngüsü fonksiyonun giriş değeri olan sec_ang ile geçen süre Second_x'i değerlendirir
        # Geçen süre, istenilen süreyi geçince fonksiyon durdurulur
        # used to record the time when we processed last frame 
        move = 1
        prev_time = 0
        fps_c = 0
        check_forward = 1
        angle = control.gyro()[3]
        angle_güdümlenme = angle
        last = 0.0
        pid_i = 0.0
        
        while Second_x <= float(sec_ang):
            # Buraya PID gelecek######
            # Görüntü işleme yapılıyor Geri döngü olarak hedefin merkezi alınıyor.
            # Geri döngü olarak işlenmiş görüntü ve hedef merkezi alınyor
            # frame_g=image , center_g[x,y]
            prev_time, fps_c = fps(prev_time)
            frame_center, center_g, pressure, Area_s = Mission.image_process(cap_s, out_s, fps_c)
            control.Pressure_Control(pressure, pwm_s)
            # Görüntü işleme eğer tespit yapamazsa None döndürür.
            # Eğer tespit yapılırsa güdümlenme döngüsüne girer
            last, pid_i = motor_function.motor_fri(pwm_s, speed, last, pid_i, angle)
            
            while center_g[0] != None:
                # merkez konumu değerlendirme amacıyla güdümlenme fonksiyonuna gönderilir
                move, pressure, last, pid_i, angle_güdümlenme,kamera_position = Mission.gudumlenme(center_g, pwm_s, frame_center,
                                                                        out_s, cap_s, Area_s,
                                                                        angle_güdümlenme, move, last, pid_i,kamera_position=kamera_position)
                # Yeni merkez noktasını bulmak amacıyla görüntü işlenir.
                control.Pressure_Control(pressure, pwm_s)
                prev_time, fps_c = fps(prev_time)
                frame_center, center_g, pressure, Area_s = Mission.image_process(cap_s, out_s, fps_c)
                # Eğer görüntü işleme framelerde görüntü bulamaz güdümlenmede ki en son komut
                # takılı kalmaması amacıyla motorlar dondurulur

                if center_g[0] == None:
                    motor_function.stop(pwm_s)
                    check_forward = 0
                    return check_forward

            # Anlık zaman değeri alınır
            stop = time.time()
            # Başlangıç zamanının anlık zamana göre farkı alınarak Second_x değişkenine atanır
            Second_x = stop - start
        # Süre dolduktan sonra motorlar durdurulur
        motor_function.stop(pwm_s)
        return check_forward

    # Açıya bağlı tercih edilirse bu şartlı ifade geçerli sayılır
    # Burada bug ver eğer görüntü bulunursa güdümlenme döngüsüne girer.Daha sonra görüntü
    # kaybedilirse motorlar dondurulur fakat motorlar çalışmadığından açı değişmez ve bu döngüde takılı kalır.
    elif choose == 'a':
        # Öncelikle gyronun ilk değeri alınır
        first_value = control.gyro()
        # Gyronun ilk yaw değerine dönülmesi istenilen açı değeri eklenir yada çıkarılır
        # Böylece ulaşılmak istenile açı değeri angle değişkenine atanır
        angle = (float(first_value[3])+sec_ang)%360  # Dönmesi istenilen açı değeri
        # İstenilen açı değerine ulaşılana kadar döngü çalışır
        # used to record the time when we processed last frame 
        move = 1
        prev_time = 0
        fps_c = 0
        check_rotation = 1
        last_e = 0.0
        pid_i_e = 0.0
        angle_inst = control.gyro()[3]
        while check == 1:
            # Görüntü işleme yapılıyor Geri döngü olarak hedefin merkezi alınıyor.
            # Geri döngü olarak işlenmiş görüntü ve hedef merkezi alınyor
            # frame_g=image , center_g[x,y]
            Second_X = 0
            prev_time, fps_c = fps(prev_time)
            frame_center, center_g, pressure, Area_s = Mission.image_process(cap_s, out_s, fps_c)
            control.Pressure_Control(pressure, pwm_s)
            # Görüntü işleme eğer tespit yapamazsa None döndürür.
            # Eğer tespit yapılırsa güdümlenme döngüsüne girer
            
            if center_g[0] != None:
                angle_inst = control.gyro()[3]
                while Second_X < float(0.5):
                    # Eğer görüntü işleme framelerde görüntü bulamaz güdümlenmede ki en son komut
                    # takılı kalmaması amacıyla motorlar dondurulur
                    if center_g[0] != None:
                        start = time.time()
                        stop = time.time()
                        Second_X = 0
                        # merkez konumu değerlendirme amacıyla güdümlenme fonksiyonuna gönderilir
                        move, pressure, last_e, pid_i_e, angle_inst,kamera_position = Mission.gudumlenme(center_g, pwm_s, frame_center,
                                                                                         out_s, cap_s, Area_s,
                                                                                         angle_inst, move, last_e,
                                                                                         pid_i_e,kamera_position=kamera_position)
                    else:
                        motor_function.stop(pwm_s)
                        # Arama Algortmasına geri dönüş için tasarlandı.
                        # Ancak ve ancak görüntü bulduktan ve görüntüyü kaybetikten sonra bu komuta girer
                        # Belirli bir süreden sonra arama algoritmasına geri dönülür.
                        stop = time.time()
                        last_e = 0.0
                        pid_i_e = 0.0
                        angle_inst = control.gyro()[3]
                        Second_X = stop - start
#                         Second_X = 3
                    # Yeni merkez noktasını bulmak amacıyla görüntü işlenir.
                    control.Pressure_Control(pressure, pwm_s)
                    prev_time, fps_c = fps(prev_time)
                    frame_center, center_g, pressure, Area_s = Mission.image_process(cap_s, out_s, fps_c)
                
                motor_function.stop(pwm_s)
                check_rotation = 0
                return check_rotation
            
            # İstenilen açı değerine ulaşıp ulaşmadığı değerlendirilmesi amacıla turn(x,<pwm>)
            # fonksiyonu çağırılır fonksiyon istenilen açı değerine ulaşıldığında motorları durdurur
            # ve geri döngğ olarak 0 derğeri döndürür
            check = control.turn(angle, pwm_s)
        return check_rotation
