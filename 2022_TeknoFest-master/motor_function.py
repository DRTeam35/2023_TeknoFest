'''
Bu kod motor fonksiyoları için oluşturulmuştur.
Tarih: 29.07.2021
Writer: Mustafa Akyüz,Yavuz Balı,Onurcan Köken
'''

import Adafruit_PCA9685
import time
import RPi.GPIO as GPIO
from PID import PID_class
from control import gyro
'''
Bu modüle içeriğinde:
    def start(power):
    def motor_fri(<pwm>,p):
    def motor_frg(<pwm>,p):
    def motor_u(<pwm>,p):
    def motor_d(<pwm>,p):
    def motor_r_yengec(<pwm>):
    def motor_l_yengec(<pwm>):
    def r_donus(<pwm>):
    def stop(<pwm>):
fonksiyonlarını içerir.
###########################################<3#####################################################
#                                                                                                #
#def start(power):                                                                               #
#        Input paramater: Power option.1 value open power.2 value  off power                     #                                           
#                                                                                                #
#def motor_fri(<pwm>,p):                                                                         #
#        Motor ileri fonksiyonu                                                                  #                                   
#        Input paramater:<pwm> :PCA9685 objesi                                                   #
#                        p:İstenilen hız değeri ((318,400) arası ileri (318,200) arası geri)     #
#def motor_frg(<pwm>,p):                                                                         #
#        Motor geri fonksiyonu                                                                   #                                   
#        Input paramater:<pwm> :PCA9685 objesi                                                   #
#                        p:İstenilen hız değeri ((318,400) arası ileri (318,200) arası geri)     #
#                                                                                                #
#def motor_u(<pwm>,p):                                                                           #
#        Motor yukarı fonksiyonu                                                                 #                                   
#        Input paramater:<pwm> :PCA9685 objesi                                                   #
#                        p:İstenilen hız değeri ((318,400) arası aşağı (318,200) arası yukarı)   #
#def motor_d(<pwm>,p):                                                                           #
#        Motor aşağı fonksiyonu                                                                  #                                   
#        Input paramater:<pwm> :PCA9685 objesi                                                   #
#                        p:İstenilen hız değeri ((318,400) arası aşağı (318,200) arası yukarı)   #
#                                                                                                #
#def motor_r_yengec(<pwm>):                                                                      #
#        Motor sağ yengeç hareketi.                                                              #
#        Input Paramater: <pwm> :PCA9685 objesi                                                  #
#                                                                                                #
#def motor_l_yengec(<pwm>):                                                                      #
#        Motor sol yengeç hareketi.                                                              #
#        Input Paramater: <pwm> :PCA9685 objesi                                                  #
#                                                                                                #
#def r_donus(<pwm>):                                                                             #
#        Motor sağ dönüş hareketi.                                                               #
#        Input Paramater: <pwm> :PCA9685 objesi                                                  #
#                                                                                                #
#def l_donus(<pwm>):                                                                             #
#        Motor sol dönüş hareketi.                                                               #
#        Input Paramater: <pwm> :PCA9685 objesi                                                  #
#                                                                                                #
#def stop(<pwm>):                                                                                #
#        Motorları durdurma komutu.                                                              #
#        Input Paramater: <pwm> :PCA9685 objesi                                                  #
#                                                                                                #
###########################################<3####################################################
'''



def start(power):
    #Röle kontrol pini servoPIM diye tanımlanıyor (neden servo yazdığını sorma deüiştirmeye üşendim)
    servoPIN = 12
    #GPIO'lar GPIO.BCM formatına set ediliyor
    GPIO.setmode(GPIO.BCM)
    #servoPIN'den çıkış alınacağı tanımlanıyor
    GPIO.setup(servoPIN, GPIO.OUT)

    if power==1:
        #servoPIN'den röleye sinyal gönderiliyor. Böylece motorlara ve sisteme güç gidiyor
        GPIO.output(servoPIN, GPIO.HIGH)
    elif power==0:
        # servoPIN'den röleye giden sinyal kesiliyor. Böylece motorlara ve sisteme giden güç kesiliyor
        GPIO.output(servoPIN, GPIO.LOW)
    
def calibration(pwm):
    #Kalibrasyon için öncelikle maximum PWM veriliyor
    pwm.set_pwm(13, 0, 500)  # sağ arka dikey
    pwm.set_pwm(12, 0, 500)  # sol arka dikey
    pwm.set_pwm(11, 0, 500)  # sağ ön itici
    pwm.set_pwm(10, 0, 500)  # sol ön itici
    pwm.set_pwm(9, 0, 500)  # sağ arka itici
    pwm.set_pwm(8, 0, 500)  # sol arka itici
    #Motorların sesinin bitmesine kadar geçen süre
    time.sleep(1)
    #Daha sonra Minimum PWM değeri giriliyor
    pwm.set_pwm(13, 0, 0)  # sağ arka dikey
    pwm.set_pwm(12, 0, 0)  # sol arka dikey
    pwm.set_pwm(11, 0, 0)  # sağ ön itici
    pwm.set_pwm(10, 0, 0)  # sol ön itici
    pwm.set_pwm(9, 0, 0)  # sağ arka itici
    pwm.set_pwm(8, 0, 0)  # sol arka itici
    # Motorların sesinin bitmesine kadar geçen süre
    time.sleep(1)
    
    # pwm.set_pwm(channel_number, pwm_high_start, pwm_high_stop)
    #En son olarak motorların durgun olduğu orta nokta PWM'i veriliyor
    pwm.set_pwm(13, 0, 318)  # sağ arka dikey
    pwm.set_pwm(12, 0, 318)  # sol arka dikey
    pwm.set_pwm(11, 0, 318)  # sağ ön itici
    pwm.set_pwm(10, 0, 318)  # sol ön itici
    pwm.set_pwm(9, 0, 318)  # sağ arka itici
    pwm.set_pwm(8, 0, 318)  # sol arka itici
    # Motorların sesinin bitmesine kadar geçen süre
    time.sleep(1)

def motor_fri(pwm,p,last_error,pid_i,desired_angle):  # ileri motorlar
    #İleri gitme esnasında en düz p=285 değerinde gidiyor
    delta_time,current_yaw = gyro()[2:4]
    pid_object = PID_class(last_error,pid_i)
    left_motor,right_motor,last_error,pid_i = pid_object.calculateMotorPowers(.218,0,10.12,p,desired_angle
                                                                              ,current_yaw,delta_time)
    

    pwm.set_pwm(8, 0, int(left_motor)) # sol mu sağ mı
    pwm.set_pwm(9, 0, int(right_motor))
    pwm.set_pwm(10, 0, int(left_motor))
    pwm.set_pwm(11, 0, int(right_motor))

    return last_error,pid_i



def motor_frg(pwm,p):  #geri motorlar
    #İleri gitme esnasında en düz p=285 değerinde gidiyor
    pwm.set_pwm(8, 0, p)
    pwm.set_pwm(9, 0, p-2)
    pwm.set_pwm(10, 0, p)
    pwm.set_pwm(11, 0, p-2)

def motor_u(pwm,p):  # yukarı motorlar
    pwm.set_pwm(13, 0, p)
    pwm.set_pwm(12, 0, p)
    
def motor_d(pwm,p):  # aşağı motorlar
    pwm.set_pwm(13, 0, p)
    pwm.set_pwm(12, 0, p)

def motor_r_yengec(pwm):  # sağ/sol motorlar
    #Burada hız değeri testlere göre belirlenmekte
    pwm.set_pwm(11, 0, 350)  # gerekirse 0'ı 5 yap
    pwm.set_pwm(10, 0, 318)  # gerekirse 0'ı 5 yap
    pwm.set_pwm(9, 0, 283)
    pwm.set_pwm(8, 0, 318)

def motor_l_yengec(pwm):  # sağ/sol motorlar
    #Burada hız değeri testlere göre belirlenmekte
    pwm.set_pwm(11, 0, 318)  # gerekirse 0'ı 5 yap
    pwm.set_pwm(10, 0, 350)  # gerekirse 0'ı 5 yap
    pwm.set_pwm(9, 0, 318)
    pwm.set_pwm(8, 0, 285)

def r_donus(pwm,p=330):
    fark = p - 318
    pwm2 = 318 - fark
    pwm.set_pwm(11, 0, 318)
    pwm.set_pwm(10, 0, p)
    pwm.set_pwm(8, 0, 318)
    if p == 330:
        pwm.set_pwm(9, 0, pwm2)
    else:
        pwm.set_pwm(9, 0, 318)
    
def l_donus(pwm,p=330):
    fark = p - 318
    pwm2 = 318 - fark
    pwm.set_pwm(11, 0, p)
    pwm.set_pwm(10, 0, 318)
    pwm.set_pwm(9, 0, 318)
    if p==330:
        pwm.set_pwm(8, 0, pwm2)
    else:
        pwm.set_pwm(8, 0, 318)

def stop(pwm):  # motorları durdur
    pwm.set_pwm(13, 0, 318)  # sağ arka dikey
    pwm.set_pwm(12, 0, 318)  # sol arka dikey
    pwm.set_pwm(11, 0, 318)  # sağ ön itici
    pwm.set_pwm(10, 0, 318)  # sol ön itici
    pwm.set_pwm(9, 0, 318)  # sağ arka itici
    pwm.set_pwm(8, 0, 318)  # sol arka itici
