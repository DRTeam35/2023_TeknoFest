import cv2
import serial
import time
import motor_function
import control
import Adafruit_PCA9685
import support
from control import gyro
#Ardunio nano'ya ilk bağlanyı yapılır
if __name__ == '__main__':
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    #Ardunio Nano sıfırdan başlatma komutu. Bu komut kullanıldığında
    #gyro sıfırlanır.Aracın olan konumunda açılar 0 olarak ayarlanır
    ser.flush()
#Ön kameranın objesi cap değişkenine atanır
#Buraya dikkat etmek lazım kamera 0,1 veya 2 olabiliyor
cap = cv2.VideoCapture(0)
#alt kameranın objesi cap değişkenine atanır
#Buraya dikkat etmek lazım kamera 0,1 veya 2 olabiliyor
#cap_2=cv2.VideoCapture(0)

# codec tanımlama ve VideoWriter nesnesi oluşturma bilgi için bkz: https://www.fourcc.org/codecs.php
fourcc = cv2.VideoWriter_fourcc('D','I','V','X')
# Yukarıdaki işlemi aşağıdaki gibi de yapabiliriz.
#fourcc = cv.VideoWriter_fourcc('X','V','I','D')
# Kaydedilecek video dosyasının adı, uzantısı, konumu, saniyedeki çerçeve sayısı ve çözünürlüğü
out = cv2.VideoWriter('recording_video/deneme1.avi',fourcc, 8.0, (int(cap.get(3)),int(cap.get(4))))

#Lütfen buraya aramo algoritmanızı kısaca betimleyiniz.

#Burada arama olgoritması tasarlanır
def searching():
    while True:
        while True:
            
            motor_function.l_donus(pwmx)
            check=support.sleep_function(-30,'a',cap,out,pwmx)#,cap_2)
            if check==0:
                break

            motor_function.r_donus(pwmx)
            check=support.sleep_function(60,'a',cap,out,pwmx)#,cap_2)
            if check==0:
                break
        
            motor_function.l_donus(pwmx)
            check=support.sleep_function(-30,'a',cap,out,pwmx)#,cap_2)
            if check==0:
                break
            
            angle = gyro()[3]
            last,pid_i=motor_function.motor_fri(pwmx,340,0,0,angle)
            
            check=support.sleep_function(5,'t',cap,out,pwmx,speed=340)
            if check==0:
                break

#Motorlara güç verilir.
motor_function.start(1)
#PCA9685 objesi pwmx değişkenşne atanır
pwmx = Adafruit_PCA9685.PCA9685()
#PCA9685 frekansı 50 hertze set edilir
pwmx.set_pwm_freq(50)  # freq of the motors 50/60 Hz
#Ön kamera açısı
pwmx.set_pwm(3, 0, 310)
#Motor kalibrasyonları yapılır.
motor_function.calibration(pwmx)
'''
Kod çalıştırıldığında aracın 0 konumu set ediliyor (aracı havuza dik bir şekilde iken kodu çalıştırın) daha sonra
aracı havuza bırakana kadar bize süre lazım burada time.sleep(saniye) komutunu kullanın zero_point() fonksiyonu
aracı gyo dan gelen verilere göre havuza paralel konuma getirecektir. searching kodu çalışana kadar herhangi bir video kaydı yoktur.
'''

#A delay is added for the desired time.
print("Süre başladı")
time.sleep(25)


start_time=time.time()
end_time=0
diff_time=0

while diff_time<5:
    control.Pressure_Control(27,pwmx)
    end_time=time.time()
    diff_time=end_time-start_time
#arama algoritması başlatılır
control.zero_point(pwmx)
searching()

