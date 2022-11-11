import numpy as np
import serial
import motor_function
'''
Bu modüle içeriğinde:
    def gyro():
    def turn(angle,<pwm>):
    def zero_point(<pwm>,check=1):
    def PID():
fonksiyonlarını içerir.
###########################################<3##################################################
#                                                                                             #
#def gyro():                                                                                  #
#        Input paramater: None                                                                #
#        return:string array in the sensor data.                                              #
#               return deger['Preassure Sensor voltage','Preassure','Pitch','Yaw']            #
#                                                                                             #
#def turn(angle,<pwm>):                                                                       #
#        Input paramater:Denetlenmesi istenilen açı değeri angle değişkeni olarak alınır.     #
#                        angle değişkeni +3,-3 açı toleransında değerlendirilir.              #
#                        Motor durdurma komutu kullanabilmek amacıyla PCA9685 objesi alınır   #
#                                                                                             #
#def zero_point(<pwm>,check=1):                                                               #
#        Input paramater:Motor komutlarını kullanabilmek amacıyla PCA9685 objesi alınır       #
#                        check default değer atamasıdır.Fonksiyon dışardan değer beklemez     #
#def PID():                                                                                   #
#                                                                                             #
#                                                                                             #
###########################################<3##################################################
'''

#Yalnızca sensör bilgisi alınır.
def gyro():
    #ardunio seri haberleşme bağlantısı yapılır
    '''Burada tekrar tekrar bağlanmamızın sebebi var.Tekrar tekrar bağlanılmazsa.
        Kod ekrana seriala basılan tüm satırları tek tek okumaya çalışıyor
        ardunio daha hızlı çalıştığından dolayı bilgiyi arkadan takip ediyoruz.
        Bu yüzden her seferinde sıfırdan bağlanıp Ardunionın seriala ilk bastığı değeri alıyoruz
    '''
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    #Ardunio serial ekranından bir satır okunur
    #Sensör bilgileri tek tek ulaşılamaz tek string olarak ardunionun seriala bastığı string alınır
    line = ser.readline().decode('utf-8').rstrip()
    # bazen yalnış bilgiler gelebildiğinden dolayı beklenen değerin uzunluğu 15'ten büyük olmalıdır
    if len(line) > 15:
        # Ardunioda her bilgi arasına boşluk konuldu.Burada boşluklar ayrılarak
        # Tek string olan bilgi deger arrayine atanıyor
        # deger['Preassure voltage','preassure','Pitch','Yaw']
        deger = line.split(" ")
        '''
        Bazı durumlarda hata olup farklı değerler okunuyor bu yüzden bunu ayıklıyoruz fakat
        fonksiyon geri döngü bilgisi vermezse kod çıldırır. Bir çok kodda gyro bilgisi alınması
        zorunlu.Bu yüzden her yeni bilgi alındığında projede bulunan gyro.txt açılır ve içeriği
        boşaltılıp yeni bilgi yazılır. Bilgi alınamadığında ise gyro.txt sadece okumak için açılır
        ve bu son değer geri döndürülür.
        '''
        # Hata olmaması amacıyla 4 bilgi içerip içermediği değerlendirilir
        if len(deger) == 4:
            # gyro.txt belgesi 'w' formatında açılır
            f = open('gyro.txt', "w")
            # Yeni alına gyro bilgisi gyro.txt'ye yazdırılır
            f.write(line)
            # dosya kağatılır
            f.close()
            # deger string arrayi geri döndürülür
            return deger
        # deger string arrayi 4 elemandan küçükse
        else:
            # gyro.txt'te 'r' formatında açılır.
            f = open('gyro.txt', "r")
            # Son kaydedilen gyro değeri ölçülür
            deger = f.read()
            # Tek string olan bilgi deger arrayine atanıyor
            # deger['Preassure voltage','preassure','Pitch','Yaw']
            deger = deger.split(" ")
            # Dosya kapatılıyor
            f.close()
            # Bilgi 4 değerden oluimalı
            if len(deger)==4:
                # deger string arrayi geri döndürülür
                return deger
    # Arduniodan gelen bilgi stringinin uzunluğu 15 ten kısa ise else şartlı ifadesi uygulanır
    else:
        # gyro.txt'te 'r' formatında açılır.
        f = open('gyro.txt', "r")
        # Son kaydedilen gyro değeri ölçülür
        deger = f.read()
        # Tek string olan bilgi deger arrayine atanıyor
        # deger['Preassure voltage','preassure','Pitch','Yaw']
        deger = deger.split(" ")
        # Dosya kapatılıyor
        f.close()
        # Bilgi 4 değerden oluimalı
        if len(deger) == 4:
            # deger string arrayi geri döndürülür
            return deger

def Pressure_Control(pressure,pwm):
    if pressure!=0:
        #Sensör değerlerini çekiyoruz
        pressure_Up=pressure-1
        pressure_Down=pressure+1
        sensor=gyro()
        #Basınç değerini float olarak preassure değişkenine atıyoruz
        Sensor_Value=float(sensor[1])
        #Aracı tepede tutmak için oluşturuldu
        if Sensor_Value > float(pressure_Down):
            motor_function.motor_u(pwm,285)
        elif Sensor_Value < float(pressure_Up):
            motor_function.motor_d(pwm,350)
        else:
            motor_function.motor_u(pwm,318)
        
def turn(angle,pwm):
    #Sola dönüşler için artı  !!!!!!!
    #Sağa dönüşler için eksi  !!!!!!!
    #Açı tolerans değeri
    tolerans=5
    #Gyrodan değerler alınır
    deger=gyro()
    #Yaw açısı deger arrayin çekilerek folata dönüştürülür
    line=float(deger[3])
    #gyronun yaw açısı,istenilen açı değerinin +5 -5 değerleri arasında ise motorlar durdurulur
    sag_sinir=(angle+tolerans)%360
    sol_sinir=(angle-tolerans)%360
    aralik=[]
    value=sol_sinir
    for x in range(0,tolerans*2):
        aralik.append(int(value))
        value=(value+1)%361
    #If the yaw angle of the gyron is between +5 -5 values of the desired angle value, the motors are stopped.
    if int(line) in aralik:
        motor_function.stop(pwm)
        #motorlar durdurulursa geri döngü olarak 0 değeri döndürülür
        return 0
    else:
        #eğer istenilen değere ulaşılamadıysa 1 değeri döndürülür
        return 1

#Bu fonksiyon aracın konumunu 0 değerine getirilmesi için yazılmıştır
def zero_point(pwm,check=1):
    #Anlık gyro değeri alınır
    line_start=gyro()
    #line_start arrayinden yaw açısı çekilerek floata dönüştürülür
    yaw=float(line_start[3])
    #Yaw açısı +3 -3 değer aralığına gelene kadar döngü devam eder
    while check==1:
        #yaw değeri negatif ise sol dönüş yapılır
        if 0<=yaw<=180:
            motor_function.l_donus(pwm,330)
        #If yaw value is positive, right turn is done
        elif 180<yaw<=360:
            motor_function.r_donus(pwm,330)
        #Açı değeri kontrol edilir eğer açı istenile değerler arasındaysa
        #check değeri 0'a çekilir döngüden çıkılır
        check=turn(0,pwm)
