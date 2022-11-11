class PID_class():
    def __init__(self,last_error,pid_i):
        self.yaw_pid_i=pid_i
        self.yaw_last_error=last_error
        self.yaw_control_signal=0.0
    def resetPidVariables(self):
        self.yaw_pid_i=0.0
        self.yaw_last_error=0.0
    def calculateMotorPowers(self,KP_yaw,KI_yaw,KD_yaw,istenilen_hız,istenilen_acı,imu_yaw,imu_deltaTime):
        istenilen_acı=float(istenilen_acı)
        imu_yaw=float(imu_yaw)
        yawError=float(istenilen_acı)-float(imu_yaw)
        if (istenilen_acı>=0 and istenilen_acı<=90) and (imu_yaw>=270 and imu_yaw<=360):
            yawError=yawError %360
        elif (imu_yaw>=0 and imu_yaw<=90) and (istenilen_acı>=270 and istenilen_acı<=360):
            yawError=-abs(360-abs(yawError))
        self.yaw_control_signal= self.getControlSignal(yawError, KP_yaw, KI_yaw, KD_yaw, int(imu_deltaTime))

        motorpower=[]
        motorpower.append(istenilen_hız+self.yaw_control_signal)
        motorpower.append(istenilen_hız-self.yaw_control_signal)
        motorpower.append(self.yaw_last_error)
        motorpower.append(self.yaw_pid_i)
        if(motorpower[0]>350):
            motorpower[0]=350
        if (motorpower[1]>350):
            motorpower[1]=350
        if (motorpower[0]<318):
            motorpower[0]=318
        if (motorpower[1]<318):
            motorpower[1]=318
        return motorpower
    def getControlSignal(self,error,kp,ki,kd,delta_time):
        pid_p=error
        pid_d=(error-self.yaw_last_error)/delta_time
        self.yaw_pid_i+=error*delta_time
        control_signal=(kp*pid_p)+(ki*self.yaw_pid_i)+(kd*pid_d)
        self.yaw_last_error=error
        return control_signal
