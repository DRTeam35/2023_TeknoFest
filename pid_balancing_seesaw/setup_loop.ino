float V, P,V_ort,P_ort,V_tot,P_tot,i;
const float  OffSet = 0.540 ;
float start_time,end_time,time_diff;

void setup() {
  initializeIMU();
  Serial.begin(115200);
  start_time=millis();
}

void loop() {
  struct IMU_Values imuValues = GetIMU_Values();

  V = analogRead(A1) * 5.00 / 1024;     //Sensor output voltage
  
  P = (V - OffSet) * 400;             //Calculate water pressure
  if (i<5.00){
      i=i+1.00;
      V_tot=V_tot+V;
      P_tot=P_tot+P;
  }
  if(i==5.00){
      V_ort=V_tot/i;
      P_ort=P_tot/i;
      V_tot=0.00;
      P_tot=0.00;          
      i=0.00;
  }
  end_time=millis();
  time_diff=end_time-start_time;
  if(time_diff>10){
  
  //Serial.print("Voltage:");
  Serial.print(V_ort, 3);
  Serial.print(" ");
  
  //Serial.print(" Pressure:");
  Serial.print(P_ort, 2);
  Serial.print(" ");

  Serial.print(imuValues.DeltaTime);
  Serial.print(" ");
  
  //Serial.print("Yaw:");
  Serial.print(imuValues.CurrentOrientation.YawAngle);
  Serial.println(" ");

  //Serial.print("Delta_Time:");
  
  start_time=millis();
  }


  if(imuValues.NewDataAvailable == false){
    return;
  }

}
