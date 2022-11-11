#define RECEIVER_COMMUNICATION_TIMEOUT 200
#define RECEIVER_MIN_VALUE 306 
#define RECEIVER_MAX_VALUE 1693
#define QUADCOPTER_MAX_TILT_ANGLE 30
#define INTERRUPT_PIN 2
#define IMU_COMMUNICATION_TIMEOUT 200

#define ACCEL_OFFSET_X -1370
#define ACCEL_OFFSET_Y 1293
#define ACCEL_OFFSET_Z 1358
#define GYRO_OFFSET_X 82
#define GYRO_OFFSET_Y -32
#define GYRO_OFFSET_Z 11
//-1378.00000,  1283.00000, 1354.00000, 83.00000, -32.00000,  10.00000
//-1376.00000,  1283.00000, 1354.00000, 83.00000, -32.00000,  11.00000

//-1234.00000,  1255.00000, 1330.00000, 81.00000, -31.00000,  12.00000
//


#define MIN_MOTOR_PULSE_WIDTH 1000
#define MAX_MOTOR_PULSE_WIDTH 2000

#define KP_roll 1.00
#define KI_roll 0.0005
#define KD_roll 400.00


struct Orientation {
  double YawAngle;
  double PitchAngle;
  double RollAngle;
};

struct IMU_Values {
  bool Error;
  bool NewDataAvailable;
  unsigned long DeltaTime;
  struct Orientation CurrentOrientation;
};
