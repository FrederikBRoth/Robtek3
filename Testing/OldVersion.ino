#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
MPU6050 mpu;

#define DPRINTSTIMER(t) for (static unsigned long SpamTimer; (unsigned long)(millis() - SpamTimer) >= (t); SpamTimer = millis())
#define DPRINTSFN(StrSize, Name, ...)             \
  {                                               \
    char S[StrSize];                              \
    Serial.print("\t");                           \
    Serial.print(Name);                           \
    Serial.print(" ");                            \
    Serial.print(dtostrf((float)__VA_ARGS__, S)); \
  } // StringSize,Name,Variable,Spaces,Percision
#define DPRINTLN(...) Serial.println(__VA_ARGS__)


// supply your own gyro offsets here, scaled for min sensitivity use MPU6050_calibration.ino
// -4232  -706  1729  173 -94 37
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = {-242, 1330, 1681, -30, 181, -40};

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
}
// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
int FifoAlive = 0; // tests if the interrupt is triggering
int IsAlive = -20; // counts interrupt start at -20 to get 20+ good values before assuming connected
// MPU control/status vars
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
byte StartUP = 100;  // lets get 100 readings from the MPU before we start trusting them (Bot is not trying to balance at this point it is just starting up.)

void MPU6050Connect()
{
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize(); // same

  if (devStatus != 0)
  {
    Serial.print("Connection to MPU failed");
  }
  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000);     // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
}

// ================================================================
// ===                    MPU DMP Get Data                      ===
// ================================================================
void GetDMP()
{ 
  mpu.resetFIFO();
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize)
    fifoCount = mpu.getFIFOCount();
  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  MPUMath();
}

// ================================================================
// ===                        MPU Math                          ===
// ================================================================
float Yaw, Pitch, Roll;
void MPUMath()
{
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  //Calculates degrees from radians
  Yaw = (ypr[0] * 180.0 / M_PI);
  Pitch = (ypr[1] * 180.0 / M_PI);
  Roll = (ypr[2] * 180.0 / M_PI);
  // DPRINTSTIMER(100) {
  DPRINTSFN(15, "\tYaw:", Yaw, 6, 1);
  DPRINTSFN(15, "\tPitch:", Pitch, 6, 1);
  DPRINTSFN(15, "\tRoll:", Roll, 6, 1);
  DPRINTLN();
  // }
}
// ================================================================
// ===                         Setup                            ===
// ================================================================
void setup()
{
  Serial.begin(115200); // 115200
  while (!Serial)
    ;
  Serial.println("i2cSetup");
  i2cSetup();
  Serial.println("MPU6050Connect");
  MPU6050Connect();
  Serial.println("Setup complete");
  pinMode(buttonPin, INPUT_PULLUP);
}
// ================================================================
// ===                          Loop                            ===
// ================================================================
void loop()
{
  // if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
  //   GetDMP();
  // }
  buttonState = digitalRead(buttonPin);
  if (buttonState == LOW)
  {
    Serial.println("Button Pressed");
    buttonPressed = true;
    GetDMP();
  }
}
