//Der Kalman-Filter von Kristian Lauszus wurde leicht abgeändert


#include <SoftwareSerial.h>
#include <ownPID.h>
#include <Kalman.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Wire.h>
#define RESTRICT_PITCH
#define MOT_X1_PIN 10
#define MOT_X2_PIN 11
#define MOT_Y1_PIN 5
#define MOT_Y2_PIN 6

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);



const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

float xspeed{};
float yspeed{};

float OutOfRange{};
float Counter{};



Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
ownPID PIDRoll (18, 0.5, 10, -3.44);                                                         //Kp, Ki, Kd, Setpoint festlegen!
ownPID PIDPitch (19, 0.7, 12, -0.31);                                                        //Kp, Ki, Kd, Setpoint festlegen!

float owntimer {};

void setup() {
  pinMode(2, INPUT_PULLUP);                                             //für fault von Driver

  pinMode(MOT_X1_PIN, OUTPUT);
  pinMode(MOT_X2_PIN, OUTPUT);

  digitalWrite(MOT_X1_PIN, LOW);
  digitalWrite(MOT_X2_PIN, LOW);

  pinMode(MOT_Y1_PIN, OUTPUT);
  pinMode(MOT_Y2_PIN, OUTPUT);

  digitalWrite(MOT_Y1_PIN, LOW);
  digitalWrite(MOT_Y2_PIN, LOW);

  if (!gyro.begin()) {                                                                 //Sensor 21002C initialisieren
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1);
  }

  if (!accelmag.begin(ACCEL_RANGE_4G)) {                                               //Sensor 8700 initialisieren
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1);

  }
  sensor_t sensor;
  gyro.getSensor(&sensor);
  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);
  sensors_event_t event;                      //21002C
  gyro.getEvent(&event);                      //21002C
  sensors_event_t aevent, mevent;             //8700
  accelmag.getEvent(&aevent, &mevent);        //8700

  Serial.begin(115200);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */

  accX = aevent.acceleration.x;
  accY = aevent.acceleration.y;
  accZ = aevent.acceleration.z;

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

}


void loop() {

  sensors_event_t event;                      //21002C
  gyro.getEvent(&event);                      //21002C
  sensors_event_t aevent, mevent;             //8700
  accelmag.getEvent(&aevent, &mevent);        //8700

  float gyroX = event.gyro.x;
  float gyroY = event.gyro.y;
  float gyroZ = event.gyro.z;
  float accX = aevent.acceleration.x;
  float accY = aevent.acceleration.y;
  float accZ = aevent.acceleration.z;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;


  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  xspeed = PIDRoll.Output_fu(kalAngleX);
  yspeed = PIDPitch.Output_fu(kalAngleY);



  xspeed = constrain(xspeed, -255, 255);
  yspeed = constrain(yspeed, -255, 255);


  if (OutOfRange) {
    xspeed = 0;
    yspeed = 0;
    if (millis() - Counter > 15000) {
      OutOfRange = false;
      Counter = 0;
    }
  } else {

    if (kalAngleX > 30) {
      OutOfRange = true;
      Counter = millis();
    };

    if (kalAngleX < -30) {
      OutOfRange = true;
      Counter = millis();
    };

    if (kalAngleY > 30) {
      OutOfRange = true;
      Counter = millis();
    };

    if (kalAngleY < -30) {
      OutOfRange = true;
      Counter = millis();
    };


  }

  if (xspeed >= 0) {
    digitalWrite(MOT_X2_PIN, LOW);
    analogWrite(MOT_X1_PIN, xspeed);
  };
  if (xspeed < 0) {
    xspeed = -xspeed;
    digitalWrite(MOT_X1_PIN, LOW);
    analogWrite(MOT_X2_PIN, xspeed);
  };


  if (yspeed >= 0) {
    digitalWrite(MOT_Y1_PIN, LOW);
    analogWrite(MOT_Y2_PIN, yspeed);
  };
  if (yspeed < 0) {
    yspeed = -yspeed;                                                     //Betrag gesetzt, da -1 wie 255 ist (max speed)
    digitalWrite(MOT_Y2_PIN, LOW);
    analogWrite(MOT_Y1_PIN, yspeed);
  };




  if (millis() - owntimer > 200) {                              //für Ziegler-Nichols-Methode nötig
    Serial.print("Y: ");
    Serial.println(yspeed);
    Serial.print("X: ");
    Serial.println(xspeed);

    owntimer = millis();

  };
}
