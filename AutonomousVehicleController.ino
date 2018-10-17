// Autonomous Vehicle Controller for Arduino
// version 0.3 14.05.2018
// Michal K
//
// AP name: "autonomous" ; pass: "car12345" ; AP IP: 192.168.1.1/24 ; TCP server on port: 1307
// List of commands:
//  F[x] -> go forward for x steps (ex. F400) [1 <= x <= 2147483647]
//  B[x] -> go backward for x steps (ex. B800) [1 <= x <= 2147483647]
//  R[x] -> rotate right for x steps (ex. R200) [1 <= x <= 2147483647]
//  L[x] -> rotate left for x steps (ex. L300) [1 <= x <= 2147483647]
//  S[x] -> rotate servo on x degree position (ex. S90) [0 <= x <= 180]
//  D[x] -> measure distance with IR sensor, x measurements (ex. D10) [1 <= x <= 100]
//  A[x] -> measure acceleration with Waveshare 10 DoF IMU, x measurements (ex. A10) [1 <= x <= 100]
//  G[x] -> measure angular velocity with Waveshare 10 DoF IMU, x measurements (ex. G25) [1 <= x <= 100]
//  M[x] -> measure magnetic field with Waveshare 10 DoF IMU, x measurements (ex. M1) [1 <= x <= 100]
//  E[m0m1m2] -> change stepper motors step resolution (ex. E111) [m == 0 || 1]
//  N -> check nFAULT pin of DRVs8825 (ex. N)
//  H -> send heartbeat (ex. H)
//  X -> reset Arduino (ex. X)
//  Y -> restore Esp8266 factory settings, then you must manually reset Arduino (ex. Y)

#include <TimerThree.h> // library for timer
#include <Servo.h> // library for servo motor
#include <Wire.h> // library for I2C bus
// libraries for Waveshare 10 DoF IMU
#include <Adafruit_Sensor.h>
#include <MPU9255.h>

// pins
#define DRV1STP 4 // DRV8825#1 STEP pin
#define DRV1DIR 5 // DRV8825#1 DIR pin
#define DRV2DIR 6 // DRV8825#2 DIR pin
#define DRV2STP 7 // DRV8825#2 STEP pin
#define DRVM0 8 // DRVs8825 M0 pin
#define DRVM1 9 // DRVs8825 M1 pin
#define DRVM2 10 // DRVs8825 M2 pin
#define SRV 11 // servo pin
#define MICROSWT 12 // microswitch pin
#define DRVNFLT 13 // DRVs8825 nFAULT pin
#define IR_SERNSOR 1 // IR sensor analog pin
// accuracy settings for Waveshare 10 DoF IMU
#define ACCURACY_A 2 // accuracy for accelerometer [G] (2 -> +-2 G)
#define ACCURACY_G 250 // accuracy for gyroscope [degree/s]
#define ACCURACY_M 6 // accuracy for magnetometer [0.1 mGauss] (6 -> 0.6 mGauss; 1 Gauss == 0.0001 T)
// other settings
#define TIMER_PERIOD 50000 // timer overflow period [us] (50000 -> 50 ms)
#define EXT_AREF 1 // use external analog reference
#define IR_DELAY 25 // delay between IR sensor measurements [ms] (measurement time: 16.5(+-3.7) + 5 ms)
#define ACCEL_DELAY 1 // delay between acceleration measurements [ms] (accelerator data rate: 4 kHz)
#define GYRO_DELAY 1 // delay between angular velocity measurements [ms] (gyroscope data rate: 8 kHz)
#define MAG_DELAY 125 // delay between magnetic field measurements [ms] (magnetometer data rate: 8 Hz)
#define STEP_DELAY 1 // delay between stepper motor steps [ms]
#define WIFI_SPEED 115200 // serial baudrate for Esp8266 [baud/s]
#define WIFI_SEND_DELAY 250 // delay between sending config commands to Esp8266 [ms]
#define WIFI_SEND_TIMEOUT 5000 // time to wait for ">" prompt, then abort sending
#define HIT_STEP_BACK 0 // how many steps to do if microswitch was pressed (0 -> OFF)

Servo servo; // servo object
MPU9255 mpu(-1, ACCURACY_G, ACCURACY_A, ACCURACY_M); // object of Waveshare 10 DoF IMU sensors
float accelScale, gyroScale, magScale; // scale factors 
int* oVars; // output data
volatile bool hitFlag = false; // is microswitch pressed flag

void waveshareInit() {
  Wire.begin(); // initialization of I2C protocol
  TWBR = 24; // 400kHz I2C clock
  mpu.initMPU9250(); // initialization of accelerometer-gyroscope
  float magCalibration[3];
  mpu.initAK8963(magCalibration); // initialization of magnetometer
  // set accuracy scale factors
  switch (ACCURACY_A) {
    case 2: accelScale = 16384.0; break;
    case 4: accelScale = 8192.0; break;
    case 8: accelScale = 4096.0; break;
    case 16: accelScale = 2048.0; break;
    default: accelScale = 16384.0; break;
  }
  switch (ACCURACY_G) {
    case 250: gyroScale = 131.0; break;
    case 500: gyroScale = 65.5; break;
    case 1000: gyroScale = 32.8; break;
    case 2000: gyroScale = 16.4; break;
    default: gyroScale = 131.0; break;
  }
  switch (ACCURACY_M) {
    case 6: magScale = 0.6; break;
    case 15: magScale = 0.15; break;
    default: magScale = 1; break;
  }
}

void wifiSetup() {
  Serial1.begin(WIFI_SPEED); // connect to Esp8266 via UART
  Serial1.println("ATE0"); // switch off cmd echo
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CWMODE_DEF=2"); // AP mode
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CIPMODE=0"); // normal transmission mode
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CIPMUX=1"); // multiple connection mode
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CWDHCP_DEF=0,1"); // DHCP on
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CIPAP_DEF=\"192.168.1.1\",\"192.168.1.1\",\"255.255.255.0\""); // set IP
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CIPDINFO=1"); // show remote ip with +IPD
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CWSAP_DEF=\"autonomous\",\"car12345\",7,2,1,0"); // AP setup
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CIPSERVERMAXCONN=1"); // set maximum connections number
  delay(WIFI_SEND_DELAY);
  Serial1.println("AT+CIPSERVER=1,1307"); // start TCP/IP server
  delay(2 * WIFI_SEND_DELAY);
  Serial1.println("AT+CIPSTO=0"); // TCP server timeout (0 -> never timeout)
  delay(WIFI_SEND_DELAY);
}

void setup() {
  // pins setup
  pinMode(DRV1STP, OUTPUT);
  pinMode(DRV1DIR, OUTPUT);
  pinMode(DRV2DIR, OUTPUT);
  pinMode(DRV2STP, OUTPUT);
  pinMode(DRVM0, OUTPUT);
  pinMode(DRVM1, OUTPUT);
  pinMode(DRVM2, OUTPUT);
  servo.attach(SRV);
  pinMode(MICROSWT, INPUT);
  pinMode(DRVNFLT, INPUT);
  //
  Timer3.initialize(TIMER_PERIOD); // initialize timer3 and set overflow period
  Timer3.stop(); // but don't start the timer
  Timer3.attachInterrupt(chkHit); // attach callback as a timer overflow interrupt
  #if EXT_AREF == 1
  analogReference(EXTERNAL); // AREF to 3.3V
  #endif
  oVars = (int*) malloc(3 * sizeof(int)); // for output data
  waveshareInit();
  wifiSetup();
}

long step(long n) { // stepper motors steps
  for(long i = 0; i < n; i++) {
    if(hitFlag == true) { // check if hit flag was set
      hitFlag = false; // clear hit flag
      return i; // stop motors and return number of done steps
    }
    digitalWrite(4, HIGH);
    digitalWrite(7, HIGH);
    delayMicroseconds(20); // probably unecessary
    digitalWrite(4, LOW);
    digitalWrite(7, LOW);
    delay(STEP_DELAY); // delay between steps
  }
  return n;
}

long go(long n, bool dir) { // go forward or backward
  if(dir == true) { // set direction (true -> forward)
    digitalWrite(5, LOW);
    digitalWrite(6, HIGH);
  }
  else {
    digitalWrite(5, HIGH);
    digitalWrite(6, LOW);
  }
  return step(n); // do steps
}

long rotate(long n, bool dir) { // rotate left or right
  if(dir == true) { // set direction (true -> right)
    digitalWrite(5, LOW);
    digitalWrite(6, LOW);
  }
  else {
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);
  }
  return step(n); // do steps
}

void chkHit() { // check if microswitch is pressed
  if(digitalRead(MICROSWT) == HIGH) {
    Timer3.stop(); // stop timer (and stop checking microswitch)
    #if HIT_STEP_BACK > 0
    go(HIT_STEP_BACK, false); // go a little back
    #endif
    hitFlag = true; // set hit flag
  }
}

int chkDist(byte n) { // check distance with IR sensor
  long result = 0;
  for(byte i = 0; i < n; i++) { // n measurements
    delay(IR_DELAY);
    result += analogRead(IR_SERNSOR);
  }
  if(n > 1)
    result /= n; // arithmetic average
  return (int) result;
  //return (int) (result * 3.22265625); // value from sensor * (3.3/1024) * 1000 [mVolts]
}

void chkAccel(int* arg, byte n) { // check acceleration with Waveshare 10 DoF IMU
  float fAccelCount[3]; // raw data from Waveshare 10 DoF IMU
  for(byte i=0; i<3; i++)
    arg[i] = 0;
  for(byte i=0; i<n; i++) {  // n measurements
    delay(ACCEL_DELAY);
    mpu.readAccelData(fAccelCount); // read data from sensor
    for(byte j=0; j<3; j++) {
      fAccelCount[j] /= accelScale; // divide all elements by the scale factor
      arg[j] += (int) (fAccelCount[j] * 100);
    }
  }
  if(n > 1) {
    for(byte i=0; i<3; i++)
      arg[i] /= n; // arithmetic average
  }
}

void chkGyro(int* arg, byte n) { // check angular velocity with Waveshare 10 DoF IMU
  float fGyroCount[3]; // raw data from Waveshare 10 DoF IMU
  long result[] = {0, 0, 0};
  for(byte i=0; i<n; i++) {  // n measurements
    delay(GYRO_DELAY);
    mpu.readGyroData(fGyroCount); // read data from sensor
    for(byte j=0; j<3; j++) {
      fGyroCount[j] /= gyroScale; // divide all elements by the scale factor
      result[j] += (int) (fGyroCount[j] * 100);
    }
  }
  if(n > 1) {
    for(byte i=0; i<3; i++)
      arg[i] = result[i] / n; // arithmetic average
  }
  else {
    for(byte i=0; i<3; i++)
      arg[i] = result[i];
  }
}

void chkMag(int* arg, byte n) { // check magnetic field with Waveshare 10 DoF IMU
  float fMagCount[3]; // raw data from Waveshare 10 DoF IMU
  long result[] = {0, 0, 0};
  for(byte i=0; i<n; i++) {  // n measurements
    delay(MAG_DELAY);
    mpu.readMagData(fMagCount); // read data from sensor
    for(byte j=0; j<3; j++) {
      fMagCount[j] /= magScale; // divide all elements by the scale factor
      result[j] += (int) (fMagCount[j] * 100);
    }
  }
  if(n > 1) {
    for(byte i=0; i<3; i++)
      arg[i] = result[i] / n; // arithmetic average
  }
  else {
    for(byte i=0; i<3; i++)
      arg[i] = result[i];
  }
  /*
  float direction = atan2(f_magCount[1], f_magCount[2]); // compute magnetic direction using arcustangens2 function
  if(direction < 0)
    direction += 2 * PI;
  float degreeDirection = direction * 180 / PI; // radians to degrees conversion
  */
}

void chRes(bool m0, bool m1, bool m2) { // change step resolution for stepper motors
  if(m0 == true)
    digitalWrite(DRVM0, HIGH);
  else
    digitalWrite(DRVM0, LOW);
  if(m1 == true)
    digitalWrite(DRVM1, HIGH);
  else
    digitalWrite(DRVM1, LOW);
  if(m2 == true)
    digitalWrite(DRVM2, HIGH);
  else
    digitalWrite(DRVM2, LOW);
}

bool chkEngines() { // check DRVs8825 nFAULT pin
  return digitalRead(DRVNFLT);
}

void softwareReset() { // restart program from beginning (peripherals and registers are not reseted)
  free(oVars); // free memory alocated for output data
  asm volatile ("  jmp 0");  
}

void espFactorySettings() { // restore Esp8266 factory settings and hang program
  Serial1.println("AT+RESTORE"); // send restore command to Esp8266
  while(true);
}

int sendData(String msg, byte len) { // send something to client
  unsigned long timeout = 0;
  String sendCmd = "AT+CIPSEND=0,"; // send initializing command
  sendCmd.concat(len); // data length
  Serial1.println(sendCmd); // start transmission
  timeout = millis() + WIFI_SEND_TIMEOUT;
  while(millis() < timeout) { // wait for cmd prompt ">"
    if(Serial1.find(">") == true) {
      Serial1.println(msg); // send data
      return 0;
    }
  }
  return -1; // send status: failed
}

void loop() {
  String call = ""; // incoming command
  long cVar = -1; // incoming command argument
  long dStps = -1; // number of done steps
  byte zeros = -1; // number of zeros preceding when sending done steps
  char idx = -1; // index of ":" sign in +IPD
  for(byte i=0; i<3; i++)
    oVars[i] = -1; // output data
  bool m0 = false, m1 = false, m2 = false; // for chRes command
  String info = ""; // response/confirmation for commands
    
  while(call.length() == 0) { // wait for command
    if(Serial1.available() > 0) { // if something is in buffer...
      call = Serial1.readString(); // ...read it
      if(call.indexOf("+IPD") == -1) // if it's not incoming command wait for more
        call = "";  
    }
  }

  idx = call.indexOf(':'); // find beginning of command
  cVar = call.substring(idx+2).toInt(); // find command argument

  switch(call.charAt(idx+1)) { // handle commands
    case 'F': // go forward
    if(cVar > 0) {
      Timer3.start(); // start the timer to check collisions
      dStps = go(cVar, true); // go and get number of done steps
      info = "F";
      zeros = (byte)log10(cVar) - (byte)log10(dStps);
      for(byte i=0; i<zeros; i++) // number of done steps may be preceded by 0's
        info.concat("0");
      info.concat(dStps);
      sendData(info, info.length() + 1); // send how many steps were done
      Timer3.stop();
    }
    //else
      // invalid go forward arg
    break;

    case 'B': // go backward
    if(cVar > 0) {
      go(cVar, false);
      sendData("B", 2); // send confirmation
    }
    //else
      // invalid go backward arg
    break;

    case 'R': // rotate right
    if(cVar > 0) {
      Timer3.start(); // start the timer to check collisions
      dStps = rotate(cVar, true); // rotate and get number of done steps
      info = "R";
      zeros = (byte)log10(cVar) - (byte)log10(dStps);
      for(byte i=0; i<zeros; i++) // number of done steps may be preceded by 0's
        info.concat("0");
      info.concat(dStps);
      sendData(info, info.length() + 1); // send how many steps were done
      Timer3.stop();
    }
    //else
      // invalid rotate right arg
    break;

    case 'L': // rotate left
    if(cVar > 0) {
      Timer3.start(); // start the timer to check collisions
      dStps = rotate(cVar, false);
      info = "L"; // rotate and get number of done steps
      zeros = (byte)log10(cVar) - (byte)log10(dStps);
      for(byte i=0; i<zeros; i++) // number of done steps may be preceded by 0's
        info.concat("0");
      info.concat(dStps);
      sendData(info, info.length() + 1); // send how many steps were done
      Timer3.stop();
    }
    //else
      // invalid rotate left arg
    break;

    case 'S': // move servo
    if(cVar >= 0 && cVar <= 180) {
      servo.write((int)abs(cVar - 180)); // swap 0* <-> 180*
      sendData("S", 2); // send confirmation
    }
    //else
      // invalid servo arg
    break;

    case 'D': // check distance with IR sensor
    if(cVar >= 1 && cVar <= 100) {
      oVars[0] = chkDist((byte)cVar);
      info = "D";
      if(oVars[0] < 0)
        info.concat("ERR!");
      else if(oVars[0] < 10)
        info.concat("000");
      else if(oVars[0] < 100)
        info.concat("00");
      else if(oVars[0] < 1000)
        info.concat("0");
      if(oVars[0] >= 0)
        info.concat(oVars[0]);
      sendData(info, 6); // send distance
    }
    //else
      // invalid chkDist arg
    break;

    case 'A': // check acceleration with Waveshare 10 DoF IMU
    if(cVar >= 1 && cVar <= 100) {
      chkAccel(oVars, (byte)cVar);
      info = "A";
      // Ax
      if(oVars[0] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[0]) < 10)
        info.concat("00");
      else if(abs(oVars[0]) < 100)
        info.concat("0");
      info.concat(abs(oVars[0]));
      // Ay
      if(oVars[1] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[1]) < 10)
        info.concat("00");
      else if(abs(oVars[1]) < 100)
        info.concat("0");
      info.concat(abs(oVars[1]));
      // Az
      if(oVars[2] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[2]) < 10)
        info.concat("00");
      else if(abs(oVars[2]) < 100)
        info.concat("0");
      info.concat(abs(oVars[2]));
      sendData(info, 14); // send acceleration data
    }
    //else
      // invalid chkAccel arg
    break;

    case 'G': // check angular velocity with Waveshare 10 DoF IMU
    if(cVar >= 1 && cVar <= 100) {
      chkGyro(oVars, (byte)cVar);
      info = "G";
      // Gx
      if(oVars[0] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[0]) < 10)
        info.concat("0000");
      else if(abs(oVars[0]) < 100)
        info.concat("000");
      else if(abs(oVars[0]) < 1000)
        info.concat("00");
      else if(abs(oVars[0]) < 10000)
        info.concat("0");
      info.concat(abs(oVars[0]));
      // Gy
      if(oVars[1] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[1]) < 10)
        info.concat("0000");
      else if(abs(oVars[1]) < 100)
        info.concat("000");
      else if(abs(oVars[1]) < 1000)
        info.concat("00");
      else if(abs(oVars[1]) < 10000)
        info.concat("0");
      info.concat(abs(oVars[1]));
      // Gz
      if(oVars[2] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[2]) < 10)
        info.concat("0000");
      else if(abs(oVars[2]) < 100)
        info.concat("000");
      else if(abs(oVars[2]) < 1000)
        info.concat("00");
      else if(abs(oVars[2]) < 10000)
        info.concat("0");
      info.concat(abs(oVars[2]));
      sendData(info, 20); // send angular velocity data
    }
    //else
      // invalid chkGyro arg
    break;

    case 'M': // check magnetic field with Waveshare 10 DoF IMU
    if(cVar >= 1 && cVar <= 100) {
      chkMag(oVars, (byte)cVar);
      info = "M";
      // Mx
      if(oVars[0] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[0]) < 10)
        info.concat("0000");
      else if(abs(oVars[0]) < 100)
        info.concat("000");
      else if(abs(oVars[0]) < 1000)
        info.concat("00");
      else if(abs(oVars[0]) < 10000)
        info.concat("0");
      info.concat(abs(oVars[0]));
      // My
      if(oVars[1] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[1]) < 10)
        info.concat("0000");
      else if(abs(oVars[1]) < 100)
        info.concat("000");
      else if(abs(oVars[1]) < 1000)
        info.concat("00");
      else if(abs(oVars[1]) < 10000)
        info.concat("0");
      info.concat(abs(oVars[1]));
      // Mz
      if(oVars[2] >= 0)
        info.concat("+");
      else
        info.concat("-");
      if(abs(oVars[2]) < 10)
        info.concat("0000");
      else if(abs(oVars[2]) < 100)
        info.concat("000");
      else if(abs(oVars[2]) < 1000)
        info.concat("00");
      else if(abs(oVars[2]) < 10000)
        info.concat("0");
      info.concat(abs(oVars[2]));
      sendData(info, 20); // send magnetic field data
    }
    //else
    // invalid chkMag arg
    break;

    case 'E': // change step resolution for stepper motors
    if(cVar / 100 == 1)
      m0 = true;
    if(cVar % 100 / 10 == 1)
      m1 = true;
    if(cVar % 100 % 10 / 1 == 1)
      m2 = true;
    chRes(m0, m1, m2);
    sendData("E", 2); // send confirmation
    break;

    case 'N': // check DRVs8825 nFAULT pin
    if(chkEngines() == true)
      sendData("N1", 3); // send nFAULT status
    else
      sendData("N0", 3);
    break;

    case 'H': // send heartbeat
    sendData("H", 2); // send
    break;

    case 'X': // reset Arduino
    softwareReset();
    break;

    case 'Y': // restore Esp8266 factory settings and hang program
    espFactorySettings();
    break;
   
    //default:
    // undefined call
    //break;
  }
}
