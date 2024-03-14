
//#include <i2c_t3.h>
#include "Wire.h"
#include "Alarms.h"
#include "I2Cdev.h"
#include "USFSMAX.h"
#include "Sensor_cal.h"
#include "IMU.h"
#include "Globals.h"
#include "Types.h"
#include "def.h"

// Instantiate class objects
I2Cdev     i2c_0(&SENSOR_0_WIRE_INSTANCE);
USFSMAX    USFSMAX_0(&i2c_0, 0);
IMU        imu_0(&USFSMAX_0, 0);
Sensor_cal sensor_cal(&i2c_0, &USFSMAX_0, 0);

// Declare global scope utility functions
void       ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM);
void       FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM);
void       DRDY_handler_0();
void       SerialInterface_handler();

#include <math.h>
#include <CrsfSerial.h>
CrsfSerial crsf(Serial3, CRSF_BAUDRATE); // pass any HardwareSerial port
void packetChannels() { } 

#include <string.h>
const char s[2] = ",";
char *token;
const byte num_ch = 5;
float con_com[num_ch];
const byte numChars = 255;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
const byte CRSF_num_ch = 13;
int CRSF_ch[CRSF_num_ch];
float thr_CRSF, thr_CRSF_01, pit_CRSF, rol_CRSF;
float thrust_m1, thrust_m2;

float send_block[4];
String sending_msg;
int sending_enabled = 0;

float arm, thr_m1, thr_m2, ang_s1, ang_s2;
#include <Servo.h>
Servo M_1, M_2;
#include <SPI.h>

#include <SD.h>
File myFile;
char filename[20];
const int chipSelect = BUILTIN_SDCARD;
int sd_counter = 0;
const int arr_size = 128;
float sd_buff1[arr_size];
float sd_buff2[arr_size];
float sd_buff3[arr_size];
float sd_buff4[arr_size];
float sd_buff5[arr_size];
float sd_buff6[arr_size];
float sd_buff7[arr_size];
float sd_buff8[arr_size];
float sd_buff9[arr_size];
float sd_buff10[arr_size];
float sd_buff11[arr_size];
float sd_buff12[arr_size];
float sd_buff13[arr_size];
float sd_buff14[arr_size];
float sd_buff15[arr_size];
int data_record_on_off = 1;

#include <Adafruit_INA260.h>
Adafruit_INA260 ina260 = Adafruit_INA260();
int ina_sensor_on_off = 1;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  PNI BOARD calibration
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DRDY        9
#define SSN         10
long mag_x, mag_y, mag_z, mag_x_corr, mag_y_corr, mag_z_corr;
long x_off, y_off, z_off, scale_x, scale_y, scale_z;
float avg_delta[3] = {0, 0, 0}, avg_delta_xyz;
float mag_offset[3] = { 176.50, -59.00, 2392.50}, mag_scale[3] = {0.905, 0.942, 1.201};
float mag_max[3] = { 8848.00, 8268.00, 8926.00}, mag_min[3] = { -8495.00, -8386.00, -4141.00}, mag_temp[3] = {0, 0, 0};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  IMU BOARD calibration
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long mag_x_mpu, mag_y_mpu, mag_z_mpu, mag_x_corr_mpu, mag_y_corr_mpu, mag_z_corr_mpu;
long x_off_mpu, y_off_mpu, z_off_mpu, scale_x_mpu, scale_y_mpu, scale_z_mpu;
float avg_delta_mpu[3] = {0, 0, 0}, avg_delta_xyz_mpu;
float mag_offset_mpu[3] = { -27.50, -6.50, -20.50}, mag_scale_mpu[3] = {0.9779, 0.9779, 1.047};
float mag_max_mpu[3] = { 78.00, 99.00, 78.00}, mag_min_mpu[3] = { -133.00, -112.00, -119.00}, mag_temp_mpu[3] = {0, 0, 0};

float roll_angle, flip_angle;
float roll_angle_off = 0;
float roll_angle_off_prev = 0;
const int sum_eHeading_size = 20;
float sum_eHeading_vector[sum_eHeading_size];
int sum_eHeading_firstIndex = 0;
float sum_eHeading;
float theta_off;

int engageThr = 0;                // Current engageThr
int previousengageThr = 0;
unsigned long startTime = 0;
int counterActionPerformed = 0;
int engageThrFlag = 0;
const unsigned long actionDelay = 1000;           // (in milliseconds)
const unsigned long actionInterval = 1400;        // (in milliseconds)
unsigned long elapsedTime;

int engageFlip = 0;                // T-mode to A-mode
int previousengageFlip = 0;        // Previous engageFlip
unsigned long startTimeFlip = 0;
int counterFlipPerformed = 0;
int engageFlipFlag = 0;
const unsigned long FlipDelay = 1000;       // (in milliseconds)
const unsigned long FlipInterval = 2000;     // (in milliseconds)
unsigned long elapsedTimeFlip;

int engageFlip2 = 0;                // A-mode to T-mode
int previousengageFlip2 = 0;
unsigned long startTimeFlip2 = 0;
int counterFlip2Performed = 0;
int engageFlip2Flag = 0;
const unsigned long Flip2Delay = 1000;       // (in milliseconds)
const unsigned long Flip2Interval = 1000;    // (in milliseconds)
unsigned long elapsedTimeFlip2;

unsigned long currentTimeLoop, previousTimeLoop;
double elapsedTimeLoop;
float currentTmodeHeading, desiredTmodeHeading, tmodeHeading;

float thrust_roll1 = 0;
float thrust_roll2 = 0;
float thr_rol_mode, thr_head, m1_rol_mode, m2_rol_mode;
float val1, val2, val3, val4;
float des_stop_ang;

float new_flip_max_thr, new_flip_min_thr;
int raw_acc_x, raw_acc_y, raw_acc_z;

int state = 1;
unsigned long startdelayforflipTime = 0;
unsigned long elapsedTimeforflipdelay;
const unsigned long FlipStateDelay = 1000;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
  Serial.println("Setup start");
  pinMode(USFS_GND, OUTPUT);                               // Set USFSMAX ground pin
  digitalWrite(USFS_GND, LOW);
  pinMode(USFS_VCC, OUTPUT);                               // Power up the USFSMAX
  digitalWrite(USFS_VCC, HIGH);

  Serial.begin(115200);
  Serial2.begin(115200);
  Serial4.begin(115200);
  delay(4000);

  M_1.attach(5);  M_2.attach(6);
  M_1.writeMicroseconds(1500);
  M_2.writeMicroseconds(1500);
  Serial.println("Motors attached");

  con_com[0] = 0.0;
  con_com[1] = 0.0;
  con_com[2] = 0.0;
  con_com[3] = 0.0;
  con_com[4] = 0.0;
  delay(500);
  crsf.onPacketChannels = &packetChannels;

  delay(500);
  pni_setup();
  Serial.println("Radio PNI done");

  delay(500);
  //  ina260.begin();
  if (data_record_on_off == 1) sd_card_setup();


  // Set up DRDY interrupt pin
  pinMode(INT_PIN, INPUT);

  // Assign Indicator LED
  //  LEDPIN_PINMODE;
  //  Alarms::blueLEDoff();

  // Initialize USFSMAX_0 I2C bus
  SENSOR_0_WIRE_INSTANCE.begin();
  delay(100);
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                     // Set I2C clock speed to 100kHz cor configuration
  delay(2000);

  // Do I2C bus scan if serial debug is active
  // Initialize USFSMAX_0

  USFSMAX_0.init_USFSMAX();                                                                                          // Configure USFSMAX and sensors
  SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                        // Set the I2C clock to high speed for run-mode data collection
  delay(100);
  Serial.println("USFS attached");
  // Attach interrupts
  attachInterrupt(INT_PIN, DRDY_handler_0, RISING);                                                                  // Attach DRDY interrupt

  // Calculate geomagnetic calibration parameters for your location (set in "config.h")
  Mv_Cal  = M_V;                                                                                                     // Vertical geomagnetic field component
  Mh_Cal  = M_H;                                                                                                     // Horizontal geomagnetic field component
  M_Cal   = sqrt(Mv_Cal * Mv_Cal + Mh_Cal * Mh_Cal);                                                                 // Geomagnetic field strength
  Del_Cal = atan(Mv_Cal / Mh_Cal);                                                                                   // Geomagnetic inclination or "Dip" angle

  calibratingG[0] = 1;
  Start_time = micros();                                                                                             // Set sketch start time
  previousTimeLoop = micros();
  currentTmodeHeading = 0;
  desiredTmodeHeading = 0;
  roll_angle_off = 0;
  tmodeHeading = 0;
  engageThr = 0;
  engageFlip = 0;
  engageFlip2 = 0;
  Serial.println("Setup done");
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void loop()
{
  // Calculate loop cycle time
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime;

  currentTimeLoop = micros();
  elapsedTimeLoop = (currentTimeLoop - previousTimeLoop) / 1000000.0;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  Receive data and Transmitter
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  recvWithEndMarker();
  showNewData();
  //  arm = con_com[0];
  val1 = con_com[1];
  val2 = con_com[2];
  val3 = con_com[3];
  val4 = con_com[4];

  assign_RX_channels();
  arm = CRSF_ch[5];


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  IMU calculations
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // Manage gyro cal
  if (calibratingG[0])
  {
    calibratingG[0] = 0;
    sensor_cal.GyroCal();
  }
  if (data_ready[0] == 1)
  {
    data_ready[0] = 0;
    ProcEventStatus(&i2c_0, 0);                                                                                      // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
    FetchUSFSMAX_Data(&USFSMAX_0, &imu_0, 0);                                                                        // USFSMAX instance 0, IMU calculation instance 0 and Sensor instance 0
  }

  USFSMAX_0.GetMxMy();

  mag_x_mpu = magData[0][0];
  mag_y_mpu = magData[0][1];
  mag_z_mpu = magData[0][2];
  mag_x_corr_mpu = (mag_x_mpu - mag_offset_mpu[0]) * mag_scale_mpu[0];
  mag_y_corr_mpu = (mag_y_mpu - mag_offset_mpu[1]) * mag_scale_mpu[1];
  mag_z_corr_mpu = (mag_z_mpu - mag_offset_mpu[2]) * mag_scale_mpu[2];

  float heading_z_mpu = atan2(mag_y_corr_mpu, mag_x_corr_mpu);
  float heading_x_mpu = atan2(mag_z_corr_mpu, mag_y_corr_mpu);
  float heading_y_mpu = atan2(mag_x_corr_mpu, mag_z_corr_mpu);

  raw_acc_x = (1000.0f * accData[0][0]);
  raw_acc_y = (1000.0f * accData[0][1]);
  raw_acc_z = (1000.0f * accData[0][2]);

  roll_angle = (atan2(raw_acc_y, raw_acc_x));
  flip_angle = (atan2(raw_acc_x, raw_acc_z));

  if (roll_angle < 0)
  {
    roll_angle_off = roll_angle + PI;
  }
  else
  {
    roll_angle_off = roll_angle - PI;
  }
  roll_angle_off_prev = roll_angle_off;


  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  PNI MAG calculations
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  mag_x_corr = (mag_x - mag_offset[0]) * mag_scale[0];
  mag_y_corr = (mag_y - mag_offset[1]) * mag_scale[1];
  mag_z_corr = (mag_z - mag_offset[2]) * mag_scale[2];
  float heading_z = (atan2(mag_y_corr, mag_x_corr));
  float heading_x = (atan2(mag_z_corr, mag_y_corr));
  float heading_y = (atan2(mag_x_corr, mag_z_corr));

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  THRUST calculations A-mode
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  float T0 = thr_CRSF_01;
  float k = 0.3;

  float T_amp = k * (sqrt(sq(rol_CRSF) + sq(pit_CRSF)));
  float theta_c = (atan2(rol_CRSF, pit_CRSF));

  if (CRSF_ch[6] == 1000)
  {
    theta_off = - PI / 4 - PI;
  }
  if (CRSF_ch[6] == 2000)
  {
    theta_c = (atan2(rol_CRSF, -pit_CRSF));
    theta_off = - PI / 4;
  }

  if (sin(heading_z + theta_c + theta_off) > 0.02)
  {
    thrust_m1 = T0 + T_amp;
  }
  else
  {
    thrust_m1 = T0 - T_amp;
  }
  thrust_m1 = min(max(thrust_m1, 0.0), (1.0 + k * (sqrt(2.0))));
  thrust_m1 = map(thrust_m1, 0.0, (1.0 + k * (sqrt(2.0))), 1500.00, 2000.00);
  thrust_m2 = map(T0, 0.0, 1.0, 1500.00, 2000.00);

  float torque_control = map(CRSF_ch[11], 1000, 2000, -300.00, 300.00) / 100.00;
  thrust_m2 = thrust_m2 - torque_control;

  thrust_m1 = min(max(thrust_m1, 1500.0), 2000.0);
  thrust_m2 = min(max(thrust_m2, 1500.0), 2000.0);

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //  THRUST calculations T-mode
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  thr_rol_mode = map(CRSF_ch[3], 998, 2011, -100.00, 100.00) / 100.00;
  des_stop_ang = 0.02;

  float des_stop_omega = 0.00;
  float kp_stop = 0.4;
  float kd_stop = 0.2;
  float kp_head = 200;
  float ki_head = 5;

  if (rol_CRSF > 0.1 || rol_CRSF < -0.1)  {
    kp_stop = 0.6;
    kd_stop = 0.2;
  }

  currentTmodeHeading = heading_y;
  tmodeHeading = tmodeHeading - rol_CRSF * 0.001;

  if (tmodeHeading < -PI) tmodeHeading = 2 * PI + tmodeHeading;
  else if (tmodeHeading > PI) tmodeHeading = -2 * PI + tmodeHeading;

  desiredTmodeHeading = tmodeHeading;
  float eHeading = currentTmodeHeading - desiredTmodeHeading;
  sum_eHeading = 0;
  int sum_eHeading_i = 0;
  while (sum_eHeading_i < sum_eHeading_size) {
    sum_eHeading += sum_eHeading_vector[sum_eHeading_i];
    sum_eHeading_i++;
  }
  sum_eHeading -= sum_eHeading_vector[sum_eHeading_firstIndex];
  sum_eHeading_i = 0;
  while (sum_eHeading_i < sum_eHeading_size - 1) {
    sum_eHeading_vector[sum_eHeading_i] = sum_eHeading_vector[sum_eHeading_i + 1];
    sum_eHeading_i++;
  }
  sum_eHeading_vector[sum_eHeading_size - 1] = eHeading;
  sum_eHeading += eHeading;
  sum_eHeading = min(max(sum_eHeading, -20.0), 20.0);

  float thr_head = (kp_head * eHeading) + (ki_head * sum_eHeading);

  if (thr_rol_mode > -0.02 && thr_rol_mode < 0.02) thr_rol_mode = 0.00;


  engageThr = thr_rol_mode;
  engageThrFlag = 0;
  if (engageThr != previousengageThr)
  {
    if (engageThr == 1 || engageThr == -1) startTime = millis();
  }
  else if (engageThr == previousengageThr && engageThr != 0 && counterActionPerformed < 1)
  {
    elapsedTime = millis() - startTime;
    if (elapsedTime >= actionDelay && elapsedTime <= (actionDelay + actionInterval))
    {
      if (engageThr == 1) engageThrFlag = 1;
      else if (engageThr == -1) engageThrFlag = -1;
    }
    else if (elapsedTime > (actionDelay + actionInterval))
    {
      counterActionPerformed = counterActionPerformed + 1;
    }
  }
  if (thr_rol_mode == 0)counterActionPerformed = 0;
  previousengageThr = engageThr;


  engageFlip = map(CRSF_ch[10], 1000, 2000, -100.00, 100.00) / 100.00;
  engageFlipFlag = 0;
  if (engageFlip != previousengageFlip)
  {
    if (engageFlip == 1 || engageFlip == -1)
    {
      startTimeFlip = millis();
    }
  }
  else if (engageFlip == previousengageFlip && engageFlip != 0 && counterFlipPerformed < 1)
  {
    elapsedTimeFlip = millis() - startTimeFlip;
    if (elapsedTimeFlip >= FlipDelay && elapsedTimeFlip <= (FlipDelay + FlipInterval))
    {
      if (engageFlip == 1) engageFlipFlag = 1;
      else if (engageFlip == -1) engageFlipFlag = -1;
    }
    else if (elapsedTimeFlip > (FlipDelay + FlipInterval))
    {
      counterFlipPerformed = counterFlipPerformed + 1;
    }
  }
  if (engageFlip == 0)counterFlipPerformed = 0;
  previousengageFlip = engageFlip;


  engageFlip2 = map(CRSF_ch[9], 1000, 2000, -100.00, 100.00) / 100.00;
  engageFlip2Flag = 0;
  if (engageFlip2 != previousengageFlip2)
  {
    if (engageFlip2 == 1 || engageFlip2 == -1) startTimeFlip2 = millis();
  }
  else if (engageFlip2 == previousengageFlip2 && engageFlip2 != 0 && counterFlip2Performed < 1)
  {
    elapsedTimeFlip2 = millis() - startTimeFlip2;
    if (elapsedTimeFlip2 >= Flip2Delay && elapsedTimeFlip2 <= (Flip2Delay + Flip2Interval))
    {
      if (engageFlip2 == 1) engageFlip2Flag = 1;
      else if (engageFlip2 == -1) engageFlip2Flag = -1;
    }
    else if (elapsedTimeFlip2 > (Flip2Delay + Flip2Interval))
    {
      counterFlip2Performed = counterFlip2Performed + 1;
    }
  }
  if (engageFlip2 == 0)counterFlip2Performed = 0;
  previousengageFlip2 = engageFlip2;


  if (thr_rol_mode > 0) //////// FORWARD ROLL
  {
    if ((roll_angle_off > -PI) && (roll_angle_off < 0))
    {
      thrust_roll1 = -thr_rol_mode * sin(roll_angle_off);
      thrust_roll2 = -thr_rol_mode * sin(roll_angle_off);
    }
    else
    {
      thrust_roll1 = 0;
      thrust_roll2 = 0;
    }
    if ((roll_angle_off > 2.4) || (roll_angle_off < -2.4))
    {
      thr_head = -thr_head;
      thr_head = min(max(thr_head, -90.0), 90.0);
    }
    else
    {
      thr_head = 0;
    }
  }
  else if (thr_rol_mode < 0) ////////// BACKWARD ROLL
  {
    if ((roll_angle_off > 0) && (roll_angle_off < PI))
    {
      thrust_roll1 = thr_rol_mode * sin(roll_angle_off);
      thrust_roll2 = thr_rol_mode * sin(roll_angle_off);
      thr_head = 0;
    }
    else
    {
      thrust_roll1 = 0;
      thrust_roll2 = 0;
      thr_head = 0;
    }
  }
  else //////////// STOP PENDULUM MOTION
  {
    if (CRSF_ch[7] > 1400)
    {
      if ((flip_angle > -1.85) && (flip_angle < -1.15) && (roll_angle_off > -2.5) && (roll_angle_off < 2.5))
      {
        thrust_roll1 = kp_stop * (des_stop_ang - roll_angle_off) + kd_stop * (des_stop_omega - (gyroData[0][2] * PI / 180));
        thrust_roll2 = kp_stop * (des_stop_ang - roll_angle_off) + kd_stop * (des_stop_omega - (gyroData[0][2] * PI / 180));
      }
      else
      {
        thrust_roll1 = 0;
        thrust_roll2 = 0;
      }
      if (engageFlipFlag == 1 || engageFlipFlag == -1)
      {
        thrust_roll1 = 0;
        thrust_roll2 = 0;
      }
    }
    else
    {
      thrust_roll1 = 0;
      thrust_roll2 = 0;
    }
  }

  thrust_roll1 = map(thrust_roll1, -1.5, 1.5, 1300.00, 1700.00);
  thrust_roll2 = map(thrust_roll2, -1.5, 1.5, 1300.00, 1700.00);

  if (engageThrFlag == 1)
  {
    thrust_roll1 = 1500 + (1600 - 1500) * (elapsedTime - 1000) / 1000 * (elapsedTime - 1000) / 1000;
    thrust_roll2 = 1500 + (1600 - 1500) * (elapsedTime - 1000) / 1000 * (elapsedTime - 1000) / 1000;
  }

  if (engageThrFlag == -1)
  {
    thrust_roll1 = 1500 - (1500 - 1350) * (elapsedTime - 1000) / 1000 * (elapsedTime - 1000) / 1000;
    thrust_roll2 = 1500 - (1500 - 1350) * (elapsedTime - 1000) / 1000 * (elapsedTime - 1000) / 1000;
  }

  // T to A flip
  new_flip_max_thr = 1650;
  new_flip_min_thr = 1150;

  if (engageFlipFlag == 1)
  {
    if (state == 1)
    {
      thrust_roll1 = 1575;
      thrust_roll2 = 1500;
      if (roll_angle_off > 1.1) state = 2;
      startdelayforflipTime = millis();
    }
    if (state == 2)
    {
      elapsedTimeforflipdelay = millis() - startdelayforflipTime;
      thrust_roll1 = new_flip_max_thr;
      thrust_roll2 = new_flip_min_thr;
      if (elapsedTimeforflipdelay >= FlipStateDelay)
      {
        if (flip_angle > 2.4 || flip_angle < -2.4) state = 3;
      }
    }
    if (state == 3)
    {
      thrust_roll1 = 1575;
      thrust_roll2 = 1575;
    }
  }

  if (arm < 1500)
  {
    state = 1;
    elapsedTimeforflipdelay = 0;
  }

  // A to T flip

  float a2t_flip_max_thr = 1800;
  float a2t_flip_min_thr = 1200;

  if (engageFlip2Flag == 1)
  {
    if (flip_angle < -2.0) {
      thrust_roll2 = 1500 + (a2t_flip_max_thr - 1500) * (elapsedTimeFlip2 - 1000) / 1000;
      thrust_roll1 = 1500 - (1500 - a2t_flip_min_thr) * (elapsedTimeFlip2 - 1000) / 1000;
    }
    else
    {
      thrust_roll1 = 1500 + (a2t_flip_max_thr - 1500) * (elapsedTimeFlip2 - 1000) / 1000;
      thrust_roll2 = 1500 - (1500 - a2t_flip_min_thr) * (elapsedTimeFlip2 - 1000) / 1000;
    }
  }

  m1_rol_mode = thrust_roll1 - thr_head;
  m2_rol_mode = thrust_roll2 + thr_head;

  m1_rol_mode = min(max(m1_rol_mode, 1000.0), 2000.0);
  m2_rol_mode = min(max(m2_rol_mode, 1000.0), 2000.0);

  if ((m1_rol_mode > 1490) && (m1_rol_mode < 1510)) m1_rol_mode = 1500;
  if ((m2_rol_mode > 1490) && (m2_rol_mode < 1510)) m2_rol_mode = 1500;

  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // COMMAND TO ACTUATORS
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (arm > 1500) {
    if (CRSF_ch[7] < 1600)
    {
      if (CRSF_ch[8] < 1500)        // MONOCOPTER MODE
      {
        thrust_m1 = min(max(thrust_m1, 1500.0), 2000.0);
        thrust_m2 = min(max(thrust_m2, 1500.0), 2000.0);
        if (CRSF_ch[6] == 1000)
        {
          M_1.writeMicroseconds(thrust_m1);
          M_2.writeMicroseconds(thrust_m2);
        }
        else if (CRSF_ch[6] == 1500)
        {
          M_1.writeMicroseconds(1500);
          M_2.writeMicroseconds(thrust_m1);
        }
        else if (CRSF_ch[6] == 2000)
        {
          M_1.writeMicroseconds(thrust_m2);
          M_2.writeMicroseconds(thrust_m1);
        }
        m1_rol_mode = 1500;
        m1_rol_mode = 1500;
      }
      else                           // ROLLING MODE
      {
        if (CRSF_ch[11] > 1500)
        {
          m1_rol_mode = 1650;
          m2_rol_mode = 1650;
        }
        if (CRSF_ch[12] > 1500)
        {
          m1_rol_mode = 1350;
          m2_rol_mode = 1350;
        }
        M_1.writeMicroseconds(m1_rol_mode);
        M_2.writeMicroseconds(m2_rol_mode);
      }
    }
    else
    {
      M_1.writeMicroseconds(val1);
      M_2.writeMicroseconds(val2);
    }


    if (sd_counter >= arr_size) {
      write_data();
      sd_counter = 0;
    }

    if (sd_counter < arr_size) {
      sd_buff1[sd_counter] = (millis() / 1000.0);
    }
    sd_counter = sd_counter + 1;

  }
  else {
    M_1.writeMicroseconds(1500);
    M_2.writeMicroseconds(1500);
  }

}

void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while (Serial2.available() > 0 && newData == false) {
    rc = Serial2.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    }
    else {
      receivedChars[ndx] = '\0'; // terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    token = strtok(receivedChars, s);
    con_com[0] = atof(token);
    for (int i = 1; i <= (num_ch - 1); i++)
    {
      token = strtok(NULL, s);
      con_com[i] = atof(token);
    }
    newData = false;
  }
}

void mag_calib_mpu()
{
  mag_temp_mpu[0] = mag_x_mpu;
  mag_temp_mpu[1] = mag_y_mpu;
  mag_temp_mpu[2] = mag_z_mpu;
  for (int jj = 0; jj < 3; jj++) {
    if (mag_temp_mpu[jj] > mag_max_mpu[jj]) mag_max_mpu[jj] = mag_temp_mpu[jj];
    if (mag_temp_mpu[jj] < mag_min_mpu[jj]) mag_min_mpu[jj] = mag_temp_mpu[jj];
  }
  // Hard iron correction
  mag_offset_mpu[0]  = (mag_max_mpu[0] + mag_min_mpu[0]) / 2;
  mag_offset_mpu[1]  = (mag_max_mpu[1] + mag_min_mpu[1]) / 2;
  mag_offset_mpu[2]  = (mag_max_mpu[2] + mag_min_mpu[2]) / 2;
  // Get soft iron correction estimate
  avg_delta_mpu[0]  = (mag_max_mpu[0] - mag_min_mpu[0]) / 2;
  avg_delta_mpu[1]  = (mag_max_mpu[1] - mag_min_mpu[1]) / 2;
  avg_delta_mpu[2]  = (mag_max_mpu[2] - mag_min_mpu[2]) / 2;
  avg_delta_xyz_mpu = (avg_delta_mpu[0] + avg_delta_mpu[1] + avg_delta_mpu[2]) / 3;
  mag_scale_mpu[0] = avg_delta_xyz_mpu / avg_delta_mpu[0];
  mag_scale_mpu[1] = avg_delta_xyz_mpu / avg_delta_mpu[1];
  mag_scale_mpu[2] = avg_delta_xyz_mpu / avg_delta_mpu[2];
}

void write_data() {
  myFile = SD.open(filename, FILE_WRITE);
  for (int i = 0; i < arr_size; i++) {
    myFile.print(sd_buff1[i], "%04d"); myFile.print(",");
    myFile.print(sd_buff2[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff3[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff4[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff5[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff6[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff7[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff8[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff9[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff10[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff11[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff12[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff13[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff14[i], "%02d"); myFile.print(",");
    myFile.print(sd_buff15[i], "%02d"); myFile.println();
  }
  myFile.close();

}

void ProcEventStatus(I2Cdev* i2c_BUS, uint8_t sensorNUM)
{
  uint8_t temp[1];

  // Read algorithm status and event status
  i2c_BUS->readBytes(MAX32660_SLV_ADDR, COMBO_DRDY_STAT, 1, temp);
  eventStatus[sensorNUM] = temp[0];

  // Decode the event status to determine what data is ready and set the appropriate DRDY fags
  if (eventStatus[sensorNUM] & 0x01) Gyro_flag[sensorNUM] = 1;
  if (eventStatus[sensorNUM] & 0x02) Acc_flag[sensorNUM]  = 1;
  if (eventStatus[sensorNUM] & 0x04) Mag_flag[sensorNUM]  = 1;
  if (eventStatus[sensorNUM] & 0x08) Baro_flag[sensorNUM] = 1;
  if (eventStatus[sensorNUM] & 0x10) Quat_flag[sensorNUM] = 1;
}

void FetchUSFSMAX_Data(USFSMAX* usfsmax, IMU* IMu, uint8_t sensorNUM)
{
  uint8_t call_sensors = eventStatus[sensorNUM] & 0x0F;

  Acq_time = 0;
  Begin = micros();

  // Optimize the I2C read function with respect to whatever sensor data is ready
  switch (call_sensors)
  {
    case 0x01:
      usfsmax->GyroAccel_getADC();
      break;
    case 0x02:
      usfsmax->GyroAccel_getADC();
      break;
    case 0x03:
      usfsmax->GyroAccel_getADC();
      break;
    case 0x07:
      usfsmax->GyroAccelMagBaro_getADC();
      break;
    case 0x0B:
      usfsmax->GyroAccelMagBaro_getADC();
      break;
    case 0x0F:
      usfsmax->GyroAccelMagBaro_getADC();
      break;
    case 0x0C:
      usfsmax->MagBaro_getADC();
      break;
    case 0x04:
      usfsmax->MAG_getADC();
      break;
    case 0x08:
      usfsmax->BARO_getADC();
      break;
    default:
      break;
  };
  Acq_time += micros() - Begin;

  if (Mag_flag[sensorNUM])
  {
    if (ScaledSensorDataFlag)                                                                                        // Calibration data is applied in the coprocessor; just scale
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        magData[sensorNUM][i] = ((float)magADC[sensorNUM][i]) * UT_per_Count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(ellipsoid_magcal[sensorNUM], magADC[sensorNUM], UT_per_Count, mag_calData[sensorNUM]);
      sensor_cal.apply_adv_calibration(final_magcal[sensorNUM], mag_calData[sensorNUM], 1.0f, sensor_point);
      MAG_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Mag_flag[sensorNUM] = 0;
  }
  if (Acc_flag[sensorNUM])
  {
    if (ScaledSensorDataFlag)                                                                                        // Calibration data is applied in the coprocessor; just scale
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        accData[sensorNUM][i] = ((float)accADC[sensorNUM][i]) * g_per_count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(accelcal[sensorNUM], accADC[sensorNUM], g_per_count, sensor_point);
      ACC_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }
    Acc_flag[sensorNUM] = 0;
  }
  if (Gyro_flag[sensorNUM] == 1)
  {
    if (ScaledSensorDataFlag)                                                                                        // Calibration data is applied in the coprocessor; just scale
    {
      for (uint8_t i = 0; i < 3; i++)
      {
        gyroData[sensorNUM][i] = ((float)gyroADC[sensorNUM][i]) * dps_per_count;
      }
    } else                                                                                                           // Calibration data applied locally
    {
      sensor_cal.apply_adv_calibration(gyrocal[sensorNUM], gyroADC[sensorNUM], dps_per_count, sensor_point);
      GYRO_ORIENTATION(sensor_point[0], sensor_point[1], sensor_point[2]);
    }

    // Call alternative (Madgwick or Mahony) IMU fusion filter
    IMu->compute_Alternate_IMU();
    Gyro_flag[sensorNUM] = 0;
  }
  if (Quat_flag[sensorNUM] == 1)
  {
    IMu->computeIMU();
    Quat_flag[sensorNUM] = 0;
  }
}

// Host DRDY interrupt handler
void DRDY_handler_0()
{
  data_ready[0] = 1;
}

// Serial interface handler
void SerialInterface_handler()
{
  serial_input = 0;
  if (Serial.available()) serial_input = Serial.read();
  if (serial_input == 49) {
    calibratingG[0] = 1; // Type "1" to initiate USFSMAX_0 Gyro Cal
  }
  if (serial_input == 50)                                                                                            // Type "2" to list current sensor calibration data
  {
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                   // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_full_gyrocal();
    delay(100);
    USFSMAX_0.Retreive_full_accelcal();
    delay(100);
    USFSMAX_0.Retreive_ellip_magcal();
    delay(100);
    USFSMAX_0.Retreive_final_magcal();
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);

    // Print the calibration results
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(gyrocal[0].V[0], 4);
    Serial.println(gyrocal[0].V[1], 4);
    Serial.println(gyrocal[0].V[2], 4); Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(gyrocal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[0][2], 4);
    Serial.print(gyrocal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[1][2], 4);
    Serial.print(gyrocal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(gyrocal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(gyrocal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(accelcal[0].V[0], 4);
    Serial.println(accelcal[0].V[1], 4);
    Serial.println(accelcal[0].V[2], 4); Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(accelcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[0][2], 4);
    Serial.print(accelcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[1][2], 4);
    Serial.print(accelcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(accelcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(accelcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(ellipsoid_magcal[0].V[0], 4);
    Serial.println(ellipsoid_magcal[0].V[1], 4);
    Serial.println(ellipsoid_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(ellipsoid_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[0][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[1][2], 4);
    Serial.print(ellipsoid_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(ellipsoid_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(ellipsoid_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(final_magcal[0].V[0], 4);
    Serial.println(final_magcal[0].V[1], 4);
    Serial.println(final_magcal[0].V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(final_magcal[0].invW[0][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[0][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[0][2], 4);
    Serial.print(final_magcal[0].invW[1][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[1][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[1][2], 4);
    Serial.print(final_magcal[0].invW[2][0], 4); Serial.print(",");
    Serial.print(final_magcal[0].invW[2][1], 4); Serial.print(",");
    Serial.println(final_magcal[0].invW[2][2], 4);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  if (serial_input == 51) {
    USFSMAX_0.Reset_DHI(); // Type "3" to reset the DHI corrector
  }
  if (serial_input == 52)                                                                                            // Type "4" to list copro config
  {
    SENSOR_0_WIRE_INSTANCE.setClock(100000);                                                                         // Set I2C clock to 100kHz to read the calibration data from the MAX32660
    delay(100);
    USFSMAX_0.Retreive_cfg();                                                                                        // Get the current USFSMAX config
    delay(100);
    SENSOR_0_WIRE_INSTANCE.setClock(I2C_CLOCK);                                                                      // Resume high-speed I2C operation
    delay(100);
    Serial.print("Accel scale = "); Serial.println(Cfg[1].Ascale);
    Serial.print("Accel ODR   = "); Serial.println(Cfg[1].AODR);
    Serial.print("Accel LPF   = "); Serial.println(Cfg[1].Alpf);
    Serial.print("Accel HPF   = "); Serial.println(Cfg[1].Ahpf);
    Serial.print("Gyro scale  = "); Serial.println(Cfg[1].Gscale);
    Serial.print("Gyro ODR    = "); Serial.println(Cfg[1].GODR);
    Serial.print("Gyro LPF    = "); Serial.println(Cfg[1].Glpf);
    Serial.print("Gyro HPF    = "); Serial.println(Cfg[1].Ghpf);
    Serial.print("Quat div    = "); Serial.println(Cfg[1].quat_div);
    Serial.print("Mag scale   = "); Serial.println(Cfg[1].Mscale);
    Serial.print("Mag ODR     = "); Serial.println(Cfg[1].MODR);
    Serial.print("Mag LPF     = "); Serial.println(Cfg[1].Mlpf);
    Serial.print("Mag HPF     = "); Serial.println(Cfg[1].Mhpf);
    Serial.print("Baro scale  = "); Serial.println(Cfg[1].Pscale);
    Serial.print("Baro ODR    = "); Serial.println(Cfg[1].PODR);
    Serial.print("Baro LPF    = "); Serial.println(Cfg[1].Plpf);
    Serial.print("Baro HPF    = "); Serial.println(Cfg[1].Phpf);
    Serial.print("AUX_1 scale = "); Serial.println(Cfg[1].AUX1scale);
    Serial.print("AUX_1 ODR   = "); Serial.println(Cfg[1].AUX1ODR);
    Serial.print("AUX_1 LPF   = "); Serial.println(Cfg[1].AUX1lpf);
    Serial.print("AUX_1 HPF   = "); Serial.println(Cfg[1].AUX1hpf);
    Serial.print("AUX_2 scale = "); Serial.println(Cfg[1].AUX2scale);
    Serial.print("AUX_2 ODR   = "); Serial.println(Cfg[1].AUX2ODR);
    Serial.print("AUX_2 LPF   = "); Serial.println(Cfg[1].AUX2lpf);
    Serial.print("AUX_2 HPF   = "); Serial.println(Cfg[1].AUX2hpf);
    Serial.print("AUX_3 scale = "); Serial.println(Cfg[1].AUX3scale);
    Serial.print("AUX_3 ODR   = "); Serial.println(Cfg[1].AUX3ODR);
    Serial.print("AUX_3 LPF   = "); Serial.println(Cfg[1].AUX3lpf);
    Serial.print("AUX_3 HPF   = "); Serial.println(Cfg[1].AUX3hpf);
    Serial.print("Vert FS     = "); Serial.println(Cfg[1].m_v, 5);
    Serial.print("Horiz FS    = "); Serial.println(Cfg[1].m_h, 5);
    Serial.print("Declination = "); Serial.println(Cfg[1].m_dec, 5);
    Serial.print("Cal points  = "); Serial.println(Cfg[1].cal_points);
    Serial.println(""); Serial.println("");
    sensor_cal.sendOneToProceed();                                                                                   // Halt the serial monitor to let the user read the calibration data
  }
  serial_input = 0;

  // Hotkey messaging
  Serial.println("'1' Gyro Cal");
  Serial.println("'2' List Cal Data");
  Serial.println("'3' Reset DHI Corrector");
  Serial.println("'4' List USFSMAX Config");
  Serial.println("");
}
