/*
 * By Corey O'Brien
 * 
 * For use in the OpenChute project
 */

//-------------------------------------------
// Libraries
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_LSM6DS33.h>
#include <Servo.h>

//-------------------------------------------
// Constants and variable declarations
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

Adafruit_LSM6DS33 lsm6ds;
Adafruit_Sensor *lsm_temp, *lsm_accel, *lsm_gyro;

Servo myservo;

//variables to store sensor data
float reading;                          //stores unfiltered data
float prevreading = 0;                  //used for filtering
float total;                            //collects sums of readings (e.g. for calibration)
float difference;                       //change in readings
float gain = 0.1;                       //gain for low pass filter

double pos;
double prevpos = 0;
double vel;
double prevvel = 0;
double acc;
double prevacc = 0;
double t;
double prevt = 0;
double dt;
double filteredpos;
double prevfilteredpos = 0;
double filteredvel;
double filteredprevvel = 0;
double filteredacc;
double filteredprevacc = 0;

double a_xoffset = 0;
double a_yoffset = 0;
double a_zoffset = 0;

double w_xoffset = 0;
double w_yoffset = 0;
double w_zoffset = 0;

double MSL;                             //finds approximate altitude when turned on

int serialflag = 0;
int counter;
int SDflag = 1;
int Sensorflag = 1;
int cardSelect = 4;
int chutearm = 0;                     //if  above set altitude, arm chute
int chuterelease = 0;                 //once back below altitude, release
int releasealt = 150;                 //release altitude in meters

//-------------------------------------------
// Setup

void setup() {

  //try to begin serial communications
  Serial.begin(115200);
  Serial.println("STARTUP");
  
  //set up servo pin
  myservo.attach(9);

  pinMode(8, OUTPUT);

  //if successful, set serial flag to 1
  if(Serial){
    serialflag = 1; 
  }

  //catch error for sensor
  //if error flash onboard LED fast
  if (!bmp.begin_I2C()) {
    Sensorflag = 0;
    pinMode(8, OUTPUT);
    while(!Sensorflag){
      Serial.println("ERROR: Could not find a valid BMP3 sensor");
      digitalWrite(8,HIGH);
      delay(100);
      digitalWrite(8,LOW);
      delay(100);
    }
  }

  //catch more errors if there is an issue with the LSM6Ds
  if (!lsm6ds.begin_I2C()) {
    Sensorflag = 0;
    pinMode(8, OUTPUT);
    while(!Sensorflag){
      Serial.println("ERROR: Could not find a valid LSM6DS sensor");
      digitalWrite(8,HIGH);
      delay(100);
      digitalWrite(8,LOW);
      delay(100);
    }
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  //some lsm6 initialization
  lsm_temp = lsm6ds.getTemperatureSensor();
  lsm_temp->printSensorDetails();

  lsm_accel = lsm6ds.getAccelerometerSensor();
  lsm_accel->printSensorDetails();

  lsm_gyro = lsm6ds.getGyroSensor();
  lsm_gyro->printSensorDetails();

  
  //This part of the code determines what altitude we are at so we can get MSL and AGL readings
  total = bmp.readAltitude(SEALEVELPRESSURE_HPA);   //need first reading to flush out memory on chip????? IDK i got some weird results without this
  total = 0;
  delay(5000);
  Serial.begin(115200);
  Serial.print("Initializing");
  //get reading from sensor
  for(counter = 0; counter < 10; counter++){
    //ignore first 3 readings just incase, see note above for flushing out memory
    if(counter > 2){
      sensors_event_t accel;
      sensors_event_t gyro;
      lsm_accel->getEvent(&accel);
      lsm_gyro->getEvent(&gyro);
      
      total += bmp.readAltitude(SEALEVELPRESSURE_HPA);
      a_xoffset += accel.acceleration.x;
      a_yoffset += accel.acceleration.y;
      a_zoffset += accel.acceleration.z;

      w_xoffset += gyro.gyro.x;
      w_yoffset += gyro.gyro.y;
      w_zoffset += gyro.gyro.z;
    }
    delay(1000);
  }
  Serial.println("");
  MSL = total/7;

  a_xoffset = a_xoffset/7;
  a_yoffset = a_yoffset/7;
  a_zoffset = a_zoffset/7 + 9.08126;

  w_xoffset = w_xoffset/7;
  w_yoffset = w_yoffset/7;
  w_zoffset = w_zoffset/7;
  
  Serial.print("altitude above sea level is ");
  Serial.println(MSL);


  pinMode(8, OUTPUT);
  digitalWrite(8,HIGH);

  
}


//-------------------------------------------
// Loop

void loop() {
  //catch more errors
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  //code to get reading from pressure sensor
  total = 0;
  
  //get reading from sensor
  for(counter = 0; counter < 5; counter++){
    total += bmp.readAltitude(SEALEVELPRESSURE_HPA);

  }
  reading = total/5;

  //get readings from gyro, acc, and temp
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm_temp->getEvent(&temp);
  lsm_accel->getEvent(&accel);
  lsm_gyro->getEvent(&gyro);

  //Find delta t
  t = millis() / 1000.0;
  dt = (t - prevt);
  prevt = t;
  
  //Get position data
  pos = reading;
  
  //implement low pass filter
  filteredpos = (1 - gain) * prevfilteredpos + (pos + prevpos) * gain / 2;
  filteredvel = (filteredpos - prevfilteredpos) / dt;
  filteredacc = (filteredvel - filteredprevvel) / dt;
  vel = (pos - prevpos) / dt;
  acc = (vel - prevvel) / dt;

  //store current value as previous position
  prevpos = pos;
  prevvel = vel;
  prevacc = acc;
  prevfilteredpos = filteredpos;
  filteredprevvel = filteredvel;
  

  difference = prevreading - reading;
  prevreading = reading;


  /*
  Serial.print("Pos: "); 
  Serial.print(pos);
  Serial.print(" m \t");
  Serial.print("Pos_f: "); 
  Serial.print(filteredpos);
  Serial.print(" m \t");
  Serial.print("dPos: "); 
  Serial.print(filteredpos - pos);
  Serial.print(" m \t");
  Serial.print("Vel: "); 
  Serial.print(vel);
  Serial.print(" m/s\t");
  Serial.print("Acc: "); 
  Serial.print(acc);
  Serial.print(" m/s^2\t");
  Serial.print("dt: "); 
  Serial.print(dt);
  Serial.print(" s\t");
  Serial.print("t: "); 
  Serial.print(t);
  Serial.print(" s\t");
  Serial.println("");
  */

  //for serial plotting
  /*
  
  Serial.print("position:");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print("filteredposition:");
  Serial.print(filteredpos);
  Serial.println(" ");

  */


  Serial.print("H(MSL): ");
  Serial.print(filteredpos);
  Serial.print(" m H(AGL): ");
  Serial.print(filteredpos - MSL);
  Serial.print(" m P: ");
  Serial.print(bmp.readPressure());
  Serial.print(" Pa a_x: ");
  Serial.print(accel.acceleration.x - a_xoffset);
  Serial.print(" m/s2 a_y: ");
  Serial.print(accel.acceleration.y - a_yoffset);
  Serial.print(" m/s2 a_z: ");
  Serial.print(accel.acceleration.z - a_zoffset);
  Serial.print(" m/s2 w_x: ");
  Serial.print(gyro.gyro.x - w_xoffset);
  Serial.print(" rad/s2 w_y: ");
  Serial.print(gyro.gyro.y - w_yoffset);
  Serial.print(" rad/s2 w_z: ");
  Serial.print(gyro.gyro.z - w_zoffset);
  Serial.println(" rad/s2 ");
  Serial.println(" ");


  //logic to decide when to release parachute
  if((filteredpos - MSL) > releasealt + 20){
    chutearm = 1;
  }

  if(chutearm && (filteredpos - MSL) < releasealt){
    chuterelease = 1;
  }

  
  //*/
  delay(100);

}
