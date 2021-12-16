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
#include <Servo.h>

//-------------------------------------------
// Constants and variable declarations
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

char logname[25];

File logfile;
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
  //set up servo pin
  myservo.attach(9);

  //try to begin serial communications
  Serial.begin(115200);

  //if successful, set serial flag to 1
  if(Serial){
    serialflag = 1;
  }

  //catch error for sensor
  //if error flash onboard LED fast
  if (!bmp.begin_I2C()) {
    Serial.println("ERROR: Could not find a valid BMP3 sensor");
    Sensorflag = 0;
    pinMode(8, OUTPUT);
    while(!Sensorflag){
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
      total += bmp.readAltitude(SEALEVELPRESSURE_HPA);
      Serial.print(".   ");
      Serial.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));
    }
    delay(1000);
  }
  Serial.println("");
  MSL = total/7;
  Serial.println(MSL);

  if (!SD.begin(cardSelect)) {
    Serial.println("ERROR: Could not find a valid SD Card");
    SDflag = 0;
    return;
  }

  strcpy(logname, "/data000.csv");
  for (uint8_t i = 0; i < 100; i++) {
    logname[5] = '0' + i/100;
    logname[6] = '0' + (i/10)%10;
    logname[7] = '0' + i%10;
    if (! SD.exists(logname)) {
      break;
    }
  }

  logfile = SD.open(logname, FILE_WRITE);
  if(!logfile) {
    return;
  }
  logfile.print("Time(s) \t Filtered Altitude MSL(m) \t Filtered Altitude AGL(m) \t Filtered Velocity(m/s) \t Filtered Acceleration(m/s/s) \t Altitude MSL(m)  \t Altitude AGL(m) \t Velocity(m/s) \t Acceleration (m/s/s) \t Chute Arm \t Chute Release");
  logfile.close();


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

  total = 0;
  //get reading from sensor
  for(counter = 0; counter < 5; counter++){
    total += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  reading = total/5;

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
  //*

  Serial.print("Altitude MSL: ");
  Serial.print(filteredpos);
  Serial.print("Altitude AGL: ");
  Serial.print(filteredpos - MSL);
  Serial.print("velocity: ");
  Serial.print(vel);
  Serial.print(" ");
  Serial.print("velocity_f: ");
  Serial.print(filteredvel);
  Serial.print("MSL: ");
  Serial.print(MSL);
  Serial.print(" Pressure: ");
  Serial.print(bmp.readPressure());
  Serial.println(" ");

  //logic to decide when to release parachute
  if((filteredpos - MSL) > releasealt + 20){
    chutearm = 1;
  }

  if(chutearm && (filteredpos - MSL) < releasealt){
    chuterelease = 1;
  }

  if(SDflag){
    //open file each time to write
    logfile = SD.open(logname, FILE_WRITE);
    if(!logfile) {
      return;
    }
  
      
    //logfile.print("Time(s) \t Filtered Altitude MSL(m) \t Filtered Altitude AGL(m) \t Filtered Velocity(m/s) \t Filtered Acceleration(m/s/s) \t Altitude MSL(m) \t Altitude AGL(m) \t Velocity(m/s) \t Acceleration (m/s/s)");
    
    logfile.print(t);
    logfile.print("\t");
    logfile.print(filteredpos);
    logfile.print("\t");
    logfile.print(filteredpos - MSL);
    logfile.print("\t");
    logfile.print(filteredvel);
    logfile.print("\t");
    logfile.print(filteredacc);
    logfile.print("\t");
    logfile.print(pos);
    logfile.print("\t");
    logfile.print(pos - MSL);
    logfile.print("\t");
    logfile.print(vel);
    logfile.print("\t");
    logfile.print(acc);
    logfile.print("\t");
    logfile.print(chutearm);
    logfile.print("\t");
    logfile.print(chuterelease);
    logfile.println("");
  
    logfile.close();
  }
  
  //*/
  delay(100);

  if(reading > 150){
    myservo.write(120);
  }else{
    myservo.write(0);
  }

}
