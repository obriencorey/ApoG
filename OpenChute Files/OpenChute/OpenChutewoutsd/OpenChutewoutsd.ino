#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;

//variables to store sensor data
float reading;
float prevreading = 0;
float total;
float difference;


double pos;
double prevpos = 0;
double vel;
double prevvel = 0;
double acc;
double prevacc = 0;
double t;
double prevt = 0;
double dt;
float gain = 0.1;
double filteredpos;
double prevfilteredpos = 0;
double filteredvel;
double filteredprevvel = 0;
double filteredacc;
double filteredprevacc = 0;

int counter;

Servo myservo;

void setup() {
  //set up servo pin
  myservo.attach(9);
  Serial.begin(115200);
  while (!Serial);

  //catch errors
  if (!bmp.begin_I2C()) { 
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

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
  
  
  Serial.print("Y:");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print("\t\Y_f:");
  Serial.print(filteredpos);
  Serial.println(" ");

 
  /*
  
  Serial.print("velocity:");
  Serial.print(vel);
  Serial.print("   ");
  Serial.print("velocity_f:");
  Serial.print(filteredvel);
  Serial.println("   ");

  */
  delay(100);

}
