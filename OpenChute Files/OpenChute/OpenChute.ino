#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Servo.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
float reading;
float prevreading = 0;
float total;
float difference;
int ascending = 0;

//flight state will represent which portion of flight OpenChute thinks it is in
// 0 - prelaunch  (if altitude readings are relatively constant and no other flight state is reached)
// 1 - ascent     (if altitude is increasing) 
// 2 - descent    (if altitude was increasing and now is decreasing)
// 3 - landed     (if altitude was decreasing and now is relatively constant)
int flightstate = 0;
int counter;
int cutoff;     //used to detect ascent and descent

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

  difference = prevreading - reading;
  prevreading = reading;

  //Allow noise between -0.5 and 0.5 m difference
  if(difference > 0.5){
    ascending = 1;
    cutoff += 1;
  }else if(difference < -0.5){
    ascending = -1;
  }else{
    ascending = 0;
    cutoff -= 1;
  }

  if(cutoff > 7){
    flightstate = 1;
  }else if(cutoff < 7){
    flightstate = 2;
  }
  Serial.print("Altitude: "); 
  Serial.print(reading);
  Serial.print(" m");
  Serial.print("\t\tFlight State: ");
  Serial.print(flightstate);
  Serial.print("\t\tAscending?");
  Serial.print(ascending);
  Serial.print("\t\tDifference: ");
  Serial.print(difference);
  Serial.println("");
  delay(100);

  if(reading > 150){
    myservo.write(120);
  }else{
    myservo.write(0);
  }

}
