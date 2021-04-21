#include "HX711.h"
#define calibration_factor 256.00

// The sensor designed to send pulses to MISO-like at 80Hz
// When this signal occurs at the same time as the MOSI-like for data request,
// The data seen as a noise
// In order to avoid this, interrupt pins have been used
// DOUT from sensor was connected to both the interrupt pin and the MISO-like pin

int numPin = 5;
// Declaration for FSS sensors
// intPin for interrupt pins in Arduino mega
int intPin[5] = {2,3,18,19,20};
// MOSI-like pins not clk pins to be exact
int clkPin[5] = {4,6,8,10,12};
// MISO-like pins
int doutPin[5] = {5,7,9,11,13};
int32_t value[5] = {0};
// For offset setting purposes
int offsetCount[5] = {0};
double sumBuffer[5] = {0};
unsigned long time;

// 객체 선언
HX711 scale_0(doutPin[0], clkPin[0]);
HX711 scale_1(doutPin[1], clkPin[1]);
HX711 scale_2(doutPin[2], clkPin[2]);
HX711 scale_3(doutPin[3], clkPin[3]);
HX711 scale_4(doutPin[4], clkPin[4]);

// Functions to be initiated with the interrupts
void read_value_0(){
  // Putting in data into the buffer for offset setting
  if(offsetCount[0] < 20){
    sumBuffer[0] += scale_0.get_value();
    offsetCount[0] += 1;
  }
  else if(offsetCount[0] == 20){
    sumBuffer[0] = sumBuffer[0]/20;
    offsetCount[0] += 1;
  }
  // digitalRead == LOW for stable read of FALLING
  // Why sumBuffer has to be *2 has yet been identified
  else if(offsetCount[0] == 21 && digitalRead(doutPin[0]) == LOW){
    value[0] = scale_0.get_value() - 2*(sumBuffer[0]);
  }
}

void read_value_1(){
  if(offsetCount[1] < 20){
    sumBuffer[1] += scale_1.get_value();
    offsetCount[1] += 1;
  }
  else if(offsetCount[1] == 20){
    sumBuffer[1] = sumBuffer[1]/20;
    offsetCount[1] += 1;
  }
  else if(offsetCount[1] == 21 && digitalRead(doutPin[1]) == LOW){
    value[1] = scale_1.get_value() - 2*(sumBuffer[1]);
  }
}

void read_value_2(){
  if(offsetCount[2] < 20){
    sumBuffer[2] += scale_2.get_value();
    offsetCount[2] += 1;
  }
  else if(offsetCount[2] == 20){
    sumBuffer[2] = sumBuffer[2]/20;
    offsetCount[2] += 1;
  }
  else if(offsetCount[2] == 21 && digitalRead(doutPin[2]) == LOW){
    value[2] = scale_2.get_value() - 2*(sumBuffer[2]);
  }
}

void read_value_3(){
  if(offsetCount[3] < 20){
    sumBuffer[3] += scale_3.get_value();
    offsetCount[3] += 1;
  }
  else if(offsetCount[3] == 20){
    sumBuffer[3] = sumBuffer[3]/20;
    offsetCount[3] += 1;
  }
  else if(offsetCount[3] == 21 && digitalRead(doutPin[3]) == LOW){
    value[3] = scale_3.get_value() - 2*(sumBuffer[3]);
  }
}

void read_value_4(){
  if(offsetCount[4] < 20){
    sumBuffer[4] += scale_4.get_value();
    offsetCount[4] += 1;
  }
  else if(offsetCount[4] == 20){
    sumBuffer[4] = sumBuffer[4]/20;
    offsetCount[4] += 1;
  }
  else if(offsetCount[4] == 21 && digitalRead(doutPin[4]) == LOW){
    value[4] = scale_4.get_value() - 2*(sumBuffer[4]);
  }
}

void setup() {
  Serial.begin(115200);

  // Setup for the FSS sensors
  // Interrupt pins set mode as input over here where the MISO-like pins declared in .cpp
  for(int i = 0; i < numPin; i++){
    pinMode(intPin[i], INPUT);
  }

  // Setting scale and offset
  // Offset not used in .cpp but offset set instead in the interrupt functions
  scale_0.set_scale(calibration_factor);
  scale_0.tare();
  scale_1.set_scale(calibration_factor);
  scale_1.tare();
  scale_2.set_scale(calibration_factor);
  scale_2.tare();
  scale_3.set_scale(calibration_factor);
  scale_3.tare();
  scale_4.set_scale(calibration_factor);
  scale_4.tare();
  
  // Declaring the intpins as interrupt pins and declaring the initiating functions
  attachInterrupt(digitalPinToInterrupt(intPin[0]), read_value_0, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[1]), read_value_1, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[2]), read_value_2, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[3]), read_value_3, FALLING);
  attachInterrupt(digitalPinToInterrupt(intPin[4]), read_value_4, FALLING);

}

void loop() {
  // Setting a loop time
  time = millis();
  if(time-millis() > 20){    
    Serial.write((byte*)value,numPin*4);
    // Serial.write("\t") for Serial.print on monitor screen
    // Has to be "\n" for it to be read as single lines on the ROS segment
    Serial.write("\n");
  }

  // For testing purposes
  /*  
  for(int i=0; i<numPin; i++){
    Serial.print(value[i]);
    Serial.print("\t");
  }
  Serial.println(value[numPin-1]);
  */
}
