/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13. 
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead(). 
 
 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground
 
 * Note: because most Arduinos have a built-in LED attached 
 to pin 13 on the board, the LED is optional.
 
 
 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe
 
 This example code is in the public domain.
 
 http://arduino.cc/en/Tutorial/AnalogInput
 
 */

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 msg;
ros::Publisher chatter("load_cell", &msg);

int sensorPin = A1;    // select the input pin for the potentiometer

int sensorValue;  // variable to store the value coming from the sensor
int sensorSum = 0;
int sensorMean;
//unsigned long time;
int loopcount = 0;
//unsigned long endtime;
//unsigned long starttime;
//unsigned long a;

void setup() {
  nh.initNode();
  nh.advertise(chatter);
  analogReference(INTERNAL);
  Serial.begin(9600);
}

void loop() {
  analogReadResolution(12);
  sensorValue = analogRead(sensorPin);
  sensorSum = sensorSum+sensorValue;    
  loopcount=loopcount+1;
  if (loopcount%100 == 1) {
    sensorMean = sensorSum/100;
    msg.data = sensorMean;
    chatter.publish( &msg );
    nh.spinOnce();
    sensorSum = 0;
  }             
}
