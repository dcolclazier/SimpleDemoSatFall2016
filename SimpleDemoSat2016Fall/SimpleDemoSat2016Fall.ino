/*
Name:		Sketch1.ino
Created:	11/4/2016 10:14:15 PM
Author:	david
*/

// the setup function runs once when you press reset or power the board
#include <MinimumSerial.h>
#include <Adafruit_LiquidCrystal.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_BNO055.h>
#include "SoftwareSerial.h"

Adafruit_BNO055 _bnoSensor;
Adafruit_BMP085_Unified _bmpSensor;

//BMP Pressure sensor variables
auto seaLevelPressure = 1014.8500326800001;
auto pressure = 0.0f;
auto altitude = 0;

//temp sensor variables
float altTemp = 0.0f;
float bnoTemp = 0.0f;

//bno calib variables
uint8_t calib_acc = 0;
uint8_t calib_sys = 0;
uint8_t calib_mag = 0;
uint8_t calib_gyr = 0;

//EDIT THESE FOR FINAL PIN PLACEMENT
auto customMagPin = A1;
auto unusedAnalogPin = A0;
auto expensiveMagRxPin = 10;
auto expensiveMagTxPin = 11;
auto openLogResetPin = 8;

//software serial used for expensive magnetometer
SoftwareSerial expensiveMagSerial(expensiveMagRxPin, expensiveMagTxPin);

//bno calibration string
String latestCalib = "S:0 M:0 A:0 G:0";

//init LCD
Adafruit_LiquidCrystal lcd(0); 

char buffer[50];
String fileName = "";

void setup() {

	//initialize serial lines
	Serial.begin(57600);
	expensiveMagSerial.begin(57600);
	
	//initialize lcd, print title
	lcd.begin(16,2);
	lcd.print("Payload McPayload Face!"); 
	
	//scroll title off-screen
	for (int i = 0; i < 23; i++)	{
		lcd.scrollDisplayLeft();
		delay(150);
	}
	lcd.clear();

	//start up the BNO055 breakout
	if (!_bnoSensor.begin())
	{
		lcd.print("No BNO detected..."); 
		while (1); //performs if check again.
	}

	//start up the BMP180..
	if (!_bmpSensor.begin())
	{
		lcd.print("No BMP detected..."); 
		while (1); //performs if check again.
	}
	delay(1000);
	_bnoSensor.setExtCrystalUse(true);
	
	//reset OpenLog
	digitalWrite(openLogResetPin, LOW);
	delay(100);
	digitalWrite(openLogResetPin, HIGH);

	//wait for OpenLog to respond with '<' to indicate it is alive and recording to a file
	while (1) {
		if (Serial.available())
			if (Serial.read() == '<') break;
	}

	//enter command mode on OpenLog
	Serial.write(26);
	Serial.write(26);
	Serial.write(26);

	//wait for OpenLog to respond with '>' to indicate we are in command mode
	while (1) {
		if (Serial.available())
			if (Serial.read() == '>') break;
	}

	//generate new random seed based on A0 pin noise
	randomSeed(analogRead(unusedAnalogPin));
	
	//generate fileName
	auto fileNum = random(999);
	fileName = "data";
	fileName += fileNum;
	fileName += ".csv";

	//send command to OpenLog to create file
	String command = "new " + fileName;
	command += " ";
	Serial.print(command);

	//display data file to lcd
	lcd.clear();
	String output = "Data file: " + fileName;
	lcd.print(output);

	//send command to enter append mode in our new file
	command = "append " + fileName;
	Serial.print(command);

	//wait for Openlog to indicate file is open and ready for writing
	while (1) {
		if (Serial.available())
			if (Serial.read() == '<') break;
	}

	//print out csv headers
	Serial.print("time,altitude,pressure,bnoMagX,bnoMagY,bnoMagZ,gyroX,gyroY,gyroZ,accelX,accelY,accelZ,eulerX,eulerY,eulerZ,gravX,gravY,gravZ,linearX,linearY,linearZ,custMagX,custMagY,custMagZ,expensiveMag,bnoTemp,bmpTemp,bnoCalib");

	//clear lcd buffer
	lcd.clear();
}

// the loop function runs over and over again until power down or reset
void loop() {

	//get altitude
	_bmpSensor.getPressure(&pressure);
	altitude = _bmpSensor.pressureToAltitude(seaLevelPressure, pressure / 100);

	//get bno data
	auto magVec = _bnoSensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
	auto gyroVec = _bnoSensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
	auto accelVec = _bnoSensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	auto euler = _bnoSensor.getVector(Adafruit_BNO055::VECTOR_EULER);
	auto gravVec = _bnoSensor.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
	auto linearVec = _bnoSensor.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

	//get chip temps
	bnoTemp = _bnoSensor.getTemp();
	_bmpSensor.getTemperature(&altTemp);

	//get current calibration 
	auto oldCalib = latestCalib;

	_bnoSensor.getCalibration(&calib_sys, &calib_gyr, &calib_acc, &calib_mag);

	latestCalib = "S:";
	latestCalib += calib_sys;
	latestCalib += " M:";
	latestCalib += calib_mag;
	latestCalib += " A:";
	latestCalib += calib_acc;
	latestCalib += " G:";
	latestCalib += calib_gyr;

	if(oldCalib != latestCalib) lcd.print(latestCalib);

	//read serial data from expensive mag
	auto customMagData = analogRead(customMagPin);

	//read analog data from custom mag (3 analog pins)
	auto expensiveMagData = expensiveMagSerial.read();
	
	//create log string
	//String command = "append " + fileName;
	//command += " ";
	String command = "";
	command += millis();
	command += ",";
	command += altitude;
	command += ",";
	command += pressure;
	command += ",";
	command += magVec.x();
	command += ",";
	command += magVec.y();
	command += ",";
	command += magVec.z();
	command += ",";
	command += gyroVec.x();
	command += ",";
	command += gyroVec.y();
	command += ",";
	command += gyroVec.z();
	command += ",";
	command += accelVec.x();
	command += ",";
	command += accelVec.y();
	command += ",";
	command += accelVec.z();
	command += ",";
	command += euler.x();
	command += ",";
	command += euler.y();
	command += ",";
	command += euler.z();
	command += ",";
	command += gravVec.x();
	command += ",";
	command += gravVec.y();
	command += ",";
	command += gravVec.z();
	command += ",";
	command += linearVec.x();
	command += ",";
	command += linearVec.y();
	command += ",";
	command += linearVec.z();
	command += ",";
	command += customMagData;
	command += ",";
	command += expensiveMagData;
	command += ",";
	command += bnoTemp;
	command += ",";
	command += altTemp;
	command += ",";
	command += latestCalib;

	Serial.println(command);

}
