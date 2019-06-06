#include <Arduino.h>   // required before wiring_private.h
#include <Wire.h>
#include <SPI.h>
#include <MadgwickAHRS.h>
#include <RH_NRF24.h>
#include <Servo.h>

#include "wiring_private.h" // pinPeripheral() function
#include "MPU9250_tockn.h"

#define SDA 4 //Wire1
#define SCL 3

#define KP 5
#define KI 0
#define KD 0

#define MOSI 11 //default SPI
#define MISO 12
#define SCK  13

#define NRF_CE			8		//NRF only works with these pins.
#define NRF_CSN			10	//So they are hardwired. This is just for the programmer.

#define BAT_VOL			A0
#define CAM_PWR			5
#define CAM_MODE		6
#define SERVO				7
#define DBG_LED			9

#define Serial		SerialUSB

#define SER_DEBUG		//enable this to get debug info in serial monitor
