#include <Arduino.h>   // required before wiring_private.h
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <RTCZero.h>
#include <MadgwickAHRS.h>
#include <TinyGPS++.h>
#include <RH_NRF24.h>

#include "wiring_private.h" // pinPeripheral() function
#include "Adafruit_BMP280.h"
#include "MPU6050_tockn.h"
#include "packet.h"

#define UART0_RX 0	//defined by default Serial1
#define UART0_TX 1

#define UART1_RX A2 //Serial2
#define UART1_TX A1

#define SDA 4 //Wire1
#define SCL 3

#define MOSI 11 //default SPI
#define MISO 12
#define SCK  13

#define NRF_CE			8		//NRF only works with these pins.
#define NRF_CSN			10	//So they are hardwired. This is just for the programmer.

#define BAT_VOL			A3
#define HALL_INT		5
#define SD_SELECT		7
#define POW_WAKEUP	A3	//8
#define LDR 				A0
#define BUZZER			6
#define DBG_LED			9

#define Serial			SerialUSB
#define gps_uart		Serial1
#define zigbee			Serial2

#define LOG_MISSION			//enable this to log mission events to SD card
#define SER_DEBUG				//enable this to get debug info in serial monitor
#define TEAM_ID 1516		//BAGS

#define BOOT				0		//reset
#define INIT				1		//Sensors initialised and ready for telemetry
#define TELEMETRY		2		//performing telemetry and related computation
#define RECOVERY		3		//waiting for recovery from crew
