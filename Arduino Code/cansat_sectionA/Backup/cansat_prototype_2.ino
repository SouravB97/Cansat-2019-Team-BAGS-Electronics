#include "pindef.h"

//Serial1 is defined by default
Uart Serial2 (&sercom4, UART1_RX, UART1_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
TwoWire Wire1(&sercom2, SDA, SCL);
//SPI is defined by default.
//SPIClass SPI1(&sercom1, MISO, SCK, MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

Adafruit_BMP280 bmp;
MPU6050 mpu6050(Wire1);
TinyGPSPlus gps;
RTCZero rtc;
packet dataPacket(TEAM_ID);

File packetFile;		//file handle for packet.csv
File missionLog;		//file handle for missin.log


void setup(){
	#ifdef SER_DEBUG
		Serial.begin(115200);
    //USBDevice.attach();
		while(!Serial)
  #endif
	gps_uart.begin(9600);
  //zigbee.begin(9600);
  Wire1.begin();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);    
	
  initSerial();
	rtc.begin();
	initLowPower();
  initBatteryVoltage();
	initHall();
	initBmp();
	initSD();
	initMpu();
	
	//CONFIGURE TIMERS AND IRQ
	tc5Configure(1000);	//interrupt every second
	
//	Serial.println("Going to sleep");
//	//sleep();
//	#ifdef SER_DEBUG
//		while(!Serial);
//  #endif	
//	Serial.println("Woke up.");
//	while(1){
//    delay(3000);
//    digitalWrite(LED_BUILTIN, LOW);
//    Serial.println("In while..");
//	}
	
	//resetMissionTime();			//Do this when prompted by ground station
  
	readFile("packets.csv");
	#ifdef LOG_MISSION
		logEvent("Altitude reset to 0.");
		//delay(2000);
		logEvent("Taking Off");
		//delay(5000);
		logEvent("Couldn't find bmp.");
	#endif
	readFile(&missionLog);

	int i = 0;
	uint32_t timer = millis();
	while(i<10){
		if(true /*millis() - timer > 1000 */){
			Serial.println("Press any key to sample packet.");
			while(!Serial.available());
			while(Serial.available())
				Serial.read();
			i++;
			
			dataPacket.mission_time = getMissionTime();
			dataPacket.packet_count = i;
			dataPacket.altitude = bmp.readAltitude(1013.25);
			dataPacket.pressure = bmp.readPressure();
			dataPacket.temperature = bmp.readTemperature();
			dataPacket.voltage = getBatteryVoltage();
			updateGps();
			//pitch and roll are being updated in ISR
			dataPacket.blade_spin_rate = getBladeSpinRate();
			dataPacket.software_state = idle;
			
			dataPacket.display();
			savePacket(&dataPacket);
			Serial.println(dataPacket.toString());
			//transmitPacketString(&dataPacket);
			
			timer = millis();
		}
	}
	
	readFile(&packetFile);
}

void loop(){
	
}

void TC5_Handler (void) {
  //YOUR CODE HERE
	
	//digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	
  // END OF YOUR CODE
  TC5->COUNT32.INTFLAG.bit.MC0 = 1; //Interrupt serviced, so clear the flag.
}