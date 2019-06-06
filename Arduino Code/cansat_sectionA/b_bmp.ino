float base_altitude = 388;	//elevation of stephenville

volatile float altitude, pressure, temperature;

void initBmp(){
  if (!bmp.begin()) {
		#ifdef SER_DEBUG
			Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
		#endif
		#ifdef LOG_MISSION
			logEvent(F("Could not find a valid BMP280 sensor."));
		#endif
	}
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */	

	//callibrateBMP();
}

void callibrateBMP(){
	#ifdef SER_DEBUG
		Serial.println("Calibrating Altitude...");
	#endif	
	#ifdef LOG_MISSION
		logEvent("Calibrating Altitude...");
	#endif			
	disableMPUInterrupts();
	float altitude_sum = 0;
	for(int i = 0; i < 5; i++){
		float altitude = bmp.readAltitude(1013.25); 
		#ifdef SER_DEBUG
			Serial.println("Current Altitude : " + String(altitude));
		#endif
		altitude_sum += altitude;
		delay(100);
	}
	base_altitude = altitude_sum / 5;
	enableMPUInterrupts();
	#ifdef SER_DEBUG
		Serial.println("Base Altitude : " + String(base_altitude));
	#endif
	#ifdef LOG_MISSION
		logEvent("Base Altitude : " + String(base_altitude));
	#endif			
}

void updateBMP(){
	disableMPUInterrupts();//Have to do this to free up the I2C bus
		altitude = bmp.readAltitude(1013.25) - base_altitude;
		pressure = bmp.readPressure();
		temperature = bmp.readTemperature();
	enableMPUInterrupts();	
}

inline float getAltitude(){
	return altitude;
}
inline float getPressure(){
	return pressure;
}
inline float getTemperature(){
	return temperature;
}


//void setup() {
//  Serial.begin(115200);
//	Serial.println("Begin");
//  Wire1.begin();
//  //I2C
//  pinPeripheral(SDA, PIO_SERCOM_ALT);
//  pinPeripheral(SCL, PIO_SERCOM_ALT);
//  
//  Serial.println(F("BMP280 test"));
//
//  if (!bmp.begin()) {
//    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
//    while (1);
//  }
//
//  /* Default settings from datasheet. */
//  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
//                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
//                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
//                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
//                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
//}
//
//void loop() {
//    Serial.print(F("Temperature = "));
//    Serial.print(bmp.readTemperature());
//    Serial.println(" *C");
//
//    Serial.print(F("Pressure = "));
//    Serial.print(bmp.readPressure());
//    Serial.println(" Pa");
//
//    Serial.print(F("Approx altitude = "));
//    Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//    Serial.println(" m");
//
//    Serial.println();
//    delay(2000);
//}

