/*
As the the container is opened, The light reaching
the cansat is increased, and hence LDR value
is increased. We can use this to determine if the cansat
has been released from the container or not.
*/

const int  MARGIN = 500;	//Outside brightness = ambient brightness + margin

int ambient_brightness;

void initLDR(){
	pinMode(LDR, INPUT);
  analogReadResolution(12); // Set analog input resolution to max, 12-bits
	//callibrateAmbientBrightness();	
}
/*
Call this function when cansat is inside container
*/	
void callibrateAmbientBrightness(){
	#ifdef SER_DEBUG
		Serial.println("Calibrating Brightness...");
	#endif	
	#ifdef LOG_MISSION
		logEvent("Calibrating Brightness...");
	#endif
	uint32_t brightness_sum = 0;
	for(int i = 0; i < 5; i++){
		int brightness = analogRead(LDR); 
		#ifdef SER_DEBUG
			Serial.println("Brightness : " + String(brightness));
		#endif
		brightness_sum += brightness;
		delay(100);
	}
	ambient_brightness = brightness_sum / 5;;
	enableMPUInterrupts();
	#ifdef SER_DEBUG
		Serial.println("Ambient brightness : " + String(ambient_brightness));
	#endif
	#ifdef LOG_MISSION
		logEvent("Ambient brightness : " + String(ambient_brightness));
	#endif		
}

bool droppedFromRocket(){
	return (analogRead(LDR) > ambient_brightness + MARGIN);
}

int getAmbientBrightness(){
	return ambient_brightness;
}