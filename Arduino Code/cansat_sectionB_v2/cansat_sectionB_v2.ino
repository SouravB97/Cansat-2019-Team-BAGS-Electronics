#include "pindef.h"

enum secB_command {
  START_REC, STOP_REC, START_STAB, STOP_STAB, START_DIR_TX, STOP_DIR_TX
};

TwoWire Wire1(&sercom2, SDA, SCL);

MPU9250 mpu9250(Wire1);
RH_NRF24 nrf24;
Servo myServo;

bool video_recording  = false;
bool secb_stabilising = false;
bool direction_tx = true;

const int transmit_wait_time = 500;	//transmit data every 500 ms

void setup(){
	initCansat();
  lineMapper(5 * KP, 88, 180  * KP, 180, 180  * KP, 0, 355  * KP, 88);
}

void loop(){
	
	if(secb_stabilising){
		uint32_t a = millis();
		int outputData = pid();
		myServo.write(outputData);
		uint32_t b = millis();
		#ifdef SER_DEBUG	
			Serial.println("Time Difference: " + String(b - a));
		#endif	
	}
		
	static uint32_t transmit_start_time = millis();
	if(millis() - transmit_start_time >= transmit_wait_time){
		if(direction_tx){
			int16_t direction = (int16_t)getDirectionFromNorth();
			if(direction_tx)	nrfTransmit(direction);
			//Serial.println("Transmitted : "+ String(direction));
			transmit_start_time = millis();
		}
	}
	
	// if(nrf24.available()){
		// secB_command command = (secB_command)nrfReceiveCommand();
		// decodeExecute(command);
	// }
	if(nrf24.available()) decodeExecute((secB_command)nrfReceiveCommand());	
}


inline void initCansat(){
	//pinMode(DBG_LED, OUTPUT);
	//digitalWrite(DBG_LED, HIGH);
	
	#ifdef SER_DEBUG
		Serial.begin(115200);
		//while(!Serial);
	#endif
		
	Wire1.begin();
	
	initSerial();
	initBatteryVoltage();
	initMpu();
	initNrf();
	myServo.attach(SERVO);
	initCamera();
		
	//digitalWrite(DBG_LED, LOW);	
}

void decodeExecute(secB_command command) {
  switch (command) {
    case START_REC:
			#ifdef SER_DEBUG
				 Serial.println("Start Rec");
			#endif
			startRecording();
      break;
    case STOP_REC:
			#ifdef SER_DEBUG
				 Serial.println("Stop Rec");
			#endif
			stopRecording();
      break;
    case START_STAB:
			#ifdef SER_DEBUG
				 Serial.println("Start stable");
			#endif
			secb_stabilising = true;
      break;
    case STOP_STAB:
			#ifdef SER_DEBUG
				 Serial.println("Stop stable");
			#endif
			secb_stabilising = false;	 
			myServo.write(90);
      break;
    case START_DIR_TX:
			#ifdef SER_DEBUG
				 Serial.println("Start direction tx");
			#endif
			direction_tx = true;
      break;
    case STOP_DIR_TX:
			#ifdef SER_DEBUG
				 Serial.println("Stop direction tx");
			#endif
			direction_tx = false;	 
      break;
    default:
      #ifdef SER_DEBUG
				 Serial.println("Invalid command");
			#endif
  }
}
