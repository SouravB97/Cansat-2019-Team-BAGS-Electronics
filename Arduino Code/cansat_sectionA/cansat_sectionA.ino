#include "pindef.h"

enum secB_command {
  START_REC, STOP_REC, START_STAB, STOP_STAB, START_DIR_TX, STOP_DIR_TX
};

//Serial1 is defined by default
Uart Serial2 (&sercom4, UART1_RX, UART1_TX, SERCOM_RX_PAD_1, UART_TX_PAD_0);
TwoWire Wire1(&sercom2, SDA, SCL);
//SPI is defined by default.
//SPIClass SPI1(&sercom1, MISO, SCK, MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

Adafruit_BMP280 bmp;
MPU6050 mpu6050(Wire1);
TinyGPSPlus gps;
RTCZero rtc;
RH_NRF24 nrf24;
packet dataPacket(TEAM_ID);

File packetFile;		//file handle for packet.csv
File missionLog;		//file handle for missin.log

volatile bool telemetry = false;	//flag which masks telemetry
volatile int packet_count = 0;
volatile uint8_t state = boot;

bool video_recording  = false;
bool secb_stabilising = false;
bool cansat_calibrated = false;

const int BAT_VALUE = 3300;

const int n_threads = 4;	//Number of timer threads in telemetry loop
const uint16_t thread_wait_time[] = {100, 250, 500, 1000};	//Time after which each thread gets called
uint32_t thread_start_time[n_threads] = {0};


void setup(){
	pinMode(POW_WAKEUP, INPUT);
	analogReadResolution(12);
	delay(300);
	while(isPowerOn()){//if(!digitalRead(POW_WAKEUP)){	//power button off or disconnect
		delay(300);	//or spend time sleeping
	}
	initCansat();
}

void loop(){
	switch(state){
		case BOOT:{
			//wait for power on
			//for some reason, calling analogRead on the SAMD21 doesnt let digitalRead be called. 
			//It always returns 0.
			if(isPowerOn()){//if(!digitalRead(POW_WAKEUP)){	//power button off or disconnect
				delay(300);	//or spend time sleeping
			}
			else{
				state = TELEMETRY;
				telemetry = true;
				enableTimerInterrupts();
				setNrfTxFlag();
				dataPacket.software_state = idle;
				#ifdef SER_DEBUG	
					Serial.println("State changed to : " + String(state));
				#endif
				#ifdef LOG_MISSION
					logEvent("State changed to : " + String(state));
				#endif	
			}
		}	break;
		
		case INIT:{		
			NVIC_SystemReset();      // processor software reset	
			telemetry = false;
			disableTimerInterrupts();
			disableMPUInterrupts();
			zigbee.flush();	//clear radio buffer
			zigbee.end();
			if(isPowerOn()){//if(!digitalRead(POW_WAKEUP)){	//power button off or disconnect
				delay(300);	//or spend time sleeping
			}
			else{
				//Power button pushed
				//delay(1000); //wait for everything to stabilise
				NVIC_SystemReset();      // processor software reset	
				//initCansat();
				//initVolatileSensors();	
				//calibCansat();
				state = TELEMETRY;
				telemetry = true;
				enableTimerInterrupts();
				setNrfTxFlag();
				enableMPUInterrupts();
				//dataPacket.software_state = idle;
				#ifdef SER_DEBUG	
					Serial.println("State changed to : " + String(state));
				#endif
				#ifdef LOG_MISSION
					logEvent("State changed to : " + String(state));
				#endif
			}	
		} break;
		
		case TELEMETRY:{
				
			if(isPowerOn()){//if(!digitalRead(POW_WAKEUP)){	//Check if battery disconnected
				state = INIT;
				break;
			}
			
			for(int thread = 0; thread < n_threads; thread++){
				if(millis() - thread_start_time[thread] >= thread_wait_time[thread]){
					switch(thread){
						case 0:	//Runs ever 100 ms
							
							//already called regardless of state
							// #ifdef SER_DEBUG
								// console(Serial);
							// #endif			
							// console(zigbee);
							
							break;
						case 1:	//Runs every 250 ms
							
							dataPacket.voltage = getBatteryVoltage();
							
							break;
						case 2:	//Runs every 500 ms
							
							dataPacket.mission_time = getMissionTime();
							dataPacket.roll = getRoll();
							dataPacket.pitch = getPitch();
							updateBMP();
							dataPacket.altitude = getAltitude();
							dataPacket.pressure = getPressure();
							dataPacket.temperature = getTemperature();
							
							dataPacket.software_state = processSoftwareState();				
							switch(dataPacket.software_state){
								case boot:
									break;
								case idle:
									break;
								case launch_detect:
									break;
								case deploy:
									break;
								case payload_release:{
									//start recording
									if(!video_recording){
										if(nrfTransmitWithAck(START_REC))	//if it times out, dont worry, we'll get em in the next iteration
											video_recording = true;
									}
									//start stabilising Section B
									if(!secb_stabilising){
										if(nrfTransmitWithAck(START_STAB))
											secb_stabilising = true;
									}
									//toggle TX LED
									//digitalWrite(PIN_LED_TXL, !digitalRead(PIN_LED_TXL));
								}
									break;
								case landed:{
						
									//stop camera recording
									for(int i = 0; i<5; ){	// give up trying to send command after 5 tries
										//stop camera recording
										if(video_recording)
											if(nrfTransmitWithAck(STOP_REC)){
												video_recording = false;
												break;
											}
											else{
												i++;
												delay(300);
											}
									}
									//stop camera stabilisation
									for(int i = 0; i<5; ){	// give up trying to send command after 5 tries
										//stop sec_b stabilising
										if(secb_stabilising)
											if(nrfTransmitWithAck(STOP_STAB)){
												secb_stabilising = false;
												break;
											}
											else{
												i++;
												delay(300);
											}
									}
									
									delay(1500);	//wait for last couple of packets to transmit this state.
										
									telemetry = false;
									disableTimerInterrupts();
									state = RECOVERY;	//telemetry done
									#ifdef SER_DEBUG	
										Serial.println("State changed to : " + String(state));
									#endif
									#ifdef LOG_MISSION
										logEvent("State changed to : " + String(state));
									#endif						
								}
									break;
								default:;
							}
												
							
							break;
						case 3:{	//Runs every 1000 ms
							
							updateGps();
							dataPacket.blade_spin_rate = getBladeSpinRate();	//call it every second
							if (telemetry) safe_println(&packetFile, dataPacket.toString()/* + String(getYaw())*/);//write packet to SD card.

						}
							break;
						default:;
					}
					thread_start_time[thread] = millis();
				}
			}
			//Code runs as fast as possible.
			
			// get bonus direction from section B
			if(getNrfRxFlag())
				if (nrf24.available())
						dataPacket.bonus_direction = nrfReceiveData();
										
		}break;
		
		case RECOVERY:{
			//camera recording, stabilisation and telemetry already turned off.
			static int wait_time = 500;	//Buzzer blink toggle time
			if(isPowerOn()){	//Check if recovery crew switched cansat off
				delay(300);
				if(isPowerOn()){	//Check if recovery crew actually switched cansat off
					state = BOOT;
					cansat_calibrated = false;
					break;	//return to state BOOT
				}
			}
			static uint32_t start_time = 0;
			if((millis() - start_time) >= wait_time){
				digitalWrite(DBG_LED, !digitalRead(DBG_LED));
				digitalWrite(BUZZER, !digitalRead(BUZZER));
				
				//already called regardless of state
				// #ifdef SER_DEBUG
					// console(Serial);
				// #endif			
					// console(zigbee);
				start_time = millis();
			}
		}break;
		
		default:;
	}
	//print state every 2 seconds
	// static uint32_t loop_tstart = millis();
	// if(millis() - loop_tstart >= 1000){
		// Serial.print("State : " + String(state));
		// Serial.print("\tpow wakeup < 1000 ?: " + String((isPowerOn())));
		// Serial.println("\tpow wakeup val: " + String(analogRead(POW_WAKEUP)));
		// loop_tstart = millis();
	// }
	#ifdef SER_DEBUG
		console(Serial);
	#endif			
	console(zigbee);
	
}//</loop>

void TC5_Handler (void) {
	//YOUR CODE HERE	
	
	if(telemetry){
		//digitalWrite(DBG_LED, !digitalRead(DBG_LED));
		digitalWrite(DBG_LED, HIGH);
		dataPacket.packet_count = ++packet_count;
		String telemetry = dataPacket.toString();
		#ifdef SER_DEBUG
			//Serial.println(telemetry);
			dataPacket.display();
		#endif
		zigbee.println(telemetry);
		digitalWrite(DBG_LED, LOW);
	}
	// END OF YOUR CODE
	TC5->COUNT32.INTFLAG.bit.MC0 = 1; //Interrupt serviced, so clear the flag.
}


//Include all the code that is stored in the MCU memory
void initCansat(){
	telemetry = false;	//flag which masks telemetry
	packet_count = 0;		
	state = BOOT;
	
	rtc.begin();
	//initLowPower();
	
	#ifdef SER_DEBUG
		//USBDevice.init();
		//USBDevice.attach();
		Serial.begin(115200);
		//while(!Serial);
	#endif 
		
	gps_uart.begin(9600);
	zigbee.begin(9600);
	Wire1.begin();
	
	pinMode(POW_WAKEUP, INPUT);
	pinMode(DBG_LED, OUTPUT);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(BUZZER, OUTPUT);
	pinMode(PIN_LED_TXL, OUTPUT);
	
	digitalWrite(DBG_LED, LOW);
	digitalWrite(LED_BUILTIN, LOW);
	analogReadResolution(12);
	
	initSerial();
	initSD();	
	createFiles();
	initBatteryVoltage();
	initHall();
	initLDR();
	initMpu();
	initBmp();	
	initNrf();
	
	resetMissionTime();
	//CONFIGURE TIMERS AND IRQ
	tc5Configure(1000);	//interrupt every second
	disableTimerInterrupts();
				
	#ifdef SER_DEBUG
		Serial.println(F("Cansat initialisation complete."));
	#endif	
	#ifdef LOG_MISSION
		logEvent(F("Cansat initialisation complete."));
	#endif		
}

/*
	Include initialisation for volatile peripherals
*/	
/*
void initVolatileSensors(){
	
	// zigbee.begin(9600);
	initSD();
	disableMPUInterrupts();
		initBmp();
		initMpu();
	enableMPUInterrupts();
	initNrf();
	#ifdef SER_DEBUG
		Serial.println(F("Volatile sensor initialisation complete."));
	#endif	
	#ifdef LOG_MISSION
		logEvent(F("Volatile sensor initialisation complete."));
	#endif		
}
*/
void initVolatileSensors(){
	telemetry = false;	//flag which masks telemetry
	//packet_count = 0;		
	//state = BOOT;
	
	//rtc.begin();
	//initLowPower();
	
	#ifdef SER_DEBUG
		//USBDevice.init();
		//USBDevice.attach();
		//Serial.begin(115200);
		//while(!Serial);
	#endif
		
	gps_uart.begin(9600);
	zigbee.begin(9600);
	//Wire1.begin();
	
	//pinMode(POW_WAKEUP, INPUT);
	//pinMode(DBG_LED, OUTPUT);
	//pinMode(LED_BUILTIN, OUTPUT);
	
	digitalWrite(DBG_LED, LOW);
	digitalWrite(LED_BUILTIN, LOW);
	//analogReadResolution(12);
	
	initSerial();
	initSD();	
	//createFiles();
	initBatteryVoltage();
	initHall();
	initLDR();
	initBmp();
	initMpu();
	initNrf();
	
	//resetMissionTime();
	//CONFIGURE TIMERS AND IRQ
	//tc5Configure(1000);	//interrupt every second
	disableTimerInterrupts();
				
	#ifdef SER_DEBUG
		Serial.println(F("Cansat initialisation complete."));
	#endif	
	#ifdef LOG_MISSION
		logEvent(F("Cansat initialisation complete."));
	#endif			
}


void calibCansat(){
	
	callibrateMPU();
	callibrateBMP();
	callibrateAmbientBrightness();
	resetHall();
	resetMissionTime();
	packet_count = 0;	//reset packet count
		
	cansat_calibrated = true;
	#ifdef SER_DEBUG
		Serial.println(F("Cansat calibration complete."));
	#endif
	#ifdef LOG_MISSION
		logEvent(F("Cansat calibration complete."));
	#endif
}

/*
	Processes sensor data and decides which op_state cansat is in. 
	*/
op_state processSoftwareState(){
	static op_state state = boot;
	
	//These flags ensure that a next state is attained only when its previous state was already attained.
	static bool idled = false;
	static bool launch_detected = false;
	static bool deployed = false;
	static bool payload_released = false;	
	
	//These flags stop the state from coming back to previous state.
	static bool idle_crossed = false;
	static bool launch_detect_crossed = false;
	static bool	deploy_crossed = false;
	static bool	payload_release_crossed = false;
	
		
	bool cansatReleased = droppedFromRocket();	//get light change data from LDR
	
	if(cansat_calibrated == false){
		state = boot;
	}
	else{
		//Rocket on the launchpad
		if(
			(dataPacket.altitude < 5) 				&&
			//(abs(dataPacket.roll)  < 1) 			&&
			//(abs(dataPacket.pitch)  < 1) 			&&
			//(mpu6050.accY < -0.5)							&&
			//(dataPacket.blade_spin_rate < 1)	&&
			(cansatReleased == false)				&&
			
			//no <prev_state reached> check as it is the first state
			(idle_crossed == false)
			){
				idled = true;
				state = idle;		
			}
			
		//rocket has started to ascend, till it reaches maximum point
		else if(
			(dataPacket.altitude > 1) 				&&
			//(abs(dataPacket.roll)  < 1) 			&&
			//(abs(dataPacket.pitch)  < 1) 			&&
			((mpu6050.accY < -0.7))							&&
			(dataPacket.blade_spin_rate < 1)	&&
			(cansatReleased == false)					&&
			
			(idled == true)										&&
			(launch_detect_crossed == false)
			){
				idle_crossed = true;
				launch_detected = true;
				state = launch_detect;
			}
				
		//cansat container has detached from the rocket and is falling with the parachute
		else if(
			(dataPacket.altitude > 450) 				&&
			//(abs(dataPacket.roll)  < 1) 			&&
			//(abs(dataPacket.pitch)  < 1) 			&&
			(mpu6050.accY < -1.2)							&&
			(dataPacket.blade_spin_rate < 1)	&&
			(cansatReleased == true)				&&
			
			(launch_detected == true)				&&
			(deploy_crossed == false)		
			){
				launch_detect_crossed = true;
				deployed = true;
				state = deploy;
			}
		
		//container has released the payload, and it is falling with autogyro
		else if(
			(dataPacket.altitude < 500) 			&&
			//(abs(dataPacket.roll)  < 1) 			&&
			//(abs(dataPacket.pitch)  < 1) 			&&
			//(mpu6050.accY < -0.5)							&&
			//(dataPacket.blade_spin_rate > 0)	&&
			(cansatReleased == true)						&&
				
			(deployed == true)									&&	
			(payload_release_crossed == false)
			){
				deploy_crossed = true;
				payload_released = true;
				state = payload_release;			
			}
			
		//payload has landed on the ground, and has stopped moving
		else if(
			(dataPacket.altitude < 50) 				&&
			//(abs(dataPacket.roll)  < 1) 			&&
			//(abs(dataPacket.pitch)  < 1) 			&&
			(abs(mpu6050.accY) < 1.2)							&&
			(dataPacket.blade_spin_rate < 1)	&&
			(cansatReleased == true)					&&
			
			(payload_released == true)
			//no landed_crossed check as it is the last state
			){
				payload_release_crossed = true;
				state = landed;	
			}
			
		//default: return whatver state was previously computed
	}
	
	//also, toggle tx_led every time state is changed.
	static op_state prev_state = boot;
	if(state != prev_state){
		digitalWrite(PIN_LED_TXL, !digitalRead(PIN_LED_TXL));
		prev_state = state;
	}

	return state;
}

