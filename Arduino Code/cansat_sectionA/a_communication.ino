/*
The exact same function logic is present in both these functions.
Ideally, I should have not copy pasted, but in one function
stream is of type Serial_* and in the other it is Uart*.
They both must somehow extend Stream somehow, but I am 
not sure how to implement them.

EDIT: Implemented
*/

/*
	This is compatible with calling Serial, SerialUSB, zigbee
	for USB/Radio console.
*/	
void console(Stream &s){
	Stream* stream = &s;
	int command;
	if(stream->available()){
		command = stream->parseInt();
		#ifdef LOG_MISSION
			logEvent("Received Command: "+ String(command));	
		#endif
		switch(command){
			case 0:
				stream->println("Toggling telemetry.");
				telemetry = !telemetry;
				break;
			case 1:
				stream->println("Resetting mission time.");
				resetMissionTime();
				break;
			case 2:
				stream->println("Recallibrating MPU");
				// disableMPUInterrupts();
				// mpu6050.calcGyroOffsets(true);
				// enableMPUInterrupts();
				callibrateMPU();	// may take upto 10 seconds while pitch and roll stabilise.
				break;
			case 3:
				stream->println("Recallibrating BMP");
				callibrateBMP();	//blocks for 500ms
				break;
			case 4:{
				
				stream->println("Enter value to be transmitted over NRF");
				while(!stream->available());
				int16_t data = (int16_t)stream->parseInt();
				//have to break this into two uint8_ts and fill it into a buffer
				uint8_t data_buf[2];
				data_buf[1] = (uint8_t)data;
				data_buf[0] = (uint8_t)(data >> 8);
				
				nrf24.send(data_buf, sizeof(data_buf));
				nrf24.waitPacketSent();
				
				//decode logic
				int16_t decoded_data = (int16_t)((data_buf[0] << 8) | (data_buf[1]));
				stream->println("Transmitted : " + String(decoded_data));
			}
				break;
			case 5: //Toggle Buzzer
				digitalWrite(BUZZER, !digitalRead(BUZZER));
				break;
			case 6:	//display LDR value
				stream->println("LDR : " + String(analogRead(LDR)));
				break;
			case 7:	//read packet file
				readFile(&packetFile, *stream);
				break;
			case 8:	//read mission file
				readFile(&missionLog, *stream);
				break;
			case 9:	//Print format string
				stream->println(dataPacket.format);
				break;
			case 10:{	//set date and time
				uint8_t hour = prompt("Hour", 0, 23, *stream); // Get the hour
				uint8_t minute = prompt("Minute", 0, 59, *stream); // Get the minute
				uint8_t second = prompt("Second", 0, 59, *stream); // Get the second
				uint8_t day = prompt("Day", 0, 31, *stream); // Get the day
				uint8_t month = prompt("Month", 0, 12, *stream); // Get the month
				uint8_t year = prompt("Year (YY)", 0, 99, *stream); // Get the year
				rtc.setTime(hour, minute, second); // Then set the time
				rtc.setDate(day, month, year); // And the date
				stream->println("Date and time set.");
			}
				break;
			case 11:	//get current date and time
				stream->println(getRTCDateTime());
				break;
			case 12:	//get Current time
				stream->println(getRTCTime());
				break;
			case 13:	//get Current date
				stream->println(getRTCDate());
				break;
			case 14:{	//set alarm for a specific time
				stream->println("Set alarm for a specific date-time");
				uint8_t hour = prompt("Hour", 0, 23, *stream); // Get the hour
				uint8_t minute = prompt("Minute", 0, 59, *stream); // Get the minute
				uint8_t second = prompt("Second", 0, 59, *stream); // Get the second
				uint8_t day = prompt("Day", 0, 31, *stream); // Get the day
				uint8_t month = prompt("Month", 0, 12, *stream); // Get the month
				uint8_t year = prompt("Year (YY)", 0, 99, *stream); // Get the year
				rtc.setAlarmTime(hour, minute, second); // Then set the time
				rtc.setAlarmDate(day, month, year); // And the date			
				rtc.enableAlarm(rtc.MATCH_HHMMSS);
				rtc.attachInterrupt(rtcAlarmMatch);
				stream->println("Alarm set.");
			}
				break;
			case 15:{ //set alarm for n seconds into the future
				stream->println("Set alarm for N seconds in the future. Enter N : ");
				while(!stream->available());
				int N = stream->parseInt();
				uint8_t extra_seconds = N % 60;
				N /= 60;
				uint8_t extra_minutes = N % 60;
				N /= 60;
				uint8_t extra_hours = N % 60;
				rtc.setAlarmTime(rtc.getHours() + extra_hours, rtc.getMinutes() + extra_minutes, rtc.getSeconds() + extra_seconds); // Then set the time
				rtc.setAlarmDate(rtc.getDay(), rtc.getMonth(), rtc.getYear()); // And the date			
				rtc.enableAlarm(rtc.MATCH_HHMMSS);
				rtc.attachInterrupt(rtcAlarmMatch);
				stream->println("Alarm set.");
			}
				break;
			case 16:{//Sleep
				stream->println("Puts cansat to SLEEP! Continue? (Y/N)");
				while(!stream->available());
				char c;
				while(stream->available())
					c = stream->read();
				if(c == 'Y' || c == 'y')
					sleep();
				else{
					stream->println("Cancelled.");
				}
			}
				break;
			case 17:	//toggle Debug LED
				digitalWrite(DBG_LED, !digitalRead(DBG_LED));
				break;
			case 18:{	//echo
				stream->println("Enter text to echo: ");
				while(!stream->available());
				String inp = stream->readString();
				stream->println(inp);
				}
				break;
			case 19:	//callibrate brightness
				stream->println("Calibrating Brightness...");
				callibrateAmbientBrightness();
				stream->println("Ambient brightness : " + String(getAmbientBrightness()));
				break;
			case 20:	//stop nrf transmission
				stream->println("NRF Transmision disabled.");
				clearNrfTxFlag();
				break;
			case 21: //enable NRF transmission
				stream->println("NRF Transmision enabled.");
				setNrfTxFlag();
				break;
			case 22:	//stop nrf_reception
				stream->println("NRF Reception disabled.");
				clearNrfRxFlag();
				break;
			case 23:	//enable nrf_reception
				stream->println("NRF Reception enabled.");
				setNrfRxFlag();
				break;
			case 24:	//reset samd21
				NVIC_SystemReset();      // processor software reset
				break;
			case 25:{	//change state
				stream->println("Enter new state. (0 - 3)");
				while(!stream->available());
				int data = stream->parseInt();
				if((data < 4) && (data >= 0)){
					state = data;
					stream->println("State changed to : " + String(state));
					#ifdef LOG_MISSION
						logEvent("State changed to : " + String(state));
					#endif	
				}	
				else
					stream->println("Invalid");
			}
				break;
			case 26:	//Calibrate cansat
				stream->println("Calibrating cansat...");
				calibCansat();
				stream->println("Cansat Calibrated.");
				break;
			case 27:{	//start video recording
				//start recording
				if(!video_recording){
					if(nrfTransmitWithAck(START_REC)){
						stream->println("Video recording started.");
						video_recording = true;
					}	
					else
						stream->println("Failed to send command.");					
				}				
			}
				break;
			case 28:{	//stop video recording
				if(video_recording){
					if(nrfTransmitWithAck(STOP_REC)){
						stream->println("Video recording stopped.");	
						video_recording = false;
					}	
					else
						stream->println("Failed to send command.");					
				}				
			}
				break;
			case 29:{	//start stabilising
				if(!secb_stabilising){
					if(nrfTransmitWithAck(START_STAB)){
							stream->println("Stabilisation started.");
							secb_stabilising = true;
					}
					else
						stream->println("Failed to send command.");
				}				
			}
				break;
			case 30:{	//stop stabilising
				if(secb_stabilising)
					if(nrfTransmitWithAck(STOP_STAB)){
						stream->println("Stabilisation stopped.");
						secb_stabilising = false;
						break;
					}
					else
						stream->println("Failed to send command.");
			}
				break;			
			case 31:{	//start direction transmission				
				if(nrfTransmitWithAck(START_DIR_TX))
					stream->println("Section B direction transmission started.");
				else
					stream->println("Failed to send command.");
			}
				break;
			case 32:{	//stop direction transmission	
				if(nrfTransmitWithAck(STOP_DIR_TX))
					stream->println("Section B direction transmission halted.");
				else
					stream->println("Failed to send command.");
			}
				break;				
			default:;
		}
	}
}
