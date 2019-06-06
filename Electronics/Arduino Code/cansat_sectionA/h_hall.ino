#define NUM_MAGNETS 4
#define DEBOUNCE_DELAY_US 10000

volatile uint32_t hall_int_count = 0;	//variable to count pulses received
uint32_t counter_start_time;	

void initHall(){
  pinMode(HALL_INT, INPUT);
  digitalWrite(HALL_INT, HIGH);
	hall_int_count = 0;
	counter_start_time = millis();
  attachInterrupt(HALL_INT, hall_int_isr, FALLING);
}

void hall_int_isr(){
	delayMicroseconds(DEBOUNCE_DELAY_US);	//debounce
	if(digitalRead(HALL_INT) == LOW)
		hall_int_count++;
}

uint32_t getHallCount(){
	return hall_int_count;
}

void resetHall(){
	hall_int_count = 0;
	counter_start_time = millis();
}

/*
	Returns instantaneous blade spin rate in RPM
	Measures the pulses in 1 second period
	Blocking method
*/
int getBladeSpinRateInst(){
  uint32_t count_prev = hall_int_count;
  delay(1000);
  int rpm = ((hall_int_count - count_prev) * 60)/ NUM_MAGNETS;
	//int rpm = rps * 60;
  return rpm;
}

/*
	Returns instantaneous blade spin rate in RPM
	Measures the time taken to complete one complete rotation.
	Blocking method
*/
//int getBladeSpinRateInst(){
//	uint32_t time = 0;
//	resetHall();
//	while(digitalRead(HALL_INT) == HIGH);	//WAIT FOR INTERRUPT TO GO LOW
//	delay(DEBOUNCE_DELAY_US / 1000);
//	if(digitalRead(HALL_INT) == LOW){ //Interrupt has arrived
//		uint32_t tstart = millis();
//		for(int i = 0; i< NUM_MAGNETS; i++){
//			while(digitalRead(HALL_INT) == LOW)//wait for line to go high
//				delay(DEBOUNCE_DELAY_US / 1000);
//			while(digitalRead(HALL_INT) == HIGH)//wait for line to go low
//				delay(DEBOUNCE_DELAY_US / 1000);//debounce
//		}
//		time = millis() - tstart;
//	}
//	int rps = 1000/time;
//	initHall();
//	return rps;
//}

/*
	returns blade spin rate in RPM, averaged since last call to function.
*/
int getBladeSpinRate(){
	static uint32_t tprev = counter_start_time;
	static uint32_t count_prev = 0;
	uint32_t tcurrent = millis();
	int rpm = (((hall_int_count - count_prev) * 1000 * 60)/(NUM_MAGNETS * (tcurrent - tprev)));
	//int rpm = 60 * rps;
  tprev = tcurrent;
	count_prev = hall_int_count;
	return rpm;
} 
