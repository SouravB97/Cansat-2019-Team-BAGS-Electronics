/*
This section deals with power down states of the mcu, along with functions to wake it up.
*/

/*
	Interrupt cannot be set in RISING mode because that requires 
	GCLK to be enabled for the EIC.
	Hence, we have to set the interrupt to HIGH and detach it 
	after servicing the interrupt.
	Hence, the interrupt can be used to wakeup the device exactly once,
	which is also coincidentally what we want.
	
	attachInterrupt function configures the EIC to be enabled during sleep mdode.
*/
void initLowPower(){
	pinMode(POW_WAKEUP, INPUT);	//Pin to wake with battery
	attachInterrupt(POW_WAKEUP, onSwitchOFF, LOW);
}

void sleep(){
	rtc.standbyMode();
	// SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	// __DSB();
	// __WFI();	
}

/*
	This function is called when the device is woken up.
*/
void onSwitchON(){
	detachInterrupt(POW_WAKEUP);
	NVIC_SystemReset();      // processor software reset
	//digitalWrite(PIN_LED_TXL, !digitalRead(PIN_LED_TXL));
	// #ifdef SER_DEBUG
		// USBDevice.init();
		// USBDevice.attach();
		// Serial.begin(115200);
		// //while(!Serial);
	// #endif
	attachInterrupt(POW_WAKEUP, onSwitchOFF, LOW);
	//NVIC_SystemReset();      // processor software reset
}

/*
*/	
void onSwitchOFF(){
	detachInterrupt(POW_WAKEUP);
	digitalWrite(DBG_LED, LOW);
	//digitalWrite(PIN_LED_TXL, !digitalRead(PIN_LED_TXL));
	attachInterrupt(POW_WAKEUP, onSwitchON, HIGH);
	//sleep();
}

