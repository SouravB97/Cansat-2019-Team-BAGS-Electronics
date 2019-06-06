
/*
SamplePeriod: The amount of time between each interrupt.
Unit: ms
Range:	0ms to 1398ms
For 0ms, actual interrupt period becomes 0.04266

Function used for telemetry at 1Hz
*/
void tc5Configure(int samplePeriod){
	// Enable GCLK for TCC2 and TC5 (timer counter input clock)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	//reset tc5
	TC5->COUNT32.CTRLA.reg = TC_CTRLA_SWRST;
	while (TC5->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);	//tcIsSyncing
	while (TC5->COUNT32.CTRLA.bit.SWRST);
	
	// Set Timer counter Mode to 32 bits
	TC5->COUNT32.CTRLA.reg |= TC_CTRLA_MODE_COUNT32;
	// Set TC5 mode as match frequency
	TC5->COUNT32.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	//set prescaler and enable TC5
	TC5->COUNT32.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
	//set presync mode to Reload or reset the counter on next prescaler clock
	TC5->COUNT32.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC;
	//set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
	uint32_t counterVal = (uint32_t)((SystemCoreClock/1000000)*(samplePeriod*1000)/1024);
	TC5->COUNT32.CC[0].reg = counterVal ;	//SystemCoreClock = 48M
	//TC5->COUNT32.CC[0].reg = (uint32_t)(46882);		//value for 1 second period
	while (TC5->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);	//tcIsSyncing
	//SerialUSB.println("String TC5 counter value is : " + String(counterVal));
	// Configure interrupt request
	NVIC_DisableIRQ(TC5_IRQn);
	NVIC_ClearPendingIRQ(TC5_IRQn);
	NVIC_SetPriority(TC5_IRQn, 4);
	NVIC_EnableIRQ(TC5_IRQn);

	// Enable the TC5 interrupt request
	TC5->COUNT32.INTENSET.bit.MC0 = 1;
	while (TC5->COUNT32.STATUS.reg & TC_STATUS_SYNCBUSY);	//tcIsSyncing	
}
inline void disableTimerInterrupts(){
	TC5->COUNT32.INTENCLR.bit.MC0 = 1;
}	
inline void enableTimerInterrupts(){
	TC5->COUNT32.INTENSET.bit.MC0 = 1;
}

/*
SamplePeriod: The amount of time between each interrupt.
Unit: ms
Range:	0ms to 1398ms
For 0ms, actual interrupt period becomes 0.04266

Function used for MPU at 50Hz
*/
void tc4Configure(int samplePeriod){
	// Enable GCLK for TC4 and TC5 (timer counter input clock)
	GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	//reset TC4
	TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);	//tcIsSyncing
	while (TC4->COUNT16.CTRLA.bit.SWRST);
	
	// Set Timer counter Mode to 32 bits
	TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
	// Set TC4 mode as match frequency
	TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
	//set prescaler and enable TC4
	TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
	//set presync mode to Reload or reset the counter on next prescaler clock
	TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC;
	//set TC4 timer counter based off of the system clock and the user defined sample rate or waveform
	uint32_t counterVal = (uint16_t)((SystemCoreClock/1000000)*(samplePeriod*1000)/1024);
	TC4->COUNT16.CC[0].reg = counterVal ;	//SystemCoreClock = 48M
	//TC4->COUNT16.CC[0].reg = (uint32_t)(46882);		//value for 1 second period
	while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);	//tcIsSyncing
	//SerialUSB.println("String TC4 counter value is : " + String(counterVal));
	// Configure interrupt request
	NVIC_DisableIRQ(TC4_IRQn);
	NVIC_ClearPendingIRQ(TC4_IRQn);
	NVIC_SetPriority(TC4_IRQn, 3);
	NVIC_EnableIRQ(TC4_IRQn);

	// Enable the TC4 interrupt request
	TC4->COUNT16.INTENSET.bit.MC0 = 1;
	while (TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY);	//tcIsSyncing	
}

//TC3 is used in RH_ASK.cpp to drive NRF module