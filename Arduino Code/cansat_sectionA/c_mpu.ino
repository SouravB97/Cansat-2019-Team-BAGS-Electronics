#define MPU_READ_FREQ 50	//Hz

Madgwick filter;

volatile float ax, ay, az;
volatile float gx, gy, gz;
volatile float pitch, roll, yaw;
float pitchOffset = 0, rollOffset = 0;


void initMpu() {
	mpu6050.begin();
	mpu6050.setGyroOffsets(-2.24, -0.16, -0.07);
	filter.begin(MPU_READ_FREQ);
	//CONFIGURE TIMERS AND IRQ
	tc4Configure(1000/MPU_READ_FREQ);	
	//callibrateMPU();	// may take upto 10 seconds while pitch and roll stabilise.
}

inline float getRoll(){
	return roll;
}
inline float getPitch(){
	return pitch;
}
inline float getYaw(){
	return yaw;
}

/*
	Waits for pitch and roll to stabilise	and computes offsets.

	Ideally this function should compute a matrix to multiply and a matrix to add to readings obtained from filter, but i'm just going to compute offsets.	
*/	
void callibrateMPU(){
	#ifdef LOG_MISSION
		logEvent("Calculating gyroscope offsets.");
	#endif		
	disableMPUInterrupts();
		#ifdef SER_DEBUG
			mpu6050.calcGyroOffsets(true);
		#else
			mpu6050.calcGyroOffsets(false);
		#endif
	enableMPUInterrupts();
	#ifdef LOG_MISSION
		logEvent("X: " + String(mpu6050.gyroXoffset) + "\tY: " + String(mpu6050.gyroYoffset)  + "\tZ: " + String(mpu6050.gyroZoffset));
	#endif		
	#ifdef SER_DEBUG
		Serial.println(F("Computing orientation offsets. Do not move Cansat."));
	#endif
	#ifdef LOG_MISSION
		logEvent(F("Computing orientation offsets."));
	#endif		
	float der_noise = 1;	//degrees
	float prev_value[] = {0,0};
	float cur_value[2];
	float derivative[2];
	do{
		prev_value[0] = roll;
		prev_value[1] = pitch;
		delay(300);
		cur_value[0] = roll;
		cur_value[1] = pitch;
		for(int i = 0; i< 2; i++)
			derivative[i] = (prev_value[i] - cur_value[i]);
		#ifdef SER_DEBUG
			Serial.print(F("Derivatives : "));
			Serial.print(derivative[0]);
			Serial.print(F("/"));
			Serial.println(derivative[1]);
		#endif	
	}
	while( (abs(derivative[0]) > der_noise ) || (abs(derivative[1]) > der_noise ));
	
	rollOffset = roll;
	pitchOffset = pitch;
	
	#ifdef SER_DEBUG
		Serial.println("Roll Offset\t: " + String(rollOffset));
		Serial.println("Pitch Offset\t: " + String(pitchOffset));
	#endif	
	#ifdef LOG_MISSION
		logEvent("Roll Offset\t: " + String(rollOffset) + "\tPitch Offset\t: " + String(pitchOffset));
	#endif			
}

inline void readAndCompute(){
	// read raw data from mpu6050
	mpu6050.update();
	ax = mpu6050.accX;
	ay = mpu6050.accY;
	az = mpu6050.accZ;
	gx = mpu6050.gyroX;
	gy = mpu6050.gyroY;
	gz = mpu6050.gyroZ;

	// update the filter, which computes orientation
	filter.updateIMU(gx, gy, gz, ax, ay, az);	
}


/*Function gets called by the interrupt at <MPU_READ_FREQ> Hertz
	The code takes 700uS to execute
	Thre mpu readings take around 10s at the current sampling to stabilise.
*/
void TC4_Handler (void) {
  //YOUR CODE HERE

	readAndCompute();
	
	// These pitch, roll and yaw values are according to the cansat's orientation.
	roll  	= -filter.getPitch() - rollOffset;// + 180;
	pitch 	= filter.getRoll() + 90 - pitchOffset;	//mpu is kept vertical
	yaw = filter.getYaw();	//getHeading()?
		
		
  // END OF YOUR CODE
  TC4->COUNT16.INTFLAG.bit.MC0 = 1; //Interrupt serviced, so clear the flag.
}

inline void disableMPUInterrupts(){
	TC4->COUNT16.INTENCLR.bit.MC0 = 1;
}	
inline void enableMPUInterrupts(){
	TC4->COUNT16.INTENSET.bit.MC0 = 1;
}