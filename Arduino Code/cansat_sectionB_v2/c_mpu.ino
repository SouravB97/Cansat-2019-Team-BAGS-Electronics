#define MPU_READ_FREQ 50  //Hz
#define TOLERANCE 2
Madgwick filter;

volatile float ax, ay, az;
volatile float gx, gy, gz;
volatile float mx, my, mz;
volatile float pitch, roll, yaw;
float pitchOffset = 0, rollOffset = 0, m1 = 0, c1 = 0, m2 = 0, c2 = 0;
float setPoint, outputData, prevError, yaw_val_for_pid, prevYaw;
uint32_t timeNow, timePast = 0;


void initMpu() {
  mpu9250.begin();
  #ifdef SER_DEBUG
    mpu9250.calcGyroOffsets(true);
  #else
    mpu9250.calcGyroOffsets(false);
  #endif
  filter.begin(MPU_READ_FREQ);
  //CONFIGURE TIMERS AND IRQ
  tc5Configure(1000/MPU_READ_FREQ); 
  
  callibrateMPU();  // may take upto 10 seconds while pitch and roll stabilise.
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
int getDirectionFromNorth(){
  //return (int)((180 / PI) * atan(my / mx));
	return getYaw();
}

/*
  Waits for pitch and roll to stabilise and computes offsets.

  Ideally this function should compute a matrix to multiply and a matrix to add to readings obtained from filter, but i'm just going to compute offsets.  
*/  
void callibrateMPU(){
  #ifdef SER_DEBUG
    Serial.println(F("Computing orientation offsets. Do not move Cansat."));
  #endif  
  float der_noise = 1;  //degrees
  float prev_value[] = {0,0};
  float cur_value[2];
  float derivative[2];
//  do{
    prev_value[0] = roll;
    prev_value[1] = pitch;
    delay(300);
    cur_value[0] = roll;
    cur_value[1] = pitch;
    for(int i = 0; i< 2; i++)
      derivative[i] = (prev_value[i] - cur_value[i]);
//    #ifdef SER_DEBUG
//      Serial.print(F("Derivatives : "));
//      Serial.print(derivative[0]);
//      Serial.print(F("/"));
//      Serial.println(derivative[1]);
//    #endif  
//  }
//  while( (abs(derivative[0]) > der_noise ) || (abs(derivative[1]) > der_noise ));
  
  rollOffset = roll;
  pitchOffset = pitch;
  
  #ifdef SER_DEBUG
    //Serial.println("Roll Offset\t: " + String(rollOffset));
    //Serial.println("Pitch Offset\t: " + String(pitchOffset));
  #endif    
}

inline void readAndCompute(){
  // read raw data from mpu9250
  mpu9250.update();
  ax = mpu9250.accX;
  ay = mpu9250.accY;
  az = mpu9250.accZ;
  gx = mpu9250.gyroX;
  gy = mpu9250.gyroY;
  gz = mpu9250.gyroZ;
  mx = mpu9250.magX;
  my = mpu9250.magY;
  mz = mpu9250.magZ;

  // update the filter, which computes orientation
  filter.updateIMU(gx, gy, gz, ax, ay, az); 
}

/*Function gets called by the interrupt at <MPU_READ_FREQ> Hertz
  The code takes 700uS to execute
  Thre mpu readings take around 10s at the current sampling to stabilise.
*/
void TC5_Handler (void) {
  readAndCompute();
  
  // These pitch, roll and yaw values are according to the cansat's orientation.
  roll    = filter.getPitch();
  pitch   = filter.getRoll();
  yaw     = filter.getYaw();
  //Serial.println("Yaw Value: " + String(yaw));
  if (abs(yaw - prevYaw) > TOLERANCE) {
    yaw_val_for_pid = yaw;
  }
  prevYaw = yaw;
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //Interrupt serviced, so clear the flag.
}

inline void disableMPUInterrupts(){
  TC5->COUNT16.INTENCLR.bit.MC0 = 1;
} 
inline void enableMPUInterrupts(){
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
}

/*
SamplePeriod: The amount of time between each interrupt.
Unit: ms
Range:  0ms to 1398ms
For 0ms, actual interrupt period becomes 0.04266

Function used for MPU at 50Hz
*/
void tc5Configure(int samplePeriod){
  // Enable GCLK for TC5 and TC5 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
  while (GCLK->STATUS.bit.SYNCBUSY);
  
  //reset TC5
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //tcIsSyncing
  while (TC5->COUNT16.CTRLA.bit.SWRST);
  
  // Set Timer counter Mode to 32 bits
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
  // Set TC5 mode as match frequency
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  //set prescaler and enable TC5
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
  //set presync mode to Reload or reset the counter on next prescaler clock
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCSYNC_PRESC;
  //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
  uint32_t counterVal = (uint16_t)((SystemCoreClock/1000000)*(samplePeriod*1000)/1024);
  TC5->COUNT16.CC[0].reg = counterVal ; //SystemCoreClock = 48M
  //TC5->COUNT16.CC[0].reg = (uint32_t)(46882);   //value for 1 second period
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //tcIsSyncing
  //SerialUSB.println("String TC5 counter value is : " + String(counterVal));
  // Configure interrupt request
  NVIC_DisableIRQ(TC5_IRQn);
  NVIC_ClearPendingIRQ(TC5_IRQn);
  NVIC_SetPriority(TC5_IRQn, 3);
  NVIC_EnableIRQ(TC5_IRQn);

  // Enable the TC5 interrupt request
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY); //tcIsSyncing 
}

//TC5 is used for servo

//TC3 is used in RH_ASK.cpp to drive NRF module

int pid() {
  setPoint = 0;
  uint32_t timeNow = millis();
  uint32_t timeDiff = timeNow - timePast;
  timePast = timeNow;
  float error = yaw_val_for_pid - setPoint;
  float integral = (error - prevError) * timeDiff;
  float derivative =  (error - prevError) / timeDiff;
  float output = KP*(error) + (KI)*(integral) + KD*(derivative);
  prevError = error;
  if ((error) < 5) {
    outputData = 90;
  }
  else {
    if (output >= KP * 5 && output <= KP * 180) {
        outputData = output * m1 + c1;
      }
    else {
        outputData = output * m2 + c2;
    }
  }
 #ifdef SER_DEBUG
    Serial.println("Yaw value without Integration Drift: " + (String)yaw_val_for_pid);
    Serial.println("Offset: " + (String)(error));
    Serial.println("Output Value: " + (String)output);
    Serial.println("Servo Mapped Value: " + (String)outputData);
  #endif
 return outputData;
}

void lineMapper(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {
  m1 = (y1 - y2) / (x1 - x2);
  c1 = y1 - x1 * m1;
  m2 = (y3 - y4) / (x3 - x4);
  c2 = y3 - x3 * m2;
}
