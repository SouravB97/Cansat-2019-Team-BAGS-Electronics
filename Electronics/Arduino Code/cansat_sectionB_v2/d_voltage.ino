/*
 * Connections:
 * VBAT---|R1|-----|R2|---GND
 *              |
 *           BAT_VOL
 */
#define R1 4700	//0 for direct connection
#define R2 10000  //any value for direct connection (Ideally infinite)
#define ADC_MAX 4096 //10 bit ADC
#define VCC 3.3 //Max voltage read by mcu

const float RES_RATIO = ((float)R1)/((float)R2);
const float VOLTAGE_DIV = 1/(1 + RES_RATIO); //Factor for calculating battery voltage
const float MAX_INP_VOLTAGE = (VCC)/VOLTAGE_DIV; //in mV
const float RESOLUTION = (MAX_INP_VOLTAGE)/((float)ADC_MAX);
const float INP_IMPEDANCE = R1 + R2;

void initBatteryVoltage() {
  // initialize battery voltage function
  pinMode(BAT_VOL,INPUT);
  analogReadResolution(12); // Set analog input resolution to max, 12-bits
}

float getBatteryVoltage() {
  return analogRead(BAT_VOL)*RESOLUTION;
}

String getADCInfo(){
  String info = "======================= ADC Info =======================\n";
  info += "R1 (Ohms)\t\t\t:\t\t";
  info += String(R1);
  info += "\n";
  info += "R2 (Ohms)\t\t\t:\t\t";
  info += String(R2);
  info += "\n";
  info += "Maximum Input Voltage (V)\t:\t\t";
  info += String(MAX_INP_VOLTAGE);
  info += "\n";
  info += "Resolution (mV)\t\t\t:\t\t";
  info += String(RESOLUTION*1000);
  info += "\n";
  info += "Input Impedance (Ohms)\t\t:\t\t";
  info += String(INP_IMPEDANCE);
  info += "\n";
  info += "========================================================\n";
  return info;
}
