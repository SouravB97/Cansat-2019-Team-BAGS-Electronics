// libraries to be included
#include <BMP280_DEV.h>
#include <Wire.h>
#include <Eeprom24C04_16.h>
#include <EEPROM.h>

#define EEPROM_ADDRESS 0x50
#define BUZZER A1
#define RELAY A2
#define MATCH_TILL 3
#define MATCH_TILL_UNEVEN 5000
#define UPPER_ALT_BOUND 4
#define LOWER_ALT_BOUND 2
#define EEPROM_SIZE 8096
#define BUZZER_ACTIVATE_ALTITUDE 1
#define HEATTIME 2000
#define MINDIFFHEIGHT 1
#define ALTITUDE_TOLERANCE 2
#define ALTITUDE_ASCEND_TOLERANCE 0.5
#define SERDEBUG true

float altitude, altitude_const, altitude_init, alt_abs, altitude_final;
// altitude = gives the BMP280 recorded altitude
// altitude_const = locally defined atitude for checking
// altitude_init = altitude value initialized
// alt_abs = absolute altitude
// altitude_final = final static altitude
short int count, countUneven, iter;
// count = MATCH_TILL, check till the point we need to match the altitude
// countUneven = MATCH_TILL_UNEVEN, check iteration to match altitude static value
// iter = iteration point to start storing data
bool record, stat, pass, uneven, cfchecker, isrtimeout, checkIfInFlight, initial_fix, dual_checker;
// record = true if altitude greater than 10m
// stat = true when the initialisation is complete
// pass = alitude greater than UPPER_BOUND
// cfchecker = check if the container is released
// isrtimeout the EEPROM is recording
// checkIfInFlight = check if the initialization is midflight
// initial_fix =  helper variable for altitude checker at initialisation
// dual checker = helper variable for altitude checker at landing

byte tempStore[4]; // extra buffer for storing the EEPROM data
union {
    float fval;
    byte bval[4];
  } floatAsBytes;
union {
    short int ival;
    byte bval[2];
    } IntAsBytes;

// initialize BMP library
BMP280_DEV bmp280;
int counter;
static Eeprom24C04_16 eeprom(EEPROM_ADDRESS);

void setup() {
  Serial.begin(9600);
  initializePins();
  initializeSensors();
  initializeTimer();
  initializeVariables();
  checkWhereAreWe();
  digitalWrite(RELAY, HIGH);
  delay(HEATTIME);
  digitalWrite(RELAY, LOW);
}

void loop() {
//   #ifdef SERDEBUG
//    altitude = getAltitiude();
//    Serial.println("----------------");
//    Serial.println("Altitude Entered" + String(altitude)); 
//   #endif
   bmp280.getAltitude(altitude);
   //logMapper();
   if (!checkIfInFlight && !stat) {
    altitudechecker();
   }
   
   if (record) {
       recordData();
   } else {
       recordChecker();
   }
   
   passAlter();
   
   if (alt_abs <= UPPER_ALT_BOUND && alt_abs >= LOWER_ALT_BOUND && pass == true && cfchecker == false) {
        containerRelease();
        cfchecker = true;
   }
   
   containerReleaseFailCheck();
   
   if ((alt_abs <= BUZZER_ACTIVATE_ALTITUDE || uneven == true) && pass == true) {
       buzzerActivate();
   }

   if (pass) {
    unevenChecker();
   }
}

void initializePins() {
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(RELAY, LOW);
  }

void initializeSensors() {
  // Code to initialize all sensors
  eeprom.initialize();
  // initiate BMP280_ addresses
  bmp280.begin(BMP280_I2C_ALT_ADDR);              // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE 
  bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE
  bmp280.setPresOversampling(OVERSAMPLING_X16);   // Setup the BMP280 to oversampling of X16
  bmp280.setIIRFilter(IIR_FILTER_16);  
}

void initializeTimer() {
  // Code to initialize timer at 1Hz
  cli();
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 7812; //(16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void initializeVariables() {
  // variable setup
  count = MATCH_TILL;
  countUneven = MATCH_TILL_UNEVEN;
  iter = 0;
  record = false, stat = false, pass = false, uneven = false, cfchecker = false, isrtimeout = false, checkIfInFlight = false;
  initial_fix = true, dual_checker = true;
  counter = 1;
  }

void checkWhereAreWe() {
  byte status_val = EEPROM.read(EEPROM.length() - 1);
  if ((bool)status_val == true) {
    checkIfInFlight = true;
    
    floatAsBytes.bval[0] = EEPROM.read(EEPROM.length() - 2);
    floatAsBytes.bval[1] = EEPROM.read(EEPROM.length() - 3);
    floatAsBytes.bval[2] = EEPROM.read(EEPROM.length() - 4);
    floatAsBytes.bval[3] = EEPROM.read(EEPROM.length() - 5);
    float altitude_init = floatAsBytes.fval;
    
    byte value_of_alt_iter[2]; 
    IntAsBytes.bval[0] = EEPROM.read(EEPROM.length() - 6);
    IntAsBytes.bval[1] = EEPROM.read(EEPROM.length() - 7);
    iter = IntAsBytes.ival;
    
    pass = (bool)EEPROM.read(EEPROM.length() - 8);
    record = true;
    }
  else if ((bool)status_val == false){    
    
    counter = 0;
    
    }
  else {
    
    counter = 0;
    
    }
  }

void logMapper() {
  #ifdef SERDEBUG
  Serial.println("Enter Altitude: ");
  Serial.println("Current Flight Log: ");
  Serial.println("----------------------");
  Serial.println("Check in Flight: " + String(checkIfInFlight));
  Serial.println("Counter Value: " + String(count));
  Serial.println("Altitude Initialized: " + String(altitude_init));
  Serial.println("Is recording started: " + String(record));
  Serial.println("EEPROM Iteration: " + String(iter));
  Serial.println("Pass Status: " + String(pass));
  Serial.println("Container Release Condition: " + String(cfchecker));
  Serial.println("Altitude Final: " + String(altitude_final));
  #endif
  }

float getAltitiude() {
  logMapper();
  while(!Serial.available()) {
    }
  return Serial.parseFloat();
}

void altitudechecker() {
  if (altitude != 0 && initial_fix == true) {
    
    if (count == MATCH_TILL) {
      initial_fix = true;
      altitude_const = altitude;
      count = MATCH_TILL - 1;
    }
    
    else {
      if (count == 0) {
        altitude_init = altitude_const;
        stat = true;
        initial_fix = false;
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
      }
      
      else if ((abs(altitude - altitude_const) <= ALTITUDE_TOLERANCE)) {
          count--;
        }
      
      else {
        if (initial_fix) {
          count = MATCH_TILL;
          stat = false;
        }
      }
    }
  }
}

void recordData() {
  alt_abs = altitude - altitude_init;
  
  if (isrtimeout) {
    
    eeWrite(alt_abs);
    isrtimeout = false;
    
  }
}

void recordChecker() {
  if ((stat) && (altitude - altitude_init) >= MINDIFFHEIGHT) {
    
    record = true;
  
  }
}

// code for writing into EEPROM
void eeWrite(float altVal) {
        if (counter == 0) {
          bool check = true;
          EEPROM.write(EEPROM.length() - 1, (byte)check);
          EEPROM.write(EEPROM.length() - 1, (byte)check);
          floatAsBytes.fval = altitude_init;
          tempStore[0] = floatAsBytes.bval[0];
          tempStore[1] = floatAsBytes.bval[1];
          tempStore[2] = floatAsBytes.bval[2];
          tempStore[3] = floatAsBytes.bval[3];
          EEPROM.write(EEPROM.length() - 2, tempStore[0]);
          EEPROM.write(EEPROM.length() - 3, tempStore[1]);
          EEPROM.write(EEPROM.length() - 4, tempStore[2]);
          EEPROM.write(EEPROM.length() - 5, tempStore[3]);
          counter++;
        }
        floatAsBytes.fval = altVal;
        tempStore[0] = floatAsBytes.bval[0];
        tempStore[1] = floatAsBytes.bval[1];
        tempStore[2] = floatAsBytes.bval[2];
        tempStore[3] = floatAsBytes.bval[3];
        eeprom.writeBytes(iter, iter + 3, tempStore);
        delay(10);
        iter = iter + 4;
        if (iter == EEPROM_SIZE) {
          iter = 0;
        }
        union {
          short int ival;
          byte bval[2];
        } IntAsBytes;
        IntAsBytes.ival = iter;
        EEPROM.write(EEPROM.length() - 6, IntAsBytes.bval[0]);
        EEPROM.write(EEPROM.length() - 7, IntAsBytes.bval[1]);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
}

ISR(TIMER1_COMPA_vect) {
  isrtimeout = true;
}

void passAlter() {
  if (alt_abs >= UPPER_ALT_BOUND && pass == false) {
    pass = true;
    EEPROM.write(EEPROM.length() - 8, (byte)pass);
   }
}

void containerRelease() {
  digitalWrite(RELAY, HIGH);
  delay(HEATTIME);
  digitalWrite(RELAY, LOW);
  eeWrite(-1);
  eeWrite(alt_abs);
  // saving the point of the container release
}


void containerReleaseFailCheck() {
  if (cfchecker == false && alt_abs < LOWER_ALT_BOUND && pass == true) {
    containerRelease(); // release the container
    cfchecker = true;
  }
}

void buzzerActivate() {
  record = false;
  EEPROM.write(EEPROM.length() - 1, (byte)false);
  EEPROM.write(EEPROM.length() - 8, (byte)false);
  #ifdef SERDEBUG
    Serial.println("BUZZER ACTIVATED");
    digitalWrite(LED_BUILTIN, HIGH);
  #endif
  while (true) {
    digitalWrite(BUZZER, HIGH);
    delay(1000);
    digitalWrite(BUZZER, LOW);
    delay(1000);
  }
}

void unevenChecker() {
    if (dual_checker) {
      if (countUneven == MATCH_TILL_UNEVEN) {
        altitude_const = altitude;
        countUneven = MATCH_TILL_UNEVEN - 1;
      }
      else {
        if (countUneven == 0 && dual_checker == true) {
          altitude_final = altitude_const;
          uneven = true;
          dual_checker = !uneven;
        }
        else if ((abs(altitude - altitude_const) <= ALTITUDE_ASCEND_TOLERANCE)) {
            countUneven--;
          }
        else {
          if (dual_checker) {
            countUneven = MATCH_TILL_UNEVEN;
            uneven = false;
          }
        }
      }
    }
}
