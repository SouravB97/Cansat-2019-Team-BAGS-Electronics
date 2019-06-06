String packetFileName = "packets";
String packetFileExt	= "csv";
String completePacketFileName;

String missionFileName = "mission";
String missionFileExt	= "log";
String completeMissionFileName;

bool sd_initialised = false;

void initSD(){
	/*
	Sets up the SD card and creates mission.log and packets.csv
	*/
	
	//----------Setting up sd card-----------//
	#ifdef SER_DEBUG
		Serial.print("Initializing SD card...");
	#endif
	
  if (!SD.begin(SD_SELECT)) {
		#ifdef SER_DEBUG
			Serial.println("initialization failed!");
		#endif
    return;
  }
	#ifdef SER_DEBUG
		Serial.println("initialization done.");
	#endif
	sd_initialised = true;
	// packetFile = newFile(packetFileName, packetFileExt);
	// missionLog = newFile(missionFileName, missionFileExt); 	
}
void createFiles(){
	packetFile = newFile(packetFileName, packetFileExt);
	missionLog = newFile(missionFileName, missionFileExt);	
}


File newFile(String fileName, String fileExt){
	/*
	Creates and sets up a new file.
	*/
	
	File tempFile;
 //completeFileName = String(fileName + "_" + String(millis()) + "." + fileExt);
  String completeFileName = String(fileName + "." + fileExt);	//filename cannot be too long

	//delete any pre-existing record
  if(SD.exists(completeFileName)){
		#ifdef SER_DEBUG
			Serial.println("Found " + completeFileName +". Removing it.");
    #endif
		SD.remove(completeFileName);
  }
  //Create a new file
	#ifdef SER_DEBUG
		Serial.println("Creating " + completeFileName + " ...");
  #endif
	if (tempFile = SD.open(completeFileName, FILE_WRITE)) {
		#ifdef SER_DEBUG
			Serial.println(completeFileName + " created.");
		#endif
	}
  else {	//die
		#ifdef SER_DEBUG
			Serial.println("Failed to create " + completeFileName);
    #endif
		return *(new File());  //return an empty file object
  }
  tempFile.close();	
	return (tempFile);
}

boolean logEvent(const String text){
	/*
	Safely stores the text to mission.log with time stamp
	*/
	return safe_println(&missionLog, String(getTimeStamp() + "\t" + text));
}

boolean safe_print(File* file_ptr, const String text){
	/*
		Ensures no two files are opened at the same time, and data is saved to SD card
		Doesn't implement file locking, or stop interrupts.
	*/
	if(!sd_initialised) return false;
	File temp;
  //open file in write mode.
  if (!(temp = SD.open(file_ptr->name(), FILE_WRITE))) {
		#ifdef SER_DEBUG		//die
			Serial.println("\t Failed to open " + String(file_ptr->name()));
		#endif
    return false;
  }
	temp.print(text);
	// #ifdef SER_DEBUG
		// Serial.println("\t Printed the line: " + text);
	// #endif
  temp.close();	
	return true;
}

boolean safe_println(File* file_ptr, const String text){
	return safe_print(file_ptr, String(text + "\n"));
}

boolean safe_print(const String filename, const String text){
	File file =  SD.open(filename, FILE_READ);
	return safe_print(&file, text);
}

boolean safe_println(const String filename, const String text){
	File file =  SD.open(filename, FILE_READ);
	return safe_println(&file, text);
}

void readFile(File* file_ptr, Stream &s){
	Stream* stream = &s;
	File temp;
	String completeFileName = String(file_ptr->name());
	//Check if the file eists
	if(!SD.exists(completeFileName)){
		stream->println("\t" + completeFileName + " Doesn't exist");
		return;	//die
	}
	
  //open file in read mode.
  if (temp = SD.open(completeFileName, FILE_READ)) {
		uint32_t size = temp.size();
		stream->println("------------------------------------------------------");
		stream->println("File:\t" + completeFileName + "\t\tSize: " + String(size) + " Bytes");
		stream->println("------------------------------------------------------");
  }
  else {	//die
		stream->println("\t Failed to open " + completeFileName);
    return;
  }

  while (temp.available()) {
		stream->write(temp.read());
  }
		stream->println("------------------------------------------------------");
  temp.close();		
}
//Default Stream Serial
void readFile(File* file_ptr){
	readFile(file_ptr, Serial);
}

void readFile(const String filename){
	File file =  SD.open(filename, FILE_READ);
	readFile(&file);
	//call file destructor
}

//to be updated with time from RTC
String getTimeStamp(){
	//return getRTCDateTime();
	return getRTCTime();
}
