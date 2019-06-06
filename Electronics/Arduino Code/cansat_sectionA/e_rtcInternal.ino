//Variable to store startTime of mission
uint32_t startTime = 1560605452;	// Saturday, June 15, 2019 1:30:52 PM Get it from https://www.epochconverter.com/

void resetMissionTime(){
	startTime = rtc.getEpoch();
	#ifdef LOG_MISSION
		logEvent("Mission time reset.");
	#endif		
}
//Returns the mission time: i.e. subtraction of "current unix time" and program's "startTime"
uint32_t getMissionTime(){
  return (rtc.getEpoch() - startTime);
}

inline String getRTCDate(){
  return (twoDigits(rtc.getDay()) + "/" + twoDigits(rtc.getMonth()) + "/" + twoDigits(rtc.getYear()));
}
/*
	Returns time in UTC format
*/	
String getRTCTime(){
  return (twoDigits(rtc.getHours()) + ":" + twoDigits(rtc.getMinutes()) + ":" + twoDigits(rtc.getSeconds()));
}

String getRTCDateTime(){
	String YYYY = String("20" + twoDigits(rtc.getYear()));
	String MM 	= twoDigits(rtc.getMonth());
	String DD 	= twoDigits(rtc.getDay());
	String HH 	= twoDigits(rtc.getHours());
	String mm 	= twoDigits(rtc.getMinutes());
	String rtcSS 	= twoDigits(rtc.getSeconds());
	
	return String(YYYY+"-"+MM+"-"+DD+" "+HH+":"+mm+":"+rtcSS);
}

void rtcAlarmMatch()
{
	pinMode(DBG_LED, OUTPUT);
	//flash led 3 times
	for(int i = 0; i<3; i++){
		digitalWrite(DBG_LED, HIGH);
		delay(500);
		digitalWrite(DBG_LED, LOW);
		delay(500);
	}  
}


String twoDigits(int x){
	String s = "";
	if(x<10)
		s = "0";
	return String(s + String(x));
}

byte prompt(String ask, int mini, int maxi, Stream &s)
{
	Stream* stream = &s;
  stream->print(ask + "? ");
  while (!stream->available()) ; // Wait for numbers to come in
  byte rsp = stream->parseInt();
  if ((rsp >= mini) && (rsp <= maxi))
  {
    stream->println(rsp);
    return rsp;
  }
  else
  {
    stream->println("Invalid.");
    return mini;
  }
}
byte prompt(String ask, int mini, int maxi)
{
	prompt(ask, mini, maxi, Serial);
}