//#define GPS_SERIAL_TIMEOUT 200		//ms

/*
	Function reads characters from UART buffer and updates
	Packet with relevant data.
	Function doesn't wait for inconming packet as that can be read in the
	next iteration.
	Function clears entire 1024 byte receive buffer each time it is called.
	
	returns number of characters encoded/removed from buffer
*/	
int updateGps(){
	int char_count = 0;
	//Read and clear Serial Buffer
	if(!gps_uart.available())
		return 0;
	while(gps_uart.available()){
		gps.encode(gps_uart.read());
		char_count++;
	}
	dataPacket.gps_sats				= gps.satellites.value();
	dataPacket.gps_lattitude	= gps.location.lat();
	dataPacket.gps_longitude	= gps.location.lng();
	dataPacket.gps_altitude		= gps.altitude.meters();
	
	char utc_time[8];
	sprintf(utc_time, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
	dataPacket.gps_time = String(utc_time);
	
	
	return char_count;
}