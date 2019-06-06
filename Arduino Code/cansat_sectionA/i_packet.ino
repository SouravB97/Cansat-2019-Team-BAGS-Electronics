
boolean savePacket(packet* dataPacket){
	/*
	Stores packet to SD Card
	*/
	return safe_println(&packetFile, dataPacket->toString());
}

packet* getLastPacket(File* packetFile){
	/*
	Reads SD card and creates a new packet from the last line.
	*/
	packet* newPacket = new packet(TEAM_ID);
	
	//Here's how I would have done it in a scripting language
	/*
		File csvFile = SD.open(packetFile->name(), FILE_WRITE);
		int cursor = csvFile.getCursor();
		csvFile.close();
		csvFile = SD.open(packetFile->name(), FILE_READ);
		csvFile.setCursor(cursor-1);
		String line = csvFile.readLine();
		String fields[] = line.split(',');
		newPacket->teamID = fields[0].toInt();
		.
		.
		newPacket->pressure = fields[4].toFloat();
		.
		.
		.
	*/
	return newPacket;
}

packet* getPacket(File* packetFile, int line){
	/*
	Reads SD card and creates a new packet from a specific line
	*/
	packet* newPacket = new packet(TEAM_ID);
	return newPacket;	
}

boolean transmitPacketData(packet* dataPacket){
	/*
	Transmits packet fields over zigbee
	*/
	zigbee.write(dataPacket->team_id);
	zigbee.write(dataPacket->mission_time);
	zigbee.write(dataPacket->packet_count);
	zigbee.write(dataPacket->altitude);
	zigbee.write(dataPacket->pressure);
	zigbee.write(dataPacket->temperature);
	zigbee.write(dataPacket->voltage);
	zigbee.print(dataPacket->gps_time);	//has to be converted to gps utc time
	zigbee.write(dataPacket->gps_lattitude);
	zigbee.write(dataPacket->gps_longitude);
	zigbee.write(dataPacket->gps_altitude);
	zigbee.write(dataPacket->gps_sats);
	zigbee.write(dataPacket->pitch);
	zigbee.write(dataPacket->roll);
	zigbee.write(dataPacket->blade_spin_rate);
	zigbee.write(dataPacket->software_state);
	zigbee.write(dataPacket->bonus_direction);

  return true;
}

boolean transmitPacketString(packet* dataPacket){
	/*
	Transmits packet as a csv string over zigbee
	*/
	zigbee.println(dataPacket->toString());
  return true;
}
