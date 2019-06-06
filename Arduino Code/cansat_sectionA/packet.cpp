#include "packet.h"
#define Serial SerialUSB
#define SER_DEBUG

//constructor
packet::packet(const uint16_t team_id){
	this->team_id = team_id;
	this->mission_time = 0;
	this->packet_count = 0;
	this->altitude = 0;
	this->pressure = 0;
	this->temperature = 0;
	this->voltage = 0;
	this->gps_time = "01:37:56";
	this->gps_lattitude = 0;
	this->gps_longitude = 0;
	this->gps_altitude = 0;
	this->gps_sats = 0;
	this->pitch = 0;
	this->roll = 0;
	this->blade_spin_rate = 0;
	this->software_state = boot;
	this->bonus_direction = 0;
}

String packet::toString(){
	String csv = String( 
	  String(this->team_id) + "," +
		String(this->mission_time) + "," +
		String(this->packet_count) + "," +
		String(this->altitude, 1) + "," +					//Format upto 1 decimal places
		String(this->pressure, 0) + "," +					//Format upto 0 decimal places
		String(this->temperature, 1) + "," +			//Format upto 1 decimal places
		String(this->voltage, 2) + "," +					//Format upto 2 decimal places
		this->gps_time + "," +
		String(this->gps_lattitude, 4) + "," +		//Format upto 4 decimal places
		String(this->gps_longitude, 4) + "," +		//Format upto 4 decimal places
		String(this->gps_altitude, 1) + "," +			//Format upto 1 decimal places
		String(this->gps_sats) + "," +
		String(this->pitch, 0) + "," +						//Format upto 0 decimal places
		String(this->roll, 0) + "," +							//Format upto 0 decimal places
		String(this->blade_spin_rate) + "," +
		//op_state_string_lut[this->software_state] + "," +
		String(this->software_state) + "," +
		String(this->bonus_direction)
	);
	this->_csv_string = csv;
	return csv;
}

void packet::display(){
	#ifdef SER_DEBUG
		Serial.println(F("======================= PACKET ======================="));
		Serial.println("Packet Size\t\t:\t\t" + String(sizeof(*this)) + "\tBytes");
		Serial.println("------------------------------------------------------");
		Serial.println("Team ID\t\t\t:\t\t" + String(this->team_id));
		Serial.println("Mission Time\t\t:\t\t" + String(this->mission_time) + "\ts");
		Serial.println("Packet Count\t\t:\t\t" + String(this->packet_count));
		Serial.println("Altitude\t\t:\t\t" + String(this->altitude) + "\tm");
		Serial.println("Pressure\t\t:\t\t" + String(this->pressure) + "\tPa");
		Serial.println("Temperature\t\t:\t\t" + String(this->temperature) + "\tC");
		Serial.println("Battery Voltage\t\t:\t\t" + String(this->voltage) + "\tV");
		Serial.println("GPS Time\t\t:\t\t" + this->gps_time);
		Serial.println("GPS Lattitude\t\t:\t\t" + String(this->gps_lattitude) + "\tdeg");
		Serial.println("GPS Longitude\t\t:\t\t" + String(this->gps_longitude) + "\tdeg");
		Serial.println("GPS Altitude\t\t:\t\t" + String(this->gps_altitude) + "\tm");
		Serial.println("GPS Sats\t\t:\t\t" + String(this->gps_sats));
		Serial.println("Pitch\t\t\t:\t\t" + String(this->pitch) + "\tdeg");
		Serial.println("Roll\t\t\t:\t\t" + String(this->roll) + "\tdeg");
		Serial.println("Blade Spin Rate\t\t:\t\t" + String(this->blade_spin_rate) + "\tRPM");
		Serial.println("Op state\t\t:\t\t" + op_state_string_lut[this->software_state] + "");
		Serial.println("Direction\t\t:\t\t" + String(this->bonus_direction) + "\tdeg");
		Serial.println(F("======================================================"));
	#endif
}
