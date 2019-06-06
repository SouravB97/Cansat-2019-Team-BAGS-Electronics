uint8_t nrf_rec_buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t nrf_buf_len = sizeof(nrf_rec_buf);

bool nrf_tx_flag = true;
bool nrf_rx_flag = true;

void initNrf(){
	if(!nrf24.init() || !nrf24.setChannel(5) || !nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPowerm18dBm)){
		#ifdef SER_DEBUG
			Serial.println("NRF initialisation failed.");
		#endif
		#ifdef LOG_MISSION
			logEvent("NRF initialisation failed.");
		#endif
	}
	// else{
		// #ifdef SER_DEBUG
			// Serial.println("NRF initialised.");
		// #endif		
		// #ifdef LOG_MISSION
			// logEvent("NRF initialised.");
		// #endif
	// }
}

void nrfTransmit(int16_t tx_data){
	//have to break this into two bytes and fill it into a rx_buffer
	uint8_t data_rx_buf[2];
	data_rx_buf[1] = (uint8_t)tx_data;
	data_rx_buf[0] = (uint8_t)(tx_data >> 8);
	
	nrf24.send(data_rx_buf, sizeof(data_rx_buf));
	nrf24.waitPacketSent();		
}
void nrfTransmit(uint8_t tx_data){
	nrf24.send(&tx_data, sizeof(tx_data));
	nrf24.waitPacketSent();			
}
/*
	Transmits an 8 bit command and waits for ACK (0xff)
	timesout after 500 ms.
*/	
bool nrfTransmitWithAck(uint8_t tx_data){
	static const int timeout = 500;
	bool succesful = false;
	
	nrf24.send(&tx_data, sizeof(tx_data));
	nrf24.waitPacketSent();
	
	uint32_t tstart = millis();
	while(millis() - tstart <= timeout){
		if(nrf24.available()){
			// Should be a message for us now   
			uint8_t ack = 0;
			if (nrf24.recv(nrf_rec_buf, &nrf_buf_len)){
				ack = nrf_rec_buf[0];
			}
			//if(ack == 0xff)
				succesful = true;
			break;
		}
	}
	return succesful;	
}

/*
   16 bit integer data
*/
int16_t nrfReceiveData() {
  int16_t data = 0;
  if (nrf24.recv(nrf_rec_buf, &nrf_buf_len))
    data = (int16_t)((nrf_rec_buf[0] << 8) | (nrf_rec_buf[1]));
  return data;
}

void setNrfTxFlag(){
	nrf_tx_flag = true;
}
void clearNrfTxFlag(){
	nrf_tx_flag = false;
}
void setNrfRxFlag(){
	nrf_rx_flag = true;
}
void clearNrfRxFlag(){
	nrf_rx_flag = false;
}
bool getNrfTxFlag(){
	return nrf_tx_flag;
}
bool getNrfRxFlag(){
	return nrf_rx_flag;
}