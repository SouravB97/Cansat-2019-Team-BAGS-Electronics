void initSerial() {
  //I2C
  pinPeripheral(SDA, PIO_SERCOM_ALT);
  pinPeripheral(SCL, PIO_SERCOM_ALT);

  //SPI is initialised by default
}
