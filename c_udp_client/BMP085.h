
#ifndef BMP085_H

void unExport_GPIOPin(char* pin);
// setMux - set up the multiplexor on A0 to connect it to ADC VIN0
void setMux(void);
void setGPIOPin(char* pin, char* dir, char* drive, char* val);
void bmp085_Calibration(void);
// Calculate temperature given uncalibrated temperature
// Value returned will be in units of 0.1 deg C
unsigned int bmp085_GetTemperature(unsigned int ut);
// Calculate pressure given uncalibrated pressure
// Value returned will be in units of Pa
unsigned int bmp085_GetPressure(unsigned int up);
// Read the uncompensated temperature value
unsigned int bmp085_ReadUT();
// Read the uncompensated pressure value
unsigned int bmp085_ReadUP();
#endif // BMP085_H
