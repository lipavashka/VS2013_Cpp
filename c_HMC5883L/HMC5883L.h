#ifndef HMC5883L_H
#define HMC5883L_H

//extern float XmagF, YmagF, ZmagF;
//extern float heading = 0.;

extern signed int Xmag, Ymag, Zmag;

typedef enum {
	HMC5883_MAGGAIN_1_3 = 0x20,  // +/- 1.3
	HMC5883_MAGGAIN_1_9 = 0x40,  // +/- 1.9
	HMC5883_MAGGAIN_2_5 = 0x60,  // +/- 2.5
	HMC5883_MAGGAIN_4_0 = 0x80,  // +/- 4.0
	HMC5883_MAGGAIN_4_7 = 0xA0,  // +/- 4.7
	HMC5883_MAGGAIN_5_6 = 0xC0,  // +/- 5.6
	HMC5883_MAGGAIN_8_1 = 0xE0   // +/- 8.1
} hmc5883MagGain;

void initHMC5883L(void);
void readHMC5883Lraw(void);
void setMagGain(unsigned short gain);

#endif
