/* Arduino PT2322 Library
 * Copyright (C) 2009 by oddWires
 *
 * This file is part of the Arduino PT2322 Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * <http://www.gnu.org/licenses/>.
 */ 

#define  FL_VOLUME_CONTROL      0x10 
#define  FR_VOLUME_CONTROL      0x20 
#define  CENTER_VOLUME_CONTROL  0x30 
#define  RL_VOLUME_CONTROL      0x40 
#define  RR_VOLUME_CONTROL      0x50 
#define  SUB_VOLUME_CONTROL     0x60 
#define  FUNCTION_SELECT        0x70 
#define  BASS_TONE_CONTROL      0x90 
#define  MIDDLE_TONE_CONTROL    0xa0 
#define  TREBLE_TONE_CONTROL    0xb0 
#define  INPUT_SW_ACTIVE        0xc7 
#define  MASTER_VOLUME_1STEP    0xd0 
#define  MASTER_VOLUME_10STEP   0xe0 
#define  SYSTEM_RESET           0xff 

#define  MUTE_ON                0x08
#define  _3D_OFF                0x04
#define  TONE_DEFEAT            0x02


#ifndef PT2322_h
#define PT2322_h

//#include <Arduino.h>
//#include <Wire.h>

class PT2322 {
    
private:
    unsigned char function;           
    unsigned char toneLookup(int);
    
    unsigned char HEX2BCD (unsigned char x);
    int writeI2CChar(unsigned char c);
    
    
public:
    int init(void); 
    void muteOn(void); 
    void muteOff(void); 
    void _3DOn(void); 
    void _3DOff(void); 
    void toneOn(void); 
    void toneOff(void); 
    void leftVolume(unsigned char flv); 
    void rightVolume(unsigned char frv); 
    void centerVolume(unsigned char cv); 
    void rearLeftVolume(unsigned char rlv); 
    void rearRightVolume(unsigned char rrv); 
    void subwooferVolume(unsigned char sv); 
    void masterVolume(unsigned char mv); 
    void bass(int tb); 
    void middle(int tm); 
    void treble(int tt);
};

#endif

