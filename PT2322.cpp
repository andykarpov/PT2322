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

#include <Arduino.h>
#include <Wire.h>
#include <PT2322.h>

int toneAttenuation[30] = {
    -14,-13,-12,-11,-10,-9,-8,-7,-6,-5,-4,-3,-2,-1,0,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0
};

unsigned const char toneValues[30] = {
    0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9,10,10,11,11,12,12,13,13,14,14,15
};

// tone table lookup
unsigned char PT2322::toneLookup (int tone){
    int* ptr = toneAttenuation;
    for (int i=0;i<sizeof(toneAttenuation);i++){
        if (tone == *(ptr+i)){
            return toneValues[i];
        }            
    }
    // default if given an invalid value
    return 7;
}

// helper method
unsigned char PT2322::HEX2BCD (unsigned char x)
{
    unsigned char y;
    y = (x / 10) << 4;
    y = y | (x % 10);
    return (y);
}
   
// helper method
int PT2322::writeI2CChar(unsigned char c)   
{   
//  shift address to right - Wire library always uses 7 bit addressing
    Wire.beginTransmission(0x88 >> 1); // transmit to device 0x88, PT2322
    Wire.write(c);   
    int rtnCode = Wire.endTransmission(); // stop transmitting
    return rtnCode;
} 

// initialize PT2322
int PT2322::init(void)   
{   
    delay(300); // in case this is first time - I2C bus not ready for this long on power on with 10uF cref
    function = _3D_OFF;       //mute OFF, 3D OFF, tone control ON   
    unsigned char masterVolumeValue = 0;    //master volume = -15db - temporary at 0   
    unsigned char bassValue   = 0x07;       //Bass   = -0dB   
    unsigned char middleValue = 0x07;       //Middle = -0dB   
    unsigned char trebleValue = 0x07;       //Treble = -0dB   
        
    // initialize device
    writeI2CChar(SYSTEM_RESET);
    writeI2CChar(INPUT_SW_ACTIVE);  // required to activate 

    // set the trim volumes to zero
    writeI2CChar(FL_VOLUME_CONTROL);      //0db   
    writeI2CChar(FR_VOLUME_CONTROL);      //0db   
    writeI2CChar(CENTER_VOLUME_CONTROL);  //0db   
    writeI2CChar(RL_VOLUME_CONTROL);      //0db   
    writeI2CChar(RR_VOLUME_CONTROL);      //0db   
    writeI2CChar(SUB_VOLUME_CONTROL);     //0db   
    
    // set the master voume
    Wire.beginTransmission(0x88 >> 1); // transmit to device 0x88, PT2322
    Wire.write(MASTER_VOLUME_1STEP | (HEX2BCD(masterVolumeValue)  &  0x0f));   
    Wire.write(MASTER_VOLUME_10STEP | ((HEX2BCD(masterVolumeValue)  &  0xf0)>>4));   
    Wire.endTransmission();       // stop transmitting

    // set default function
    writeI2CChar(FUNCTION_SELECT | function);   

    // and finish with the tone controls
    writeI2CChar(BASS_TONE_CONTROL | bassValue);   
    writeI2CChar(MIDDLE_TONE_CONTROL | middleValue);   
    return writeI2CChar(TREBLE_TONE_CONTROL | trebleValue);   
}   

// mute on
void PT2322::muteOn(void)   
{   
    function |= MUTE_ON;
    writeI2CChar(FUNCTION_SELECT | function);
}   

// mute off
void PT2322::muteOff(void)   
{   
    function  &= (0x0f - MUTE_ON);   
    writeI2CChar(FUNCTION_SELECT | function);
}   
   
// 3D on
void PT2322::_3DOn(void)   
{   
    function  &= (0x0f - _3D_OFF);   
    writeI2CChar(FUNCTION_SELECT | function);
}   
   
// 3D off
void PT2322::_3DOff(void)   
{   
    function |= _3D_OFF;   
    writeI2CChar(FUNCTION_SELECT | function);
}   
   
// tone on
void PT2322::toneOn(void)   
{   
    function  &= (0x0f - TONE_DEFEAT);   
    writeI2CChar(FUNCTION_SELECT | function);
}   
   
// tone off
void PT2322::toneOff(void)   
{   
    function |= TONE_DEFEAT;   
    writeI2CChar(FUNCTION_SELECT | function);
}   

//range : 0 to -15dB   
void PT2322::leftVolume(unsigned char flv)   
{   
     writeI2CChar(FL_VOLUME_CONTROL | -flv);
}   
      
//range : 0 to -15dB   
void PT2322::rightVolume(unsigned char frv)   
{   
    writeI2CChar(FR_VOLUME_CONTROL | -frv);
}      
   
//range : 0 to -15dB   
void PT2322::centerVolume(unsigned char cv)   
{   
    writeI2CChar(CENTER_VOLUME_CONTROL | -cv);
}      
   
//range : 0 to -15dB   
void PT2322::rearLeftVolume(unsigned char rlv)   
{   
    writeI2CChar(RL_VOLUME_CONTROL | -rlv);
}      
   
//range : 0 to -15dB   
void PT2322::rearRightVolume(unsigned char rrv)   
{   
    writeI2CChar(RR_VOLUME_CONTROL | -rrv);
}      
   
//range : 0 to -15dB   
void PT2322::subwooferVolume(unsigned char sv)   
{   
    writeI2CChar(SUB_VOLUME_CONTROL | -sv);
}      
   
//range : 0 to -79dB   
void PT2322::masterVolume(unsigned char mv)   
{   
    Wire.beginTransmission(0x88 >> 1); // transmit to device 0x88, PT2322
    Wire.write(MASTER_VOLUME_1STEP  | (HEX2BCD(-mv)   &  0x0f));   
    Wire.write(MASTER_VOLUME_10STEP | ((HEX2BCD(-mv)  &  0xf0)>>4));   
    Wire.endTransmission();       // stop transmitting
}

//range : +14 to -14dB, 2dB step   
void PT2322::bass(int tb)   
{   
    unsigned char tbv = toneLookup(tb);
    writeI2CChar(BASS_TONE_CONTROL | tbv);
}      
   
//range : +14 to -14dB, 2dB step   
void PT2322::middle(int tm)   
{   
    unsigned char tmv = toneLookup(tm);
    writeI2CChar(MIDDLE_TONE_CONTROL | tmv);
}      
   
//range : +14 to -14dB, 2dB step   
void PT2322::treble(int tt)   
{   
    unsigned char ttv = toneLookup(tt);
    writeI2CChar(TREBLE_TONE_CONTROL | ttv);
}   
   
   
