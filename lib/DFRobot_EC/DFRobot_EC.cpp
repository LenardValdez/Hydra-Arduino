/*
 * file DFRobot_EC.cpp
 * @ https://github.com/DFRobot/DFRobot_EC
 *
 * Arduino library for Gravity: Analog Electrical Conductivity Sensor / Meter Kit V2 (K=1), SKU: DFR0300
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.01
 * date  2018-06
 */
#include "Arduino.h"

#include "DFRobot_EC.h"

#define RES2 820.0
#define ECREF 200.0

DFRobot_EC::DFRobot_EC()
{
    this->_ecvalue                = 0.0;
    this->_kvalue                 = 1.0;
    this->_kvalueLow              = 1.0;
    this->_kvalueHigh             = 1.0;
} 

DFRobot_EC::~DFRobot_EC()
{

}

void DFRobot_EC::begin() {
    this->_kvalueLow = 1.0;                       // For new EEPROM, write default value( K = 1.0) to EEPROM
    this->_kvalueHigh = 1.0;                      // For new EEPROM, write default value( K = 1.0) to EEPROM
    this->_kvalue =  this->_kvalueLow;            // set default K value: K = kvalueLow
}

float DFRobot_EC::readEC(float voltage, float temperature) {
    float value = 0,valueTemp = 0;
    this->_rawEC = 1000*voltage/RES2/ECREF;
    valueTemp = this->_rawEC * this->_kvalue;
    //automatic shift process
    //First Range:(0,2); Second Range:(2,20)
    if(valueTemp > 2.5){
        this->_kvalue = this->_kvalueHigh;
    }else if(valueTemp < 2.0){
        this->_kvalue = this->_kvalueLow;
    }

    value = this->_rawEC * this->_kvalue;             //calculate the EC value after automatic shift
    value = value / (1.0+0.0185*(temperature-25.0));  //temperature compensation
    this->_ecvalue = value;                           //store the EC value for Serial CMD calibration
    return value;
}
