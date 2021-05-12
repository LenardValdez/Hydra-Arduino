/*
 * file DFRobot_PH.cpp * @ https://github.com/DFRobot/DFRobot_PH
 *
 * Arduino library for Gravity: Analog pH Sensor / Meter Kit V2, SKU: SEN0161-V2
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  2018-04
 */

#include "Arduino.h"
#include "DFRobot_PH.h"



DFRobot_PH::DFRobot_PH()
{
    this->_phValue        = 7.0;
    this->_acidVoltage    = 2032.44;    //buffer solution 4.0 at 25C
    this->_neutralVoltage = 1500.0;     //buffer solution 7.0 at 25C
}

DFRobot_PH::~DFRobot_PH()
{

}

void DFRobot_PH::begin()
{
    this->_neutralVoltage = 1500.0;  // new EEPROM, write typical voltage
    this->_acidVoltage = 2032.44;  // new EEPROM, write typical voltage
}

float DFRobot_PH::readPH(float voltage, float temperature)
{
    float slope = (7.0-4.0)/((this->_neutralVoltage-1500.0)/3.0 - (this->_acidVoltage-1500.0)/3.0);  // two point: (_neutralVoltage,7.0),(_acidVoltage,4.0)
    float intercept =  7.0 - slope*(this->_neutralVoltage-1500.0)/3.0;
    //Serial.print("slope:");
    //Serial.print(slope);
    //Serial.print(",intercept:");
    //Serial.println(intercept);
    this->_phValue = slope*(voltage-1500.0)/3.0+intercept;  //y = k*x + b
    return _phValue;
}