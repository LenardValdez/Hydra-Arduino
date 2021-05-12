/*
 * file DFRobot_PH.h * @ https://github.com/DFRobot/DFRobot_PH
 *
 * Arduino library for Gravity: Analog pH Sensor / Meter Kit V2, SKU: SEN0161-V2
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.0
 * date  2018-04
 */

#define _DFROBOT_PH_H_
#include "Arduino.h"

class DFRobot_PH
{
public:
    DFRobot_PH();
    ~DFRobot_PH();
    float   readPH(float voltage, float temperature); // voltage to pH value, with temperature compensation
    void    begin();   //initialization

private:
    float  _phValue;
    float  _acidVoltage;
    float  _neutralVoltage;
};
