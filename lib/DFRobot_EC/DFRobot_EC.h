/*
 * file DFRobot_EC.h * @ https://github.com/DFRobot/DFRobot_EC
 *
 * Arduino library for Gravity: Analog Electrical Conductivity Sensor / Meter Kit V2 (K=1), SKU: DFR0300
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2018
 * Copyright   GNU Lesser General Public License
 *
 * version  V1.01
 * date  2018-06
 */

#define _DFROBOT_EC_H_

#include "Arduino.h"

class DFRobot_EC
{
public:
    DFRobot_EC();
    ~DFRobot_EC();
    float   readEC(float voltage, float temperature);                       // voltage to EC value, with temperature compensation
    void    begin();                                                        //initialization

private:
    float  _ecvalue;
    float  _kvalue;
    float  _kvalueLow;
    float  _kvalueHigh;
    float  _rawEC;
};
