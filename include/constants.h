#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__
//init indication
bool initialized = false;
bool primed = false;
bool first_run = true;

//Global var struct   
struct constants
{
    //tolerance variables
    float air_humidity_min;
    float air_humidity_max;

    float air_temperature_min;
    float air_temperature_max;

    float ec_reading_min;
    float ec_reading_max;

    float ph_reading_min;
    float ph_reading_max;
} tolerance;


#endif // __CONSTANTS_H__