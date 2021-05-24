//init indication
bool initialized = false;

//Global vars   

struct constants
{
    //tolerance variables
    int air_humidity_min;
    int air_humidity_max;

    int air_temperature_min;
    int air_temperature_max;

    float ec_reading_min;
    int ec_reading_max;

    float ph_reading_min;
    float ph_reading_max;
} tolerance;