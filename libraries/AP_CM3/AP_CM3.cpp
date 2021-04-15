#include <AP_CM3/AP_CM3.h>

extern AP_HAL::HAL& hal;

AP_CM3 *AP_CM3::_singleton;

AP_CM3::AP_CM3()
{
    if(_singleton != nullptr)
    {
        return;
    }
    _singleton = this;
}

void AP_CM3::send_cm3_message()
{
    hal.uartD->printf("message from FC\n");
    
}




namespace AP 
{
    AP_CM3 &cm3()
    {
        return *AP_CM3::get_singleton();
    }
};