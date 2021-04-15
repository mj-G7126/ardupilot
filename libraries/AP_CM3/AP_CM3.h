#pragma once

#include <AP_HAL/AP_HAL.h>

class AP_CM3
{
    public:
    AP_CM3();
    static AP_CM3 *get_singleton()
    {
        return _singleton;
    }


    void send_cm3_message();


    private:
    static AP_CM3* _singleton;


};


namespace AP 
{
    AP_CM3 &cm3();
};

