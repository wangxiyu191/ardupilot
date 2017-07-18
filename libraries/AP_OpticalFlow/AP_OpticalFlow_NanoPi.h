//
// Created by wxy on 7/14/17.
//
#pragma once

#include "OpticalFlow.h"
class AP_OpticalFlow_NanoPi : public OpticalFlow_backend {
public:
    /// constructor
    AP_OpticalFlow_NanoPi(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init() override ;

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_NanoPi *detect(OpticalFlow &_frontend);



private:
    AP_HAL::UARTDriver *_uart;
    const float flow_pixel_scaling = 4.41e-8;

    bool setup_sensor(void);
    char receive_buf[100];
    uint8_t receive_buflen=0;

    void timer(void);

    bool hand_shake(void);

    uint32_t last_update_ms;

    long long int read_long(void);
};
