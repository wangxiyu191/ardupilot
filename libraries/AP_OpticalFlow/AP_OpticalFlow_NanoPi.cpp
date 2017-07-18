//
// Created by wxy on 7/14/17.
//

#include "AP_OpticalFlow_NanoPi.h"
#include "AP_SerialManager/AP_SerialManager.h"
#include <ctype.h>
#include <stdio.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/edc.h>
#include <AP_AHRS/AP_AHRS.h>
#include <utility>

extern const AP_HAL::HAL& hal;

AP_OpticalFlow_NanoPi::AP_OpticalFlow_NanoPi(OpticalFlow &_frontend) : OpticalFlow_backend(_frontend) {
    _uart = hal.uartC;
}

void AP_OpticalFlow_NanoPi::init(void)
{

}

AP_OpticalFlow_NanoPi *AP_OpticalFlow_NanoPi::detect(OpticalFlow &_frontend) {
    AP_OpticalFlow_NanoPi  *sensor = new AP_OpticalFlow_NanoPi(_frontend);

    if (!sensor->setup_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_OpticalFlow_NanoPi::hand_shake(void){
    hal.console->printf("Optical Flow handshake\n");
    _uart->println("HELLO\n");
    _uart->flush();
    uint32_t begin_ms = AP_HAL::millis();
    uint32_t nbytes = _uart->available();
    while(1) {
        while (nbytes-- > 0) {
            char c = (char) _uart->read();
            if (c == '\n') {
                receive_buf[receive_buflen] = 0;
                bool result = strcasecmp("HELLO", receive_buf) == 0;
                receive_buflen = 0;
                if (result) {
                    printf("Found NanoPi\n");
                }
                return true;
            } else if (isalpha(c)) {
                receive_buf[receive_buflen++] = c;
                if (receive_buflen == sizeof(receive_buf)) {
                    receive_buflen = 0;
                }
            } else {
                break;
            }
        }
        if( AP_HAL::millis() - begin_ms > 10){
            break;
        }
        nbytes = _uart->available();
    }
    return false;
}

bool AP_OpticalFlow_NanoPi::setup_sensor(void) {
    _uart->begin(115200);
    _uart->set_flow_control(_uart->FLOW_CONTROL_DISABLE);
    hal.console->printf("uart init %d:\n",_uart->is_initialized());

    init();
    return hand_shake();
}

long long  AP_OpticalFlow_NanoPi::read_long(void){
    long long  result = -1;

    uint32_t begin_ms = AP_HAL::millis();

    uint32_t nbytes = _uart->available();
    receive_buflen = 0;
    while(1){
        while (nbytes-- > 0){
            char c = (char)_uart->read();

            if(c == '\n'){
                receive_buf[receive_buflen]=0;
                result = atol(receive_buf);
                receive_buflen=0;

                return result;
            }else if (isdigit(c)||c == '-') {
                receive_buf[receive_buflen++]=c;
                if(receive_buflen == sizeof(receive_buf)){
                    receive_buflen = 0;
                }
            }else{
                return result ;
            }
        }
        if( AP_HAL::millis() - begin_ms > 10){
            break;
        }
        nbytes = _uart->available();
    }
    return result ;


}

// update - read latest values from sensor and fill in x,y and totals.
void AP_OpticalFlow_NanoPi::update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - last_update_ms < 60) {
        return;
    }

    uint32_t dt_ms = last_update_ms - now ;
    last_update_ms = now;

    const Vector3f &gyro = get_ahrs().get_gyro();

    _uart->printf("DATA\n");
    long long detla_x = -read_long();
    long long  detla_y = -read_long();
    long long  time_us = read_long();

    struct OpticalFlow::OpticalFlow_state state;
    state.device_id = 1;
    state.surface_quality = 128;
    //hal.console->printf("x = %lld y= %lld\n" ,detla_x ,detla_y);
    if(detla_x != -1 && detla_y != -1){
        const Vector2f flowScaler = _flowScaler();
        float flowScaleFactorX = 1.0f + 0.001f * flowScaler.x;
        float flowScaleFactorY = 1.0f + 0.001f * flowScaler.y;
        float dt = time_us * 1.0e-6;

        state.flowRate = Vector2f(detla_x * flowScaleFactorX,
                                  detla_y * flowScaleFactorY);
        state.flowRate *= flow_pixel_scaling / dt;



        state.bodyRate = Vector2f(gyro.x, gyro.y) ;

        _applyYaw(state.flowRate);


    }else{
        state.flowRate.zero();
        state.bodyRate.zero();
    }

    _update_frontend(state);



}


