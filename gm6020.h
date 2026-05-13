#ifndef INCLUDED_gm6020_H
#define INCLUDED_gm6020_H

#include "mbed.h"
#include "CANManager.h"
#include <vector>

enum ControlMode {
    ANGLE,
    SPEED,
    TRACON
};

class gm6020 : public CANReceiver {
    public:
        CANMessage gm6020_can_msg;
        gm6020(RawCAN &can,int motor_num);
        int gm6020_send(int* moter);
        void rbms_read(CANMessage &msg, short *rotation,short *speed);
        bool handle_message(const CANMessage &msg) override;
        void can_read(){}
        const vector<int>& getSumDeg() const { return _sum_deg; }
        float pid(float T,short rpm_now, short set_speed,float *delta_rpm_pre,float *ie,float KP=150,float KI=70, float KD=0);
        void deg_control(float* set_deg,int* motor,float* KP_GM6020,float* KI_GM6020,float* KD_GM6020);
        void pos_control(float *set_deg, int *output, float Kp, float Ki, float Kd);
        float get_rotation() { return _rotation; }
        int _rotation;
        int _delta_deg;
        int now_pos;
    private:
        static constexpr float pi = 3.1415926535;
        CANMessage _canMessage,_canMessage2,_msg;
        RawCAN &_can;
        vector<int> _sum_deg;
        int _motor_num,_motor_max;
        unsigned short _r;
        int _speed;
        int _torque;
        int _temperature;
        int set_deg;
        
        


};


#endif
