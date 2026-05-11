#include "mbed.h"
#include "gm6020.h"

UnbufferedSerial pc(USBTX, USBRX, 921600);

 DigitalOut leds[4]{
     DigitalOut(LED1),
     DigitalOut(PC_1),
     DigitalOut(PC_2),
     DigitalOut(PC_3),
 };

enum wheel_number{
    FR,
    FL,
    BR,
    BL,
    AWS,
    ALL_WHEEL
};

RawCAN can_motor(PA_11, PA_12, 1000000);
CAN can_signal(PA_11, PA_12, 1000000);

gm6020 wheel(can_motor, 1);

#define MAINLOOP_RATE 1ms
#define DT 0.001f

// ===== PIDゲイン =====
float kp = 160.0f;
float ki = 18.7f;
float kd = 1.77f;

// ===== PID変数 =====
float target_deg = 0.0f;

float integral = 0.0f;
float prev_error = 0.0f;

int motor_output[ALL_WHEEL] = {0};

int main()
{
    printf("start\r\n");

     for(auto &i : leds) i = false;
     leds[0] = true;

    bool init = false;

    while (true) {

        // CAN受信
        wheel.can_read();

        // 現在角度取得
        float current_deg = wheel.get_rotation();

        // 起動時の目標角度を現在位置にする
        if(!init){
            target_deg = current_deg;
            init = true;
        }

        // ===== キーボード入力 =====
        if(pc.readable()){

            char key;
            pc.read(&key, 1);

            switch(key){

                case '0':
                    target_deg = 0;
                    break;

                case '1':
                    target_deg = 90.0f;
                    break;

                case '2':
                    target_deg += 15.0f;
                    break;

                case '3':
                    target_deg -= 15.0f;
                    break;

                case 'q':
                    target_deg = current_deg;
                    integral = 0;
                    break;
            }
        }

        // ===== PID =====

        float error = target_deg - current_deg;

        // 角度ラップ処理
        while(error > 180) error -= 360;
        while(error < -180) error += 360;

        // 積分
        integral += error * DT;

        // 積分制限
        if(integral > 1000) integral = 1000;
        if(integral < -1000) integral = -1000;

        // 微分
        float derivative = (error - prev_error) / DT;

        // 微分制限
        if(derivative > 1000) derivative = 1000;
        if(derivative < -1000) derivative = -1000;

        // PID計算
        float output =
            kp * error +
            ki * integral +
            kd * derivative;

        prev_error = error;

        // 出力制限
        if(output > 5000) output = 5000;
        if(output < -5000) output = -5000;

        // モータ出力
        motor_output[0] = (int)output;

        // CAN送信
        wheel.gm6020_send(motor_output);

        // デバッグ表示
        printf(
            "target:%6.2f current:%6.2f error:%6.2f out:%6d\r\n",
            target_deg,
            current_deg,
            error,
            motor_output[0]
        );

/*
        // Teleplot用
        printf(
            ">target:%f\n>current:%f\n>output:%f\n",
            target_deg,
            current_deg,
            output
        );
        */

        //leds[3] = !leds[3];

        ThisThread::sleep_for(MAINLOOP_RATE);
    }
}
