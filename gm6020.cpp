#include "gm6020.h"
#include "mbed.h"
#include <chrono>
using namespace std::chrono;
#define DIGIT pow(10,2)//小数点表示用
gm6020::gm6020(RawCAN &can,int motor_num): _can(can),_motor_num(motor_num){
    _motor_max=25000;//電圧
    if(_motor_num<=8){
        _can.frequency(1000000); // CANのビットレートを指定
        _can.mode(CAN::Normal); // CANのモードをNormalに設定
    }
}

int gm6020::gm6020_send(int* motor) {//motorへ制御信号を送信する関数
    char _byte[_motor_num*2];//byteデータ変換用
    int _a=0;
    for(int i=0;i<_motor_num;i++){  //int dataを2byteに分割
        if(motor[i]>_motor_max)return 0;//入力値がmotor上限以上の場合return0
        _byte[_a++] = (char)(motor[i] >> 8); // int値の上位8ビットをcharに変換
        _byte[_a++] = (char)(motor[i] & 0xFF); // int値の下位8ビットをcharに変換
    }

    _canMessage.id = 0x1ff;//esc id1~4のcanの送信id//お前なんで0x300じゃないの
    _canMessage.len = 8;//can data長(8byte固定)
    _canMessage2.id = 0x2ff;//esc id5~8のcanの送信id
    _canMessage2.len = 8;
    _a = 0;
    int _i=0;
    for(int i=0;i<_motor_num;i++){//canmessageにbyte dataをセット
        if(i<4){
            _canMessage.data[_a] = _byte[_a];
            _a++;
            _canMessage.data[_a] = _byte[_a];
            _a++;
        }else{
            _canMessage2.data[_i++] = _byte[_a++];
            _canMessage2.data[_i++] = _byte[_a++];
        }
    }
    while(_a<15){//あまりのcanmassage.dataに0を代入(NULL値を残さない)
        if(_a<7){
            _canMessage.data[_a++] = 0;
            _canMessage.data[_a++] = 0;
        }else{
            _canMessage2.data[_a++] = 0;
            _canMessage2.data[_a++] = 0;
        } 
    }
    // CANメッセージの送信
    if (_can.write(_canMessage)/*&&_can.write(_canMessage2)*/) {
        return 1;
    }else{
        return -1;
    }

}

void gm6020::rbms_read(CANMessage &msg, short *rotation, short *speed) {
    _r = (msg.data[0] << 8) | (msg.data[1] & 0xff);
    now_pos = _r; 

    // ここで 0~360度 に変換されている
    _rotation = (float)_r * 360.0f / 8192.0f;
    *rotation = (short)_rotation;


            _speed = (msg.data[2] << 8) | (msg.data[3] & 0xff);
            if (_speed & 0b1000000000000000){//マイナス値の場合(最上位ビットが1のとき)(2の補数)
                _speed--;
                _speed = -~_speed;
            }
            *speed=_speed;

            _torque = (msg.data[4] << 8) | (msg.data[5] & 0xff);
            if (_torque & 0b1000000000000000){
                _torque--;
                _torque = -~_torque;
            }
            _temperature = msg.data[6];
}

void gm6020::can_read() {
    // while(true) を消して、if文にする
    if(_can.read(_msg)) {
        // IDが0x201〜0x208（モーターからの返信ID）なら解析する
        if(_msg.id >= 0x201 && _msg.id <= 0x208) {
            short r, s;
            rbms_read(_msg, &r, &s); // ここで now_pos が更新される
        }
    }
}

float gm6020::pid(float T,short deg_now, short set_deg,float *delta_deg_pre,float *ie,float KP, float KI,float KD )//pid制御
{
    float de;
    float delta_deg;
    delta_deg = (set_deg - deg_now + 540) % 360 - 180;
    //delta_deg  = set_deg - deg_now;
    de = (delta_deg - *delta_deg_pre)/T;
    *ie = *ie + (delta_deg + *delta_deg_pre)*T/2;
    float out_speed  = KP*delta_deg + KI*(*ie) + KD*de;
    *delta_deg_pre = delta_deg;
    //printf(">delta_deg:%d.%2d\n",(int)delta_deg,abs((int)(delta_deg*DIGIT-(int)delta_deg*DIGIT)));
    //printf(">deg_now:%d.%2d\n",(int)deg_now,abs((int)(deg_now*DIGIT-(int)deg_now*DIGIT)));
    // printf("%d.%2d    ",(int)vx,abs((int)(vx*DIGIT-(int)vx*DIGIT)));
    // printf("%d.%2d    \r\n",(int)x,abs((int)(x*DIGIT-(int)x*DIGIT)));
    //ThisThread::sleep_for(5ms);
    return out_speed;
}

void gm6020::deg_control(float* set_rad,int* motor,float* KP_GM6020,float* KI_GM6020,float* KD_GM6020){//速度制御用関数
    short rotation[_motor_num],speed[_motor_num];
    float delta_deg_pre[_motor_num],ie[_motor_num];
    Timer tm[_motor_num];//タイマーインスタンス生成(配列ではない)
    for(int i=0;i<_motor_num;i++){//初期化
        delta_deg_pre[i]=0.0;
        ie[i]=0.0;
        tm[i].start();
    }
    while(1){
        for(int id=0;id<_motor_num;id++){
            if(gm6020_can_msg.id==0x205+id){//esc idごとに受信データ割り振り
                CANMessage msg=gm6020_can_msg;
                rbms_read(msg,&rotation[id],&speed[id]);//data変換
                int deg = (int)(set_rad[id]/pi*180.0f);//rad→degに変換
                motor[id] = (int)pid(duration<float>(tm[id].elapsed_time()).count(),rotation[id],deg,&delta_deg_pre[id],&ie[id],KP_GM6020[id],KI_GM6020[id],KD_GM6020[id]);
                if(motor[id]>_motor_max){motor[id]=_motor_max;}else if(motor[id]<-_motor_max){motor[id]=-_motor_max;}//上限確認超えてた場合は上限値にset
                tm[id].reset();//timer reset
            }
        }
        ThisThread::sleep_for(1ms);
    }
}


void gm6020::pos_control(float *set_deg, int *output, float Kp, float Ki, float Kd) {
    // 1. 度数法(0~360)を 生データ(0~8191)に変換
    // 8192 / 360 = 約22.75
    int target_raw = (int)(set_deg[0] * 8192.0f / 360.0f);

    // 2. 現在の角度を取得 (0 ~ 8191)
    int current_pos = now_pos; 
    
    // 3. 偏差を計算
    int error = target_raw - current_pos;

    // 4. 最短距離計算 (1回転8192の半分4096で判定)
    if (error > 4096) error -= 8192;
    else if (error < -4096) error += 8192;

    // 5. PID計算
    static float integral = 0;
    static int prev_error = 0;
    
    integral += error;
    float derivative = error - prev_error;
    
    float out = (error * Kp) + (integral * Ki) + (derivative * Kd);
    
    // 出力リミッター
    if (out > 25000) out = 25000;
    if (out < -25000) out = -25000;
    
    output[0] = (int)out;
    prev_error = error;
}
