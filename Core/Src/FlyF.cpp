#include <bits/stdc++.h>
using namespace std;
#include "state_type.hpp"
#include "wrapper.hpp"
#include "stdio.h"
#include "math.h"
#include <cstring>

// EKF library
#include "attitude_ekf.h"

// PID library
#include "PID.h"
#include<cstdio>

// EKF instance
//AttitudeEKF_t* attitude_ekf = nullptr;

// センサーデータ
//float accel_data[3] = {0, 0, 0};  // m/s^2
//float gyro_data[3] = {0, 0, 0};   // deg/s

// タイミング制御
static uint32_t last_update_tick = 0;
static const uint32_t UPDATE_PERIOD_MS = 10;  // 10ms (100Hz)

void flyf(State* current_state, Context* context, int sbusdata9ch, int sbusdata3ch){
    context->count++;
    printf("Fly %d\n",context->count);
    // 10ms周期で実行
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed = current_tick - last_update_tick;

    if(elapsed < UPDATE_PERIOD_MS){

    	return;
    }

    last_update_tick += UPDATE_PERIOD_MS;
    if(current_tick - last_update_tick > UPDATE_PERIOD_MS){

    	last_update_tick = current_tick;
    }

    // ----- センサーデータ取得 -----
    // 使うセンサーのライブラリに合わせてください
    // float accel_data[3] 加速度[m/s^2]
    // float gyro_data[3] 角速度[dgree/s]


    // ----- EKF更新 -----
    float roll_deg = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg = 0.0f;
    if(attitude_ekf != nullptr){

    	// ジャイロをdeg/s -> rad/sに変換
    	float gyro_rad[3];
    	gyro_rad[0] = gyro_data[0] * M_PI / 180.0f;
    	gyro_rad[1] = gyro_data[1] * M_PI / 180.0f;
    	gyro_rad[2] = gyro_data[2] * M_PI / 180.0f;

    	// EKF更新
    	if(AttitudeEKF_Update(attitude_ekf, accel_data, gyro_rad)){

    		// オイラー角取得
    		float roll = AttitudeEKF_GetRoll(attitude_ekf);
    		float pitch = AttitudeEKF_GetPitch(attitude_ekf);
    		float yaw = AttitudeEKF_GetYaw(attitude_ekf);

    		// 結果表示（rad -> deg変換）
    		roll_deg = roll * 180.0f / M_PI;
    		pitch_deg = pitch * 180.0f / M_PI;
    		yaw_deg = yaw * 180.0f / M_PI;
   			printf("Roll: %+7.2f  Pitch: %+7.2f  Yaw: %+7.2f [deg]\n", roll_deg, pitch_deg, yaw_deg);
   		}
   	}
    //ここからPID
    //目標値の変換（中央1024を0度とし、傾きを最大30度にする計算）
    float roll_target = (sbusdata3ch - 1024) * 0.05f;
    float pitch_target = (sbusdata1ch - 1024) * 0.05f;
    float yaw_target = (sbusdata4ch - 1024) * 0.05f;
    roll_pid.calc(roll_target, roll_deg);
    pitch_pid.calc(pitch_target, pitch_deg);
    yaw_pid.calc(yaw_target, yaw_deg);
    float roll_u = roll_pid.getData();
    float pitch_u = pitch_pid.getData();
    float yaw_u = yaw_pid.getData();
    if(sbusdata9ch < 1000){
        	*current_state = State::Dis;
    }
}
