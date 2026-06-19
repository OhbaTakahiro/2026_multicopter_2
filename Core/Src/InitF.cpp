#include <bits/stdc++.h>
using namespace std;
#include "state_type.hpp"
#include "wrapper.hpp"
#include <stdio.h>
#include "tim.h"
#include "motor_controller.hpp"

// グローバルモーターコントローラーポインタ
MotorController* motor_controller = nullptr;

void initf(State* current_state, Context* context){
	// インスタンスの作成
	motor_controller = new MotorController(&htim1, TIM_CHANNEL_1, MotorMode::NORMAL);

	// 初期化の確認
	if (motor_controller->isInitialized()) {

	    printf("MotorController (NORMAL mode) initialized successfully\n");
	}
	else {

	    printf("MotorController initialization failed\n");
	}

	// パルス幅の範囲を設定（1000us ~ 2000us）
	motor_controller->setPulseRange(1000, 2000);
	printf("Pulse range set to 1000us ~ 2000us\n");

	// 0%に設定（初期値を送信）
	motor_controller->setSpeed(0.0f);
	printf("Motor speed: 0 %%\n");
	HAL_Delay(10000);
    context->count++;
    printf("Init %d\n",context->count);
    *current_state = State::Calib;
}
