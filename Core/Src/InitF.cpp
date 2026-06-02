#include <bits/stdc++.h>
using namespace std;
#include "state_type.hpp"
#include "wrapper.hpp"
#include <stdio.h>

void initf(State* current_state, Context* context){
    context->count++;
    printf("Init %d\n",context->count);
    *current_state = State::Calib;
}
