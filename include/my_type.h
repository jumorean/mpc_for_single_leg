#pragma once


#include <ethercat.h>



namespace elmo{

namespace op_mode{

class Profile{
public:
    static const char positon = 1;
    static const char velocity = 3;
    static const char torque = 4;
};

class CyclicSynchronous{
public:
    static const char position = 8;
    static const char velocity = 9;
    static const char torque = 10;
};

typedef CyclicSynchronous CS;
static const char none_mode = 0;
//typedef Profile P;


}



}

