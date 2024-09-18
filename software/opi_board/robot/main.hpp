#ifndef MAIN_HPP
#define MAIN_HPP

#include <math.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <thread>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#define SERIAL_PORT "/dev/ttyACM0"
#define BAUD_RATE 921600
// #define TOF_PORT "/dev/ttyS0"
// #define BAUD_RATE 921600
typedef enum Direction // 确定小车当前行驶状态
{                      // 八个方向+不确定
    FORWARD_LEFT = 3,
    FORWARD = 1,
    FORWARD_RIGHT = 2,
    LEFT = 5,
    RIGHT = 4,
    BACKWARD_LEFT = 7,
    BACKWARD = 8,
    BACKWARD_RIGHT = 6,
    UNSURE = 0,
} Direction;
typedef enum Object_Type // 确定小车当前目标 //前右2 前左3 后右1 后左4
{
    // 敌人
    ENEMY = 1,
    // 己方能量
    SELF_ENERGY = 2,
    // 敌方能量
    ENEMY_ENERGY = 3,
    // 公共能量
    PUBLIC_ENERGY = 4
} Object_Type;
typedef struct CAR // 确定小车整体
{
    enum Direction avoid_dir;
    int is_OnStage;
    int act_Flag;
} Car;
typedef struct _Object // 目标物体相对位置
{
    // 是否
    int isDetected;
    // 角度
    float angle;
    // 距离
    float distance;
    // 类型
    enum Object_Type type;
} Object;

#endif // MAIN_HPP
