#include <iostream>
#include <iomanip>
#include <stdlib.h>
#include "opencv4/opencv2/opencv.hpp"
#include <thread>
#include <cmath>
#include "main.hpp"
//敌方能量块
#define ENEMY_BLOCK_ID 2
//我方能量块
// 蓝色为1，黄色为2
#define SELF_ENERGY_BLOCK_ID 1 //
//公共能量块
#define PUBLIC_ENERGY_BLOCK_ID 0

extern Object enemy;
extern Object self_energy;
extern Object enemy_energy;
extern Object public_energy;

extern "C"
{
#include "apriltag/apriltag.h"
#include "apriltag/tag36h11.h"
#include "apriltag/tag25h9.h"
#include "apriltag/tag16h5.h"
#include "apriltag/tagCircle21h7.h"
#include "apriltag/tagCircle49h12.h"
#include "apriltag/tagCustom48h12.h"
#include "apriltag/tagStandard41h12.h"
#include "apriltag/tagStandard52h13.h"
#include "apriltag/common/getopt.h"
}

using namespace std;
using namespace cv;

int detect_Apriltag(void);

//多线程开启Apriltag码检测
int start_ApriltagDetection();