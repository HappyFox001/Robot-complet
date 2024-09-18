#include <string>
#include <opencv2/opencv.hpp>
#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif
#include <thread>
#include <cmath>
#include <vector>
#include <algorithm>
#include <iostream>
#include <thread>

#include "main.hpp"
extern Object enemy;
extern Object self_energy;
extern Object enemy_energy;
extern Object public_energy;


int lidar_start() ;

int lidarThreadStart();