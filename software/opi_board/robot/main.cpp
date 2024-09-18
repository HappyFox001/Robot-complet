#include "radar.hpp"
#include "apriltagDetect.hpp"
#include "main.hpp"
#include <wiringPi.h>
#include <wiringSerial.h>
#include "tof/nlink_linktrack_nodeframe0.h"
#include "tof/nlink_linktrack_nodeframe1.h"
#include "tof/nlink_tofsense_frame0.h"
#include "tof/nlink_tofsensem_frame0.h"
#include "tof/nlink_utils.h"

#define THRESHOLD 70 // 灰度阈值
Object enemy = {0, 0, 0, ENEMY};
Object self_energy = {0, 0, 0, SELF_ENERGY}; // 400台上快速度；200台上慢速度；700冲台速度
Object enemy_energy = {0, 0, 0, ENEMY_ENERGY};
Object public_energy = {0, 0, 0, PUBLIC_ENERGY};
Car car = {UNSURE, 0, 0};
unsigned char pkgzero[] = {
    0xFF,
    0x55,
    0x00,
    0x00,
    0x00,
    0x00,
    0xC3};
enum Direction lastedge = UNSURE;
int start_sw[2] = {1, 1};            // 开关
int gray[4] = {0, 0, 0, 0};          // 灰度传感器数 0:后左；3：后右；2：右前；1：左前
int ir[4] = {0, 0, 0, 0};            // 红外开关
float ir_distance[4] = {0, 0, 0, 0}; // 4个TOF 前面是二号和三号
short speed_left;                    // 左轮速度，矢量 左轮为实际右轮0 设置为-1000到1000
short speed_right;                   // 右轮速度，矢量
short last_speedL = 0;
short last_speedR = 0;
int if_go = 0;
int counts = 0;
void Get_Sensor_Data(void);       // 调用所有数据获取接口
void Start(void);                 // 等待启动
void Get_Opi_Uart_Data(void);     // 获取opi上位机返回的物体识别结果，更新enemy、self_energy、enemy_energy、public_energy
void Get_TOF(void);               // 获取TOF数据
void State_Judge(void);           // 状态判断
void Go_Up_Stage(void);           // 上台
void Dir_Adjust(void);            // 调整车身方向
void Attack(void);                // 攻击
Direction Detect_Edge(void);      // 车辆边缘状态
void Avoid(Direction avoid_dir);  // 避开边缘
void Track_Target_bycamera(void); // 根据相机目标跟踪
void Track_Target_byradar(void);  // 根据雷达跟踪目标
void Roam(void);                  // 漫游
void if_on_stage(void);           // 判断十分在台上
void go_stage();
void stop();
unsigned char query[] = {0x57, 0x10, 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x63};
unsigned char rx_buf[16] = {0};
int fd_tof;
int fd_stm;
unsigned char response[16] = {0};
float object_angle[2] = {361.0, 361.0};
float object_distance[2] = {-1.0, 1.0};
int select_obeject = 0;
int tof_init(const char *device, const int baud)
{

    fd_tof = serialOpen("/dev/ttyACM0", 921600); // 打开串口
    if (fcntl(fd_tof, F_SETFL, FNDELAY) == -1)
    {
        printf("set nodelay mode error");
    }
    if (fd_tof == -1)
    { // 如果打开串口失败则退出程序
        printf("  serialOpen tof failed!\n");
        return 0;
    }
    serialFlush(fd_tof);
    return 1;
}

int tof_read(int start_id, int end_id)
{
    for (int i = start_id; i < end_id; i++)
    {
        // 发送查询指令
        query[4] = i;
        query[7] = 0x63 + i;
        memset(response, 0, sizeof(response));

        write(fd_tof, query, sizeof(query));
        delay(5);
        read(fd_tof, response, sizeof(response));

        // delay(5);

        // printf("Sent query: ");
        // for (int i = 0; i < sizeof(query); i++)
        // {
        //     printf("%02X ", query[i]);
        // }
        // printf("\n");
        // printf("rev data: ");
        // for (int i = 0; i < sizeof(response); i++)
        // {
        //     printf("%02X ", response[i]);
        // }

        // 解析数据
        if (g_nts_frame0.UnpackData(response, 16))
        {
            // printf("i:%d  id:%d, distance:%f\r\n", i, g_nts_frame0.result.id, g_nts_frame0.result.dis);
            ir_distance[i] = g_nts_frame0.result.dis;
        }
        else
        {
            // printf("TOFSense Frame0 data unpack faild:\r\n");
        }
    }

    return 0;
}

int uart_stm_init(const char *device, const int baud)
{

    fd_stm = serialOpen(device, baud); // 打开串口

    // if (fcntl(fd_stm, F_SETFL, FNDELAY) == -1)
    // {
    //     printf("set nodelay mode error");
    // }
    if (fd_stm == -1)
    { // 如果打开串口失败则退出程序
        printf("  serialOpen stm32 failed!\n");
        return 0;
    }
    serialFlush(fd_stm);
    return 1;
}
void uart_stm_read(void)
{
    tcflush(fd_stm, TCIFLUSH);
    delay(100);
    read(fd_stm, rx_buf, sizeof(rx_buf));

    for (int i = 0; i < 16; i++)
    {
        if (rx_buf[i] == 0xFF && rx_buf[i + 1] == 0x55 && rx_buf[i + 7] == 0xC3)
        {
            start_sw[0] = (rx_buf[2] & 0x10) >> 4;
            start_sw[1] = (rx_buf[2] & 0x20) >> 5;
            ir[0] = rx_buf[2] & 0x01;
            ir[1] = (rx_buf[2] & 0x02) >> 1;
            ir[2] = (rx_buf[2] & 0x04) >> 2;
            ir[3] = (rx_buf[2] & 0x08) >> 3;
            gray[0] = rx_buf[3];
            gray[1] = rx_buf[4];
            gray[2] = rx_buf[5];
            gray[3] = rx_buf[6];
            break;
        }
    }
}

void uart_stm_send(void)
{
    // printf("data send\n");
    // speed_right = -speed_right * 1.2;
    // if (speed_left * last_speedL < 0 || speed_right * last_speedR < 0)
    // {
    //     tcflush(fd_stm, TCIFLUSH); // 清空缓冲区
    //     delay(50);
    //     write(fd_stm, pkgzero, sizeof(pkgzero));
    //     delay(50);
    // }
    // last_speedL = speed_left;
    // last_speedR = speed_right;
    unsigned char pkg[] = {
        0xFF,
        0x55,
        (unsigned char)((speed_left >> 8) & 0xff),
        (unsigned char)(speed_left & 0xff),
        (unsigned char)((speed_right >> 8) & 0xff),
        (unsigned char)(speed_right & 0xff),
        0xC3};
    tcflush(fd_stm, TCIFLUSH); // 清空缓冲区
    delay(50);
    write(fd_stm, pkg, sizeof(pkg));
    // printf("send_end\n");
}

void Get_Sensor_Data(void)
{
    uart_stm_read();
    tof_read(1, 4);
}

void Start(void)
{
    while (start_sw[0]) // 如果两个启动都按下
    {
        printf("等待启动\n");
        printf("start_sw[0]:%d\n", start_sw[0]);
        uart_stm_send();
        Get_Sensor_Data();
    }
    speed_left = -800;
    speed_right = -800;
    // while (!caris_OnStage)
    // {
    //     uart_stm_send();
    //     printf("冲刺上台中\n");
    //     Get_Sensor_Data();
    //     if_on_stage();
    // }
    uart_stm_send();
    delay(800);
    speed_left = 0;
    speed_right = 0;
    uart_stm_send();
    delay(50);
    speed_left = -400;
    speed_right = -100;
    uart_stm_send();
    delay(1000);
}

void Go_Up_Stage(void) // 冲刺上台
{

    speed_left = 100;
    speed_right = 100; // 设置一个较大的速度
    uart_stm_send();
    while (car.is_OnStage != 1)
    {
        Get_Sensor_Data(); // 更新包括可能的平台传感器在内的传感器数据
        if_on_stage();
    }
}
void if_on_stage(void)
{
    if (ir[0] && ir[1] && ir[2] && ir[3])
    {
        car.is_OnStage = 0; //    车子在台下
    }
    else
    {
        car.is_OnStage = 1;
    }
}
void Dir_Adjust(void) // 台下旋转正对平台
{
    int attempts = 0; // 添加尝试次数限制以避免可能的死循环
    do
    {
        Get_Sensor_Data(); // 更新红外距离传感器读数
        if (abs(ir_distance[2] - ir_distance[3]) > 0.1)
        {
            if (ir_distance[2] > ir_distance[3])
            {
                speed_left = 10; // 左边距离较远，微调向左转
                speed_right = 15;
            }
            else
            {
                speed_left = 15; // 右边距离较远，微调向右转
                speed_right = 10;
            }
            uart_stm_send();
        }
        else
        {
            // 正对平台，准备上台
            break;
        }
        attempts++;
    } while (attempts < 10); // 避免无限循环，限制调整次数
}
Direction Detect_Edge(void)
{
    //

    int front_left_ir = ir[2];
    int front_right_ir = ir[1];
    int rear_left_ir = ir[3];
    int rear_right_ir = ir[0];
    if (!(front_left_ir || front_right_ir || rear_left_ir || rear_right_ir))
    {
        return UNSURE;
    }
    else if (front_left_ir && front_right_ir)
    {
        return FORWARD; // 前方出圈
    }
    else if (rear_left_ir && rear_right_ir)
    {
        return BACKWARD; // 后方出圈
    }
    else if (front_left_ir && rear_left_ir)
    {
        return LEFT;
    }
    else if (rear_right_ir && rear_left_ir)
    {
        return RIGHT; // 右侧出圈
    }
    else if (front_left_ir)
    {
        return FORWARD_LEFT; // 前左出圈
    }
    else if (front_right_ir)
    {
        return FORWARD_RIGHT; // 前右出圈
    }
    else if (rear_left_ir)
    {
        return BACKWARD_LEFT; // 后左出圈
    }
    else if (rear_right_ir)
    {
        return BACKWARD_RIGHT; // 后右出圈
    }
    return UNSURE;
}
void Avoid(Direction dir)
{
    switch (dir)
    {
        // 1
    case FORWARD:
        printf("前方出线\n");
        stop();
        speed_left = -100; // 后退
        speed_right = -100;
        break;
        // 2
    case FORWARD_RIGHT:
        printf("前右出线\n");
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        delay(25);
        speed_left = -200; // 左轮后退更慢，右轮后退快，向左转
        speed_right = -100;
        break;
        // 3
    case FORWARD_LEFT:
        printf("前左出线\n");
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        delay(25);
        speed_left = -100; // 右轮后退更慢，左轮后退快，向右转
        speed_right = -200;
        break;
        // 4
    case RIGHT:
        printf("右方出线\n");
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        delay(25);
        speed_left = 500; // 左轮前进，右轮后退，原地向左旋转
        speed_right = -500;
        break;
        // 5
    case LEFT:
        printf("左方出线\n");
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        delay(25);
        speed_left = -500; // 右轮前进，左轮后退，原地向右旋转
        speed_right = 500;
        break;
        // 6
    case BACKWARD_RIGHT:
        printf("后右出线\n");
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        delay(25);
        speed_left = 200; // 左轮前进更快，右轮前进慢，向左前进
        speed_right = 100;
        break;
        // 7
    case BACKWARD_LEFT:
        printf("后左出线\n");
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        delay(25);
        speed_left = 100; // 右轮前进更快，左轮前进慢，向右前进
        speed_right = 200;
        break; // 8
    case BACKWARD:
        printf("后方出线\n");
        stop();
        speed_left = 100; // 直接前进
        speed_right = 100;
        break;
        // 0
    case UNSURE: // 不确定，保持不变
        printf("状态不变\n");
        break;

    default:
        break;
    }
}
// 目标跟踪，函数效果：设置电机目标速度和转动方向
void Track_Target_bycamera(void)
{
    if (fabsf(ir_distance[0] - ir_distance[1]) < 0.4)
    {
        speed_left = 1000;
        speed_right = 1000;
        return;
    }
    float angle;

    if (enemy_energy.isDetected)
    {
        angle = enemy_energy.angle;
    }
    else if (public_energy.isDetected)
    {
        angle = public_energy.angle;
    }
    else
    {
        return;
    }
    // 机器人基本速度和最大速度
    int baseSpeed = 30.0;
    // 计算左右轮的速度，基于敌人的角度
    int leftMotorSpeed, rightMotorSpeed;

    if (angle > 0)
    {
        // 敌人在右侧，减小右轮速度，增加左轮速度
        leftMotorSpeed = baseSpeed;
        rightMotorSpeed = -baseSpeed;
    }
    else
    {
        // 敌人在左侧，减小左轮速度，增加右轮速度
        leftMotorSpeed = -baseSpeed;
        rightMotorSpeed = baseSpeed;
    }

    // 设置电机的目标速度和方向
    speed_left = leftMotorSpeed;
    speed_right = rightMotorSpeed;
}
void Track_Target_byradar(void)
{
    // if (self_energy.isDetected)
    // {
    //     select_obeject = select_obeject == 0 ? 1 : 0;
    // }
    // 机器人基本速度和最大速度
    int baseSpeed = 100;
    // 计算左右轮的速度，基于敌人的角度
    float angle;
    float distance;
    angle = object_angle[select_obeject];
    distance = object_distance[select_obeject];

    printf("雷达角度：%3f\n", angle);
    if (distance == -1 || angle == 361.0)
    {
        printf("雷达识别失败\n");
        speed_left = 50;
        speed_right = 50;
    }
    else
    {
    
        if (angle < 10 && angle > -10)
        {
            printf("猛冲\n");
            speed_left = 150;
            speed_right = 150;
            return;
        }
       else  if (angle < 10 && angle > -10 && !self_energy.isDetected)
        {
            printf("直行\n");
            speed_left = 150;
            speed_right = 150;
            return;
        }
        else if (angle > 0 && angle < 20)
        {
            printf("左转\n");
            speed_left = 300;
            speed_right = 150;
        }
        else if (angle < 0 && angle > -20)
        {
            printf("右转\n");
            speed_left = 150;
            speed_right = 300;
        }
        else if (angle < -20)
        {
            printf("右大转\n");
            // 敌人在右侧，减小右轮速度，增加左轮速度
            speed_left = -800;
            speed_right = 800;
        }
        else
        {
            printf("左大转\n");
            // 敌人在左侧，减小左轮速度，增加右轮速度
            speed_left = 800;
            speed_right = -800;
        }
    }
}

// 漫游（暂时转圈圈吧），函数效果：设置电机目标速度和转动方向
void Roam(void)
{
    // 设定电机速度
    float speed = 30.0; // 设定一个基础速度

    // 设置左右轮的速度，其中一个轮子的速度稍高，另一个稍低，以产生旋转效果
    // 根据你的机器人设计，可能需要调整这个速度差来优化旋转的平滑度和半径
    speed_left = speed;        // 左轮以基础速度运行
    speed_right = speed * 0.8; // 右轮以稍低的速度运行，造成向左旋转

    uart_stm_send();
}
void State_Judge(void)
{
    printf("State_JUdge\n");
    enum Direction edge = Detect_Edge();

    // 如果检测到边缘
    printf("edge: %d\n", edge);
    if (edge != 0)
    {
        Avoid(edge);
        if_go = 1;
        // printf("边缘避障\n");
        // if(edge==lastedge)
        // {
        //     counts+=1;
        // }
        // else{
        //     counts=1;
        // }
        // if(counts>=3)
        // {
        //     Avoid(edge);
        // }
    }
    // 如果检测到敌人
    // else if (enemy_energy.isDetected || public_energy.isDetected)
    // {
    //     printf("相机识别\n");
    //     Track_Target_bycamera();
    // }
    // else if (!self_energy.isDetected && ir_distance[2] < 0.5 && ir_distance[3] < 0.5 && ir_distance[2] > 0.1 && ir_distance[3] > 0.1)
    // {
    //     printf("前方冲撞\n");
    //     printf("距物块：%2f %2f\n", ir_distance[2], ir_distance[3]);
    //     speed_left = 200;
    //     speed_right = 200;
    // }
    // else if (!self_energy.isDetected && ir_distance[2] < 0.5 && ir_distance[2] > 0.1)
    // {
    //     printf("左方冲撞\n");
    //     printf("距物块：%2f\n", ir_distance[2]);
    //     speed_left = 300;
    //     speed_right = 200;
    // }
    // else if (!self_energy.isDetected && ir_distance[3] < 0.5 && ir_distance[3] > 0.1)
    // {
    //     printf("右方冲撞\n");
    //     printf("距物块：%2f\n", ir_distance[3]);
    //     speed_left = 200;
    //     speed_right = 300;
    // }
    else
    {
        if (if_go == 1)
        {
            speed_left = 0;
            speed_right = 0;
            uart_stm_send();
            delay(25);
            speed_left = 800;
            speed_right = -800;
            uart_stm_send();
            delay(500);
            if_go = 0;
        }
        printf("Track_Target_byradar\n");
        Track_Target_byradar();
    }
    lastedge = edge;
}

int Car_Init()
{
    printf("Car_init\n");
    wiringPiSetup();
    uart_stm_init("/dev/ttyS0", 115200);
    printf("/dev/ttyS0 opened\n");
    tof_init("/dev/ttyACM0", 921600);
    printf("/dev/ttyACM0 opened\n");
    return 0;
}
void wheel_test()
{
    speed_left = 300;
    speed_right = 300;
    uart_stm_send();
    delay(5000);
    speed_left = 0;
    speed_right = 0;
    uart_stm_send();
    delay(5000);
    speed_left = -300;
    speed_right = -300;
    uart_stm_send();
    delay(5000);
}
void uart_test()
{
    uart_stm_send();
    Get_Sensor_Data();
    for (int i = 0; i < 16; i++)
    {
        printf("%.2X ", rx_buf[i]);
    }
    printf("\n");
    for (int i = 0; i < 16; i++)
    {
        printf("%.2X ", response[i]);
    }
    Avoid(Detect_Edge());
}
void tof_test()
{
    printf("distance:");
    tof_read(0, 4);
    printf("\n");
    delay(100);
}
void turning()
{
    speed_left = 600;
    speed_right = 600;
    uart_stm_send();
    delay(5000);
    speed_left = 0;
    speed_right = 0;
    uart_stm_send();
    delay(5000);
    speed_left = -600;
    speed_right = -600;
    uart_stm_send();
    delay(5000);
}
void Big_test()
{
    speed_left = 10;
    speed_right = 10;
    uart_stm_send();
    Get_Sensor_Data();
    // Avoid(Detect_Edge());
    printf("\n");
    printf("Switch:\n");
    printf("%d\n", start_sw[0]);
    printf("%d\n", start_sw[1]);
    printf("Gray:\n");
    for (int i = 0; i < 4; i++)
    {
        printf("%d ", gray[i]);
    }
    printf("\n");
    printf("IR:\n");
    for (int i = 0; i < 4; i++)
    {
        printf("%d ", ir[i]);
    };
    printf("\n");
    printf("IR_DISTANCE:\n");
    for (int i = 0; i < 4; i++)
    {
        printf("%.2f ", ir_distance[i]);
    }
    printf("\n");
    printf("Ladar:\n");
    for (int i = 0; i < 2; i++)
    {
        printf("%.2f ", object_angle[i]);
    }
    printf("\n");
    printf("Camera");
    printf("enemy_energy: ");
    printf("%.2f ", enemy_energy.angle);
    printf("\n");
    printf("public_energy: ");
    printf("%.2f ", public_energy.angle);
    printf("\n");
}
void stop()
{
    speed_left = -400;
    speed_right = -400;
    uart_stm_send();
    delay(200);
    speed_left = 0;
    speed_right = 0;
    uart_stm_send();
}
void normal_model()
{
   Start();
    while (1)
    {
        car.is_OnStage=1;
        speed_left = 0;
        speed_right = 0;
        uart_stm_send();
        Get_Sensor_Data();
        if_on_stage();
        delay(1000);
        while (car.is_OnStage)
        {
            printf("台上\n");
            uart_stm_send();
            Get_Sensor_Data();
            if_on_stage();
            printf("红外开关：%d %d %d %d\n", ir[0], ir[1], ir[2], ir[3]);
            State_Judge();
            delay(25);
        }
        while (!car.is_OnStage)
        {
            printf("台下\n");
            go_stage();
            delay(25);
        }
    }
}
void test_radar()
{
    printf("radar:\n");
    printf("%3f %3f", object_angle[0], object_angle[1]);
    printf("\n");
}
void text_wheel_left_forward()
{
    speed_left = 50;
    uart_stm_send();
    delay(1000);
}
void text_wheel_left_Backward()
{
    speed_left = -50;
    uart_stm_send();
    delay(1000);
}
void text_wheel_right_forward()
{
    speed_right = 50;
    uart_stm_send();
    delay(1000);
}
void go_stage()
{
    speed_left = 100;
    speed_right = 100;
    uart_stm_send();
    delay(1000);
    speed_left = -800;
    speed_right = -800;
    uart_stm_send();
    delay(800);
    speed_left = 0;
    speed_right = 0;
    uart_stm_send();
    Get_Sensor_Data();
    delay(1000);
    uart_stm_send();
    Get_Sensor_Data();
    if_on_stage();
}
int main(void)
{
    Car_Init();
    // speed_left =-100;
    // speed_right=-100;
    // uart_stm_send();
    // std::thread ApriltagThread(detect_Apriltag);
    delay(5000);
    std::thread lidarThread(lidar_start);

    delay(10000);
    //  // speed_left=-200;
    //  // speed_right=-200;
    //  // uart_stm_send();
    //  // delay(1000);
    //  // speed_left=0;
    //  // speed_right=0;
    //  // uart_stm_send();
    normal_model();
    //detect_Aprilta();
    //  speed_left=100;
    // while (1)
    // {
    //     speed_left = 0;
    //     speed_right = 0;
    //     uart_stm_send();
    //     Get_Sensor_Data();
    //     printf("红外开关：%d %d %d %d\n", ir[0], ir[1], ir[2], ir[3]);
    //     printf("红外测距：%f %f\n", ir_distance[2], ir_distance[3]);
    //     printf("灰度传感器：%d %d %d %d\n", gray[0], gray[1], gray[2], gray[3]);
    //     printf("Ladar:\n");
    //     for (int i = 0; i < 2; i++)
    //     {
    //         printf("%.2f ", object_angle[i]);
    //     }
    //     delay(100);
    // }
}
