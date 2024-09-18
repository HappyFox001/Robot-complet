#include "apriltagDetect.hpp"

float x2angle(float x, int width)
{
    // 计算相对于图像中心的偏移量
    float offset = x - width / 2.0;

    // 计算偏移量占图像宽度的比例
    float ratio = offset / width;

    // 将比例转换为弧度
    float angle = ratio * (M_PI / 2.0);

    return angle;
}

// 仅仅检测tag36h11
int detect_Apriltag(void)
{
    // 开启摄像头
    // cv::VideoCapture cap(0, cv::CAP_V4L2);

    VideoCapture cap(0);
    if (!cap.isOpened())
    {
        std::cerr << "无法打开摄像头" << std::endl;
        return -1;
    }

    // 设置分辨率和帧率
    int width = 1280;
    int height = 720;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    apriltag_family_t *tf = NULL;
    const char *famname = "tag36h11";
    tf = tag36h11_create();
    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    if (errno == ENOMEM)
    {
        printf("Unable to add family to detector due to insufficient memory to allocate the tag-family decoder with the default maximum hamming value of 2. Try choosing an alternative tag family.\n");
        exit(-1);
    }

    td->quad_decimate = 2.0; // decimate
    td->quad_sigma = 0.0;    // blur
    td->nthreads = 2;        // threads
    td->debug = 0;           // debug
    td->refine_edges = 1;    // refine-edges

    cout << "  " << cap.get(CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CAP_PROP_FRAME_HEIGHT) << " @" << cap.get(CAP_PROP_FPS) << "FPS" << endl;
    Mat frame, gray;
    while (1)
    {
        errno = 0;
        cap >> frame;
        // to gray
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        // detect
        image_u8_t im = {gray.cols, gray.rows, gray.cols, gray.data};
        zarray_t *detections = apriltag_detector_detect(td, &im); // 在此处进行推理
        if (errno == EAGAIN)
        {
            printf("Unable to create the %d threads requested.\n", td->nthreads);
            exit(-1);
        }
        // 结果处理
        for (int i = 0; i < zarray_size(detections); i++)
        {

            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            printf("Detection id : %d  x:%f y:%f \n",  det->id, det->c[0], det->c[1]);

            // 状态清零
            enemy_energy.isDetected = 0;
            self_energy.isDetected = 0;
            public_energy.isDetected = 0;

            // 如果是敌方能量块
            if (det->id == ENEMY_BLOCK_ID)
            {
                enemy_energy.isDetected = 1;
                enemy_energy.angle = x2angle(det->c[0], width);
            }
            // 如果是我方能量块
            else if (det->id == SELF_ENERGY_BLOCK_ID)
            {
                self_energy.isDetected = 1;
                self_energy.angle = x2angle(det->c[0], width);
            }
            // 如果是公共能量块
            else if (det->id == PUBLIC_ENERGY_BLOCK_ID)
            {
                public_energy.isDetected = 1;
                public_energy.angle = x2angle(det->c[0], width);
            }
        }

        // 清理数据
        apriltag_detections_destroy(detections);
        // 显示图像
        // imshow("Tag Detections", frame);
    }
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
    return 0;
}

int start_ApriltagDetection()
{
    std::thread ApriltagThread(detect_Apriltag);
    if (ApriltagThread.joinable())
    {
        ApriltagThread.join();
    }
    return 0;
}
