#include "radar.hpp"
#include "CYdLidar.h"
using namespace std;
using namespace ydlidar;
using namespace cv;
extern float object_angle[2];
extern float object_distance[2];
// 检查两个点是否连续
bool arePointsContinuous(const LaserPoint &p1, const LaserPoint &p2, float angleThreshold, float rangeThreshold)
{
    float angleDiff = std::fabs(p1.angle - p2.angle);
    float rangeDiff = std::fabs(p1.range - p2.range);
    return angleDiff <= angleThreshold && rangeDiff <= rangeThreshold;
}

// 连续点检测函数
std::vector<std::vector<LaserPoint>> detectContinuousPoints(const std::vector<LaserPoint> &points, float angleThreshold, float rangeThreshold)
{
    std::vector<std::vector<LaserPoint>> continuousSegments;

    if (points.empty())
    {
        return continuousSegments;
    }

    std::vector<LaserPoint> currentSegment;
    currentSegment.push_back(points[0]);

    for (size_t i = 1; i < points.size(); ++i)
    {
        if (arePointsContinuous(points[i - 1], points[i], angleThreshold, rangeThreshold))
        {
            currentSegment.push_back(points[i]);
        }
        else
        {
            if (!currentSegment.empty())
            {
                continuousSegments.push_back(currentSegment);
                currentSegment.clear();
            }
            currentSegment.push_back(points[i]);
        }
    }

    // 添加最后一个段
    if (!currentSegment.empty())
    {
        continuousSegments.push_back(currentSegment);
    }

    return continuousSegments;
}

// 将角度和范围转换为图像坐标
cv::Point polarToImage(float angle, float range, int imgWidth, int imgHeight)
{
    // 假设激光雷达在图像中心
    int cx = imgWidth / 2;
    int cy = imgHeight / 2;

    // 计算图像坐标
    int x = static_cast<int>(cx + range * std::cos(angle) * 100); // 缩放因子 100
    int y = static_cast<int>(cy - range * std::sin(angle) * 100); // 缩放因子 100

    return cv::Point(x, y);
}

// 作用：过滤掉距离小于minRange或者大于maxRange的点，保留minRange到maxRange之间的点
// 参数：points：激光雷达采样点的集合 minRange：最小距离 maxRange：最大距离
// 返回：过滤后的点集合
std::vector<LaserPoint> filterPointsByRange(std::vector<LaserPoint> &points, double minRange, double maxRange)
{
    std::vector<LaserPoint> filteredPoints;
    filteredPoints.reserve(points.size());
    for (auto &point : points)
    {
        if (point.range > minRange && point.range < maxRange)
        {
            // 旋转90度
            // point.angle -= M_PI / 2;
            filteredPoints.push_back(point);
        }
    }
    return filteredPoints;
}

// 作用：过滤掉小于minSize的分割段
// 参数：segments：分段的集合 minSize：最小分段数
// 返回：过滤后的分段

std::vector<std::vector<LaserPoint>> filterSegmentsBySize(const std::vector<std::vector<LaserPoint>> &segments, size_t minSize)
{
    std::vector<std::vector<LaserPoint>> filteredSegments;
    filteredSegments.reserve(segments.size());
    for (const auto &segment : segments)
    {
        if (segment.size() > minSize)
        {
            filteredSegments.push_back(segment);
        }
    }
    return filteredSegments;
}

double calculateSegmentWidth(const std::vector<LaserPoint> &segment)
{
    // 计算分段的宽度
    // width=(LastAngle - FirstAngle)*AverageRange
    if (segment.empty())
    {
        return 0;
    }
    else
    {
        double firstAngle = segment.front().angle;
        double lastAngle = segment.back().angle;
        double averageRange = 0;
        for (const auto &point : segment)
        {
            averageRange += point.range;
        }
        averageRange /= segment.size();
        return abs((lastAngle - firstAngle) * averageRange);
    }
}

// 作用：按最近点的范围对分段进行排序,并取宽度在minWidth和maxWidth之间的分段
// 参数：segments：分段的集合 maxSegments：最大分段数 minWidth：最小宽度 maxWidth：最大宽度
// 返回：目标分段
std::vector<std::vector<LaserPoint>> filterClosestSegments(const std::vector<std::vector<LaserPoint>> &segments, size_t maxSegments, double minWidth, double maxWidth)
{
    std::vector<std::pair<double, std::vector<LaserPoint>>> distanceSegments;

    for (const auto &segment : segments)
    {
        double segmentWidth = calculateSegmentWidth(segment);
        // printf("Segment Width: %f\n", segmentWidth);
        if (segmentWidth >= minWidth && segmentWidth <= maxWidth)
        {
            double minRange = std::numeric_limits<double>::max();
            for (const auto &point : segment)
            {
                if (point.range < minRange)
                {
                    minRange = point.range;
                }
            }
            distanceSegments.emplace_back(minRange, segment);
        }
    }

    std::sort(distanceSegments.begin(), distanceSegments.end(), [](const std::pair<double, std::vector<LaserPoint>> &a, const std::pair<double, std::vector<LaserPoint>> &b)
              { return a.first < b.first; });

    std::vector<std::vector<LaserPoint>> closestSegments;
    closestSegments.reserve(maxSegments);
    for (size_t i = 0; i < std::min(maxSegments, distanceSegments.size()); ++i)
    {
        closestSegments.push_back(distanceSegments[i].second);
    }

    return closestSegments;
}

// 作用：计算分段的几何中心
// 参数：segments：分段的集合
// 返回：几何中心
std::pair<double, double> calculateGeometricCenter(const std::vector<LaserPoint> &segment)
{
    double totalAngle = 0;
    double totalRange = 0;
    size_t totalCount = segment.size();

    for (const auto &point : segment)
    {
        totalAngle += point.angle;
        totalRange += point.range;
    }

    double averageAngle = totalAngle / totalCount;
    double averageRange = totalRange / totalCount;

    return {averageAngle, averageRange};
}

// 作用：在图像上绘制分段
// 参数：img：图像 segments：分段的集合 colors：颜色 imgWidth：图像宽度 imgHeight：图像高度
// 返回：无
void drawSegments(cv::Mat &img, const std::vector<std::vector<LaserPoint>> &segments, const cv::Scalar *colors, int imgWidth, int imgHeight)
{
    for (size_t i = 0; i < segments.size(); ++i)
    {
        cv::Scalar color = colors[i % 3];
        for (size_t j = 0; j < segments[i].size() - 1; ++j)
        {
            cv::Point pt1 = polarToImage(segments[i][j].angle, segments[i][j].range, imgWidth, imgHeight);
            cv::Point pt2 = polarToImage(segments[i][j + 1].angle, segments[i][j + 1].range, imgWidth, imgHeight);
            cv::line(img, pt1, pt2, color, 2);
        }
    }
}

void drawReferenceLines(cv::Mat &img, int imgWidth, int imgHeight, double maxRange)
{
    // Draw 0-degree line (vertical line in the middle)
    cv::Point center(imgWidth / 2, imgHeight / 2);
    cv::line(img, center, cv::Point(center.x, 0), cv::Scalar(255, 255, 255), 1);

    // Draw range circles (from 1 meter to maxRange)
    for (double range = 1.0; range <= maxRange; range += 1.0)
    {
        int radius = static_cast<int>((range / maxRange) * (imgHeight / 2));
        cv::circle(img, center, radius, cv::Scalar(255, 255, 255), 1);
    }
}

int lidar_start()
{
    // init system signal
    ydlidar::os_init();

    CYdLidar laser;
    //////////////////////string property/////////////////
    /// Lidar ports
    std::map<std::string, std::string> ports = ydlidar::lidarPortList();
    std::string port = "/dev/ttyUSB0";
    // if(!ports.empty()) {
    //     port = ports.begin()->second;
    // }
    /// lidar port
    laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
    /// ignore array
    std::string ignore_array;
    ignore_array.clear();
    laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                      ignore_array.size());

    //////////////////////int property/////////////////
    /// lidar baudrate
    int optval = 115200;
    laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
    /// tof lidar
    optval = TYPE_TRIANGLE;
    laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    /// device type
    optval = YDLIDAR_TYPE_SERIAL;
    laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    /// sample rate
    optval = 4;
    laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    /// abnormal count
    optval = 1;
    laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    //////////////////////bool property/////////////////
    /// fixed angle resolution
    bool b_optvalue = true;
    laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
    /// rotate 180
    laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
    /// Counterclockwise
    laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
    b_optvalue = true;
    laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
    /// one-way communication
    b_optvalue = true;
    laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
    /// intensity
    b_optvalue = false;
    laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
    /// Motor DTR
    b_optvalue = true;
    laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
    /// HeartBeat
    b_optvalue = false;
    laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

    //////////////////////float property/////////////////
    /// unit: °
    float f_optvalue = 180.f;
    laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
    f_optvalue = -180.f;
    laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
    /// unit: m
    f_optvalue = 6.f;
    laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
    f_optvalue = 0.08f;
    laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
    /// unit: Hz
    f_optvalue = 6.f;
    laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));

    // initialize SDK and LiDAR
    printf("__Test__\n");
    bool ret = laser.initialize();
    if (ret)
    { // success
        // Start the device scanning routine which runs on a separate thread and enable motor.
        ret = laser.turnOn();
    }
    else
    {
        fprintf(stderr, "%s\n", laser.DescribeError());
        fflush(stderr);
    }

    // Turn On success and loop
    float angleThreshold = 0.15f; // 设定的角度阈值
    float rangeThreshold = 0.3f;  // 设定的距离阈值

    // // 创建一个空白图像
    // int imgWidth = 800;
    // int imgHeight = 800;
    // cv::Mat img = cv::Mat::zeros(imgHeight, imgWidth, CV_8UC3);

    // 设置颜色
    cv::Scalar colors[] = {cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 255, 0), cv::Scalar(255, 0, 255)};

    while (ret && ydlidar::os_isOk())
    {
        LaserScan scan;

        if (laser.doProcessSimple(scan))
        {

            // 保留0.1到5米之间的点
            std::vector<LaserPoint> points = filterPointsByRange(scan.points, 0.1, 0.8) ;
            // 检测连续点
            std::vector<std::vector<LaserPoint>> segments = detectContinuousPoints(points, angleThreshold, rangeThreshold);
            //printf("size of segments:%ld \n",segments.size());

            // 创建一个空白图像
            // img.setTo(cv::Scalar(0, 0, 0));
            // 绘制激光雷达的点
            // for (const auto &point : points)
            // {
            //     cv::Point pt = polarToImage(point.angle, point.range, imgWidth, imgHeight);
            //     cv::circle(img, pt, 1, cv::Scalar(255, 255, 255), -1);
            // }

            // 过滤掉小于7个点的分段
            std::vector<std::vector<LaserPoint>> segments2 = filterSegmentsBySize(segments, 5);
            //printf("size of segments2:%ld \n",segments2.size());
            // 按最近点的范围对分段进行排
            std::vector<std::vector<LaserPoint>> closestSegments = filterClosestSegments(segments2, 2, 0.1, 0.6);
            // 计算最近分段（物体）的几何中心
           // printf("size of closestSegments:%ld \n",closestSegments.size());


            if( closestSegments.size())
            {
                for (size_t i = 0; i < closestSegments.size(); ++i)
                {
                    auto [averageAngle, averageRange] = calculateGeometricCenter(closestSegments[i]);
                    object_angle[i] = averageAngle * 180 / 3.14;
                    object_distance[i] = averageRange;
                    // printf("Segment %zu Geometric Center - Angle: %f, Range: %f\n", i, averageAngle, averageRange);
                    // 在[averageAngle, averageRange]处绘制一个圆
                    ////////////////////////////////////正式比赛时禁用////////////////////////////////////
                    // cv::Point pt = polarToImage(averageAngle, averageRange, imgWidth, imgHeight);
                    // cv::circle(img, pt, 10, colors[i % 3], -1);
                    ///////////////////////////////////////////////////////////////////////////////
                }
            }
            else{
                for (size_t i = 0; i < 2; ++i)
                {
                    object_angle[i] = 360;
                    object_distance[i] = -1;
                    
                }
            }

            ////////////////////////开始绘制（正式比赛事禁用）////////////////////////
            // //生成参考线
            // drawReferenceLines(img, imgWidth, imgHeight, 5.0);
            // // 在图像上绘制
            // drawSegments(img, closestSegments, colors, imgWidth, imgHeight);

            //cv::imshow("Laser Point Segments", img);
            // if (cv::waitKey(30) == 27)
            // {
            //     break;
            // }
            //////////////////////////////结束绘制//////////////////////////////////
        }
        else
        {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }

    return 0;
}

// 作用：启动激光雷达线程
int lidarThreadStart()
{
    std::thread lidarThread(lidar_start);
    if (lidarThread.joinable())
    {
        lidarThread.join();
    }
    return 0;
}
