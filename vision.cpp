#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <tuple>

#include <mutex>
#include <fstream>

#include "vision.h"
#include "message.h"
#include "globals.h"
#include "timer.h"

#include "ts_queue.h"

using namespace std;

bool isFindBlueBarrier = false;
bool isRemovedBlueBarrier = false;
BlueBarrierInit BlueBarrierConfig;

std::vector<FrameData> g_results;
std::mutex g_results_mutex;

//用State定义VisionTaskState，用于表示视觉处理状态
State VisionTaskState = State::BlueBarrier;
//定义斑马线任务信息管理变量
ZebraInfo ZebraInformation;
//定义转向标志识别任务信息管理变量
ArrowInfo ArrowInformation;
extern ThreadSafeQueue<cv::Mat> g_frame_queue; // 从 main.cpp 引入全局队列

void vision_loop()
{
    cv::VideoCapture cap("testline.mp4");
	//创建一个对象，用于储存照片
	cv::Mat frame;



	while(!allfinishflag.load())
	{
		//从摄像头获取照片储存在frame中
        cap.read(frame);
        cv::Mat frame_clone = frame.clone();//克隆一份
		//图像处理状态机
		if (VisionTaskState == State::BlueBarrier)
		{
            isBlueBarrierRemoved(frame_clone);
            //这个函数会在不断循环中检测蓝色挡板是否被移开，如果移开，
            // 则会通知t_control进程"Start"
            //从而在t_control中将电机速度设置为一个不为0的值
            //然后切换到下一个状态ToBlueCone
		}
		else if(VisionTaskState == State::ToBlueCone)
		{
            //updateTargetRoute(frame_clone);
            //-------测试--------
            IMfr.fetch_add(1);//帧率计数
            std::tuple<double, cv::Vec4f, cv::Vec4f> res = DetectLeftRightLines(frame_clone);
            double error;
            cv::Vec4f left_line_fit, right_line_fit;
            std::tie(error, left_line_fit, right_line_fit) = res;
            lane_error.store(error);
            cv::Mat img = cv::Mat::zeros(96, 320, CV_8UC3);
            drawFitLine(img, left_line_fit, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            drawFitLine(img, right_line_fit, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            //绘制中点
            cv::Point target_point(error+160, 60);
            cv::circle(img, target_point, 3, cv::Scalar(255), -1);
            //绘制目标中点
            cv::Point now_point(160, 60);
            cv::circle(img, now_point, 4, cv::Scalar(255), -1);
            // --- 新增：推送帧到队列 ---
            if (!allfinishflag.load())
            {
                g_frame_queue.push(img);
            }
            // ------------------------
            std::lock_guard<std::mutex> lock(g_results_mutex);
            g_results.push_back({ error, left_line_fit, right_line_fit });
            //--------------------
            //此时车辆已经发车，在路上会遇到一个锥桶，
            //在遇到锥桶之前，处理图像，识别跑道，把与中点的误差更新到全局变量lane_error中去
            //在遇到锥桶之后，选择从左边或者右边绕过去，把与规划好的路线的误差更新到lane_error中去
            //t_control进程会不断循环，根据lane_error的值调整舵机角度，实现沿着路线行驶
            //当判断已经绕过锥桶之后，切换到下一个状态ZebraCrossing
		}
		else if(VisionTaskState == State::ToZebraCrossing)
		{
            searchZebraCrossing(frame_clone);
            //此时会在路上遇到斑马线
            //在遇到斑马线之前，处理图像，识别跑道，把与中点的误差更新到全局变量lane_error中去
            //在遇到斑马线之后，则通知t_control进程"Stop"(使用send_message("Stop"))
            //随后t_control进程会控制小车停车，并播报录音（与本函数无关），
            //随后切换到下一个状态SearchLaneChangeMarker
		}
        else if (VisionTaskState == State::SearchLaneChangeMarker)
        {
            DetectLaneChangeSign(frame_clone);
            //这个函数进入3秒倒计时，随后通知t_control进程"ReStart",使得t_control进程重新发车
            //但是这个函数需要在这3秒内，识别斑马线后面的转向标志，并更新全局变量lane_change_direction的值
            //全局变量lane_change_direction的值可以是宏定义LANE_CHANGE_NONE，LANE_CHANGE_LEFT，LANE_CHANGE_RIGHT
            //10秒结束后，切换到下一个状态LaneChange
        }
        else if (VisionTaskState == State::LaneChange)
        {
            PlanLaneChangeRoute(frame_clone);
            //这个函数需要先读取全局变量lane_change_direction的值，来查看下一个任务是左转还是右转
            //根据lane_change_direction的值分别进行图像处理，规划出路线
            //然后把与规划好的路线的误差更新到lane_error中去
            //当判断离开弯道并回正时，切换到下一个状态ToTheEnd
        }
        else if (VisionTaskState == State::ToTheEnd)
        {
            FollowLaneAndStop(frame_clone);
            //这个函数需要在到达停车区域之前沿着跑道循迹，把与中点的误差更新到全局变量lane_error中去
            //同时不断识别地上可能出现的停车区域标志，识别完成后，把识别结果存储在全局变量stop_area中
            //全局变量stop_area的宏定义取值可以是AREA_NONE,AREA_A,AREA_B
            //随后根据识别到的区域规划左停车和右停车的路线
            //当认为可以停下时，则通知t_control进程"ReStop"(使用send_message("ReStop"))
        }
        else if(VisionTaskState == State::QR_Code)
        {
            SendCameraFrameToPC(frame_clone);
            //这个函数需要将摄像头画面回传给电脑
            //然后由人工扫码
        }
	}
    cap.release();
}


//>>>>>>识别蓝色挡板<<<<<<<<<<
void isBlueBarrierRemoved(cv::Mat frame_clone)
{
    if (isFindBlueBarrier == false)
    {
        FindBlueBarrier(frame_clone, BlueBarrierConfig);
        isFindBlueBarrier = BlueBarrierConfig.is_bluebarrier_present;
        std::cout << "还未检测到蓝色挡板" << std::endl;
    }
    else
    {
        if (isRemovedBlueBarrier == false)
        {
            FindBlueBarrier(frame_clone, BlueBarrierConfig);
            isRemovedBlueBarrier = !BlueBarrierConfig.is_bluebarrier_present;
            std::cout << "检测到蓝色挡板" << std::endl;
        }
        else
        {
            //视觉处理切换到下一个状态
            VisionTaskState = State::ToBlueCone;
            //向t_control进程发送信息
            send_message("Start");
            std::cout << "发车" << std::endl;
        }
    }
}
void FindBlueBarrier(cv::Mat frame_clone, BlueBarrierInit& config)
{
    // 保存原图一份用于显示
    cv::Mat display = frame_clone.clone();
    // 把frame_clone转换为 HSV 色彩空间
    cv::cvtColor(frame_clone, frame_clone, cv::COLOR_BGR2HSV);
    // 从 frame_clone 中提取 ROI 区域，得到一个子图像 roiArea
    // 对 roiArea 的修改会直接作用在 frame_clone 上
    cv::Mat roiArea = frame_clone(config.roi);
    //定义一个变量mask储存提取蓝色后的结果
    cv::Mat mask;
    //将ROI区域中符合要求的蓝色置为白色
    cv::inRange(roiArea, config.lower_blue, config.upper_blue, mask);
    //统计mask中白色像素的个数
    int white_area = cv::countNonZero(mask);
    if (white_area > config.white_threshold)
    {
        config.consecutive_detected++;
        config.consecutive_missed = 0;
        if (config.consecutive_detected >= config.detect_threshold_frames)
        {
            config.is_bluebarrier_present = true;
        }
    }
    else
    {

        config.consecutive_missed++;
        config.consecutive_detected = 0;
        if (config.consecutive_missed >= config.miss_threshold_frames)
        {
            config.is_bluebarrier_present = false;
        }

    }
    //调试功能
    if (config.enable_debug_display)
    {
        // ---------- 叠加 mask 到原图 ----------
        // 将 mask 转为三通道（白色区域才有值）
        cv::Mat mask_color;
        cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);

        // 定义 ROI 区域原图部分
        cv::Mat roi_display = display(config.roi);

        // 让白色区域在原图上变亮（简单叠加）
        roi_display.setTo(cv::Scalar(255, 255, 255), mask);

        // ---------- 绘制 ROI 框 ----------
        cv::rectangle(display, config.roi, cv::Scalar(0, 255, 0), 2); // 绿色框

        // ---------- 显示白色像素数量 ----------
        std::string text = "White Pixels: " + std::to_string(white_area);
        cv::putText(display, text, cv::Point(config.roi.x + 5, config.roi.y + 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        // ---------- 显示整张图 ----------
        cv::imshow("Blue Barrier Detection", display);
        cv::waitKey(1);
    }

}
//>>>>>规划绕行锥桶路线<<<<<<<
void updateTargetRoute(const cv::Mat& frame_clone) 
{
    /**
     * 功能需求：
     * - 在遇到锥桶之前，处理图像，识别跑道，把与中点的误差更新到全局变量lane_error中去
     * - 在遇到锥桶之后，选择从左边或者右边绕过去，把与规划好的路线的误差更新到lane_error中去
     * - 当绕过锥桶之后，切换到下一个状态ToZebraCrossing
     *
     * 注意事项：
     *  - lane_error 为线程共用原子变量，读写是线程安全的
     *  - 斑马线检测方法可由后续开发者自由选择（图像处理或深度学习）
     *
     * 传入参数:
     * - 摄像机捕捉的一帧图像的克隆frame_clone
     * 
     * 返回值:无
     */
}
//辅助函数

void gammaCorrection(const cv::Mat& input, cv::Mat& output) {
    // 将图像转换为灰度图像以计算平均亮度
    cv::Mat gray_image;
    cv::cvtColor(input, gray_image, cv::COLOR_BGR2GRAY);
    double mean_intensity = cv::mean(gray_image)[0];

    // 根据平均亮度选择伽马值
    double gamma = 0.5 + mean_intensity / 80.0;
    if (gamma > 2) gamma = 2;

    // 创建查找表
    cv::Mat lookupTable(1, 256, CV_8U);
    for (int i = 0; i < 256; i++) {
        lookupTable.at<uchar>(i) = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
    }

    cv::LUT(input, lookupTable, output);
}

void drawFitLine(cv::Mat& img, const cv::Vec4f& linefit,
    const cv::Scalar& color, int thickness = 2, int lineType = cv::LINE_AA)
{
    float vx = linefit[0], vy = linefit[1];
    float x0 = linefit[2], y0 = linefit[3];

    // 在图像的顶部和底部各取一个 y，求对应的 x
    float y_top = 0.0f;
    float y_bot = static_cast<float>(img.rows - 1);

    cv::Point2f p1, p2;

    // 处理接近水平/竖直的情况，避免除以 0
    const float eps = 1e-6f;
    if (std::abs(vy) > eps) {
        // x = x0 + (y - y0) * (vx / vy)
        p1 = cv::Point2f(x0 + (y_top - y0) * (vx / vy), y_top);
        p2 = cv::Point2f(x0 + (y_bot - y0) * (vx / vy), y_bot);
    }
    else {
        // vy ≈ 0，方向几乎水平：直接用左右边界的 x
        float x_left = 0.0f;
        float x_right = static_cast<float>(img.cols - 1);
        // y = y0 + (x - x0) * (vy / vx)；此时 vy≈0，所以 y≈y0
        p1 = cv::Point2f(x_left, y0);
        p2 = cv::Point2f(x_right, y0);
    }

    // 将线段裁剪到图像边界内，避免越界
    cv::Point ip1(cvRound(p1.x), cvRound(p1.y));
    cv::Point ip2(cvRound(p2.x), cvRound(p2.y));
    if (cv::clipLine(img.size(), ip1, ip2)) {
        cv::line(img, ip1, ip2, color, thickness, lineType);
    }
}
double calcAverageX(
    const cv::Vec4f& left_line,
    const cv::Vec4f& right_line,
    bool has_left,
    bool has_right,
    double y1,
    double y2)
{
    auto calcX = [](const cv::Vec4f& line, double y) {
        double vx = line[0], vy = line[1];
        double x0 = line[2], y0 = line[3];
        return x0 + vx / vy * (y - y0);
        };

    // 左线的平均x
    double xavg_left;
    if (has_left) {
        double x1 = calcX(left_line, y1);
        double x2 = calcX(left_line, y2);
        xavg_left = (x1 + x2) / 2.0;
    }
    else {
        xavg_left = 0.0; // 左边界
    }

    // 右线的平均x
    double xavg_right;
    if (has_right) {
        double x1 = calcX(right_line, y1);
        double x2 = calcX(right_line, y2);
        xavg_right = (x1 + x2) / 2.0;
    }
    else {
        xavg_right = 320.0; // 右边界
    }

    // 返回整体平均横坐标
    return (xavg_left + xavg_right) / 2.0;
}
std::tuple<double, cv::Vec4f, cv::Vec4f> DetectLeftRightLines(cv::Mat& data)
{
    cv::Rect roi_rect(0, (data.rows / 2 - 90 + 60), data.cols, (data.rows / 2.5));
    cv::Mat cropped_image = data(roi_rect);
    cv::resize(cropped_image, cropped_image, cv::Size(), 0.5, 0.5);


    gammaCorrection(cropped_image, cropped_image);

    cv::Mat hsv_image;
    cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat gray_image;
    cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat blur;
    cv::bilateralFilter(gray_image, blur, 7, 60, 60);
    cv::Mat gaussian_blur;
    cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);

    cv::Mat ca;
    cv::Canny(gaussian_blur, ca, 30, 50);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat dilated_ca;
    cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 2);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 50, 25, 10);

    std::vector<cv::Vec4i> right_lines;
    std::vector<cv::Vec4i> left_lines;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180.0 / CV_PI;

        if (angle_deg < 0)  angle_deg += 180;  // 把角度范围调整到 [0,180)
        // 根据角度筛选左右线
        if (angle_deg >= 18 && angle_deg <= 89)
        {
            right_lines.push_back(line);
        }
        else if (angle_deg >= 91 && angle_deg <= 175)
        {
            left_lines.push_back(line);
        }
    }
    //最小二乘法
    auto fitMainLine = [](const std::vector<cv::Vec4i>& lines, cv::Vec4f& line_out) -> bool {
        if (lines.empty()) return false;
        std::vector<cv::Point2f> pts;
        for (auto& l : lines) {
            pts.emplace_back(l[0], l[1]);
            pts.emplace_back(l[2], l[3]);
        }
        if (pts.size() < 2) return false;
        cv::fitLine(pts, line_out, cv::DIST_L2, 0, 0.01, 0.01);
        return true;
        };

    cv::Vec4f left_line_fit, right_line_fit;
    bool has_left = fitMainLine(left_lines, left_line_fit);
    bool has_right = fitMainLine(right_lines, right_line_fit);
    //求平均中点
    double avgX = calcAverageX(left_line_fit, right_line_fit, has_left, has_right, 30, 60);


    //计算误差
    double error = avgX - 160;
    return std::make_tuple(error, left_line_fit, right_line_fit);
}

void SaveResultsToCSV(const std::string& filename) {
    std::lock_guard<std::mutex> lock(g_results_mutex);
    std::ofstream file(filename);

    file << "error,left_x1,left_y1,left_x2,left_y2,right_x1,right_y1,right_x2,right_y2\n";

    for (const auto& d : g_results) {
        file << d.error << ","
            << d.left_line[0] << "," << d.left_line[1] << "," << d.left_line[2] << "," << d.left_line[3] << ","
            << d.right_line[0] << "," << d.right_line[1] << "," << d.right_line[2] << "," << d.right_line[3] << "\n";
    }

    file.close();
    std::cout << "Datas saved to " << filename << std::endl;
}
//>>>>>>>>沿线继续行驶直到斑马线停车并发车<<<<<<<<<
void searchZebraCrossing(const cv::Mat& frame_clone) 
{
    //初步图像处理
    cv::Rect roi_rect(0, (frame_clone.rows / 2 - 90 + 60), frame_clone.cols, (frame_clone.rows / 2.5));
    cv::Mat cropped_image = frame_clone(roi_rect);
    gammaCorrection(cropped_image, cropped_image);
    //检测跑道并将误差写入lane_error
    std::tuple<double, cv::Vec4f, cv::Vec4f> res = DetectLeftRightLinesForZebra(cropped_image);
    double error;
    cv::Vec4f left_line_fit, right_line_fit;
    std::tie(error, left_line_fit, right_line_fit) = res;
    lane_error.store(error);
    //检测斑马线
    dect_rxd(cropped_image);
    if (ZebraInformation.final_found_rxd == true)
    {
        std::cout << "Rxd find" << std::endl;
        VisionTaskState = State::SearchLaneChangeMarker;
    }
    /**
     *
     * 功能需求：
     * - 此时会在路上遇到斑马线
     * - 在遇到斑马线之前，处理图像，识别跑道，把与中点的误差更新到全局变量lane_error中去
     * - 在遇到斑马线之后，则通知t_control进程"Stop"(使用send_message("Stop"))
     * - 随后切换到下一个状态SearchLaneChangeMarker

     * 注意事项：
     *  - lane_error 为线程共用原子变量，读写是线程安全的
     *  - 斑马线检测方法可由后续开发者自由选择（图像处理或深度学习）
     *
     * 传入参数:
     * - 摄像机捕捉的一帧图像的克隆frame_clone
     *
     * 返回值:无
     */
}
std::tuple<double, cv::Vec4f, cv::Vec4f> DetectLeftRightLinesForZebra(cv::Mat& cropped_image)
{
    cv::Mat hsv_image;
    cv::cvtColor(cropped_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat gray_image;
    cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);

    cv::Mat blur;
    cv::bilateralFilter(gray_image, blur, 7, 60, 60);
    cv::Mat gaussian_blur;
    cv::GaussianBlur(blur, gaussian_blur, cv::Size(5, 5), 30);

    cv::Mat ca;
    cv::Canny(gaussian_blur, ca, 30, 50);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    cv::Mat dilated_ca;
    cv::dilate(ca, dilated_ca, kernel, cv::Point(-1, -1), 2);

    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(dilated_ca, lines, 1, CV_PI / 180, 50, 25, 10);

    std::vector<cv::Vec4i> right_lines;
    std::vector<cv::Vec4i> left_lines;
    for (size_t i = 0; i < lines.size(); ++i)
    {
        cv::Vec4i line = lines[i];
        double angle_rad = atan2(line[3] - line[1], line[2] - line[0]);
        double angle_deg = angle_rad * 180.0 / CV_PI;

        if (angle_deg < 0)  angle_deg += 180;  // 把角度范围调整到 [0,180)
        // 根据角度筛选左右线
        if (angle_deg >= 18 && angle_deg <= 89)
        {
            right_lines.push_back(line);
        }
        else if (angle_deg >= 91 && angle_deg <= 175)
        {
            left_lines.push_back(line);
        }
    }
    //最小二乘法
    auto fitMainLine = [](const std::vector<cv::Vec4i>& lines, cv::Vec4f& line_out) -> bool {
        if (lines.empty()) return false;
        std::vector<cv::Point2f> pts;
        for (auto& l : lines) {
            pts.emplace_back(l[0], l[1]);
            pts.emplace_back(l[2], l[3]);
        }
        if (pts.size() < 2) return false;
        cv::fitLine(pts, line_out, cv::DIST_L2, 0, 0.01, 0.01);
        return true;
        };

    cv::Vec4f left_line_fit, right_line_fit;
    bool has_left = fitMainLine(left_lines, left_line_fit);
    bool has_right = fitMainLine(right_lines, right_line_fit);


    //求平均中点
    double avgX = calcAverageX(left_line_fit, right_line_fit, has_left, has_right, 30, 60);


    //计算误差
    double error = avgX - 160;
    return std::make_tuple(error, left_line_fit, right_line_fit);
}
void dect_rxd(const cv::Mat& cropped_image){
    cv::Mat gray_image;
    cv::cvtColor(cropped_image, gray_image, cv::COLOR_BGR2GRAY);
    cv::Mat blur;
    cv::GaussianBlur(gray_image, blur, cv::Size(7, 7), 0);
    cv::Mat thresh1;
    cv::threshold(blur, thresh1, 175, 255, cv::THRESH_BINARY);
    cv::Mat mask;
    cv::erode(thresh1, mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 1);

    std::vector<std::vector<cv::Point>> cnts;
    cv::findContours(mask.clone(), cnts, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    ZebraInformation.now_num_rectangles = 0;    //清零上一帧找到的矩形数量

    //矩形筛选：1、面积处于一定范围；2、宽度大于某个值；3、数量大于某个值；满足以上判断为此帧发现斑马线
    for (size_t i = 0; i < cnts.size(); i++) {
        double area = cv::contourArea(cnts[i]);
        if (area >= ZebraInformation.area_min && area <= ZebraInformation.area_max) {
            cv::Rect rect = cv::boundingRect(cnts[i]);
            if (rect.width > ZebraInformation.rect_width) {
                ZebraInformation.now_num_rectangles ++;
            }
        }
    }
    if (ZebraInformation.now_num_rectangles >= ZebraInformation.num_rectangles) {
        ZebraInformation.numbercounter += 1;
    }
    //连续多帧发现斑马线才判定为真的找到斑马线
    if (ZebraInformation.numbercounter  >= ZebraInformation.confirm_frames){
        ZebraInformation.final_found_rxd = true;
    }
}
//>>>>>>>>>识别变道标志<<<<<<<<<
void DetectLaneChangeSign(const cv::Mat& frame_clone)
{
    //启动计时器，记录第一次进入的时间
    if (ArrowInformation.time_initialized == false) {
        ArrowInformation.start_time = std::chrono::high_resolution_clock::now();
        ArrowInformation.time_initialized = true;
    }

    //图像预处理
    cv::Mat cut_image = frame_clone.clone()(cv::Rect(30, frame_clone.rows / 2, frame_clone.cols - 60, frame_clone.rows / 2.5));
    gammaCorrection(cut_image, cut_image);

    cv::Mat hsv_image;
    cv::cvtColor(cut_image, hsv_image, cv::COLOR_BGR2HSV);
    cv::Mat mask_arrow;
    if (ArrowInformation.isbluerunway == true) {
        cv::Mat Red1, Red2;
        cv::Scalar scalarl1 = cv::Scalar(0, 43, 46);
        cv::Scalar scalarl2 = cv::Scalar(146, 43, 46);
        cv::Scalar scalarH1 = cv::Scalar(10, 255, 255);
        cv::Scalar scalarH2 = cv::Scalar(180, 255, 255);
        cv::inRange(hsv_image, scalarl1, scalarH1, Red1);
        cv::inRange(hsv_image, scalarl2, scalarH2, Red2);
        mask_arrow = Red1 | Red2;
    }
    else {
        cv::Scalar scalarl = cv::Scalar(95, 65, 65); // Scalar(100, 43, 46);  Scalar(124, 255, 255);
        cv::Scalar scalarH = cv::Scalar(125, 255, 255);
        cv::inRange(hsv_image, scalarl, scalarH, mask_arrow);
    }

    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);

    cv::erode(mask_arrow, mask_arrow, kernel, cv::Point(-1, -1), 1);
    cv::dilate(mask_arrow, mask_arrow, kernel, cv::Point(-1, -1), 1);

    cv::dilate(mask_arrow, mask_arrow, kernel, cv::Point(-1, -1), 1);
    cv::erode(mask_arrow, mask_arrow, kernel, cv::Point(-1, -1), 1);

    //记录现在时间
    ArrowInformation.now_time = std::chrono::high_resolution_clock::now();
    //计算耗时
    auto time_diff = ArrowInformation.now_time - ArrowInformation.start_time;
    std::chrono::duration<double, std::milli> time_ms = time_diff;
    if (time_ms.count() <= ArrowInformation.time_gap.count())
    {
        dect_arrow(mask_arrow);
    }
    else
    {
        if (ArrowInformation.left_count > ArrowInformation.right_count)
        {
            std::cout << "Final Left" << std::endl;
            lane_change_direction.store(LANE_CHANGE_LEFT);
        }
        else
        {
            std::cout << "Final Right" << std::endl;
            lane_change_direction.store(LANE_CHANGE_RIGHT);
        }
        send_message("ReStart");
        VisionTaskState = State::LaneChange;
    }
}
void dect_arrow(cv::Mat mask_arrow)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_arrow, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        std::cout << "no arrow flag1" << std::endl;
    }
    else
    {
        //找出最大轮廓roiContour
        auto max_contour = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        cv::Rect boundingBox = cv::boundingRect(*max_contour);
        cv::Mat roiContour = mask_arrow.clone()(boundingBox);
        //去除左右和下方背景并反色
        for (int row = 0; row < roiContour.rows; row++) {
            for (int col = 0; col < roiContour.cols; col++) {
                if (roiContour.at<uchar>(row, col) == 0) {  // 黑色
                    roiContour.at<uchar>(row, col) = 255;  // 变成白色
                }
                else {
                    break;  // 遇到黑色，跳出内层循环 
                }
            }
        }

        for (int row = 0; row < roiContour.rows; row++) {
            for (int col = roiContour.cols - 1; col >= 0; col--) {
                if (roiContour.at<uchar>(row, col) == 0) {  // 黑色
                    roiContour.at<uchar>(row, col) = 255;  // 变成白色
                }
                else {
                    break;  // 遇到黑色，跳出内层循环 
                }
            }
        }

        for (int row = roiContour.rows - 1; row >= roiContour.rows - 5; row--) {
            for (int col = roiContour.cols - 1; col >= 0; col--) {
                if (roiContour.at<uchar>(row, col) == 0) {  // 黑色
                    roiContour.at<uchar>(row, col) = 255;  // 变成白色
                }
            }
        }
        roiContour = 255 - roiContour;        //反色
        //再次提取轮廓
        std::vector<std::vector<cv::Point>> contours_roi;
        cv::findContours(roiContour, contours_roi, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours_roi.empty()) {
            std::cout << "no arrow flag2" << std::endl;
        }
        else
        {
            //找出最大轮廓
            auto contour = *std::max_element(contours_roi.begin(), contours_roi.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(contour);
            //筛选：面积大于MIN_AREA_THRESHOLD
            if (area > ArrowInformation.MIN_AREA_THRESHOLD)
            {
                //筛选：长宽比在ASPECT_RATIO_MIN、ASPECT_RATIO_MAX之间
                cv::Rect rect = cv::boundingRect(contour);
                float aspect_ratio = (float)rect.width / rect.height;
                if (ArrowInformation.ASPECT_RATIO_MIN < aspect_ratio && aspect_ratio < ArrowInformation.ASPECT_RATIO_MAX)
                {
                    std::vector<cv::Point> approx;
                    double epsilon = 0.03 * cv::arcLength(contour, true);
                    cv::approxPolyDP(contour, approx, epsilon, true);
                    //筛选：顶点个数大于4。并找出箭头箭尾
                    if (approx.size() >= 4)
                    {
                        //计算质心
                        cv::Moments M = cv::moments(contour);
                        int cx = int(M.m10 / M.m00);
                        int cy = int(M.m01 / M.m00);
                        // 选出箭头的尖端和尾部点
                        cv::Point tipPoint, tailPoint;
                        double maxTipDistance = 0;
                        double maxTailDistance = 0;
                        for (const cv::Point& p : approx)
                        {
                            double distance = std::sqrt(std::pow(p.x - cx, 2) + std::pow(p.y - cy, 2));
                            // 寻找最远的箭头尖端
                            if (p.y < cy && distance > maxTipDistance) {
                                maxTipDistance = distance;
                                tipPoint = p;
                            }
                            // 寻找最远的箭尾
                            if (p.y > cy && distance > maxTailDistance) {
                                maxTailDistance = distance;
                                tailPoint = p;
                            }
                        }
                        // 判断箭头的方向
                        double angle1 = std::atan2(tipPoint.y - cy, tipPoint.x - cx) * 180.0 / CV_PI;
                        double angle2 = std::atan2(-tailPoint.y + cy, -tailPoint.x + cx) * 180.0 / CV_PI;
                        if (angle1 > angle2)
                        {
                            std::cout << "Right" << std::endl;
                            ArrowInformation.right_count += 1;
                        }
                        else
                        {
                            std::cout << "Left" << std::endl;
                            ArrowInformation.left_count += 1;
                        }
                    }
                }

            }
        }
    }
}
//>>>>>>>>>规划变道路线<<<<<<<<<
void PlanLaneChangeRoute(const cv::Mat& frame_clone)
{
    /**
     *
     * 功能说明：
     *这个函数需要先读取全局变量lane_change_direction的值，来查看下一个任务是左转还是右转
     * - 根据lane_change_direction的值分别进行图像处理，规划出路线
     * - 然后把与规划好的路线的误差更新到lane_error中去
     * - 当判断离开弯道并回正时，切换到下一个状态ToTheEnd
     *
     * 传入参数:
     * - 摄像机捕捉的一帧图像的克隆frame_clone
     *
     * 返回值:无
     */
}
//>>>>>>>>>停车<<<<<<<<<
void FollowLaneAndStop(const cv::Mat& frame_clone)
{
    /**
     *
     * 功能说明：
     * - 这个函数需要在到达停车区域之前沿着跑道循迹，把与中点的误差更新到全局变量lane_error中去
     * - 同时不断识别地上可能出现的停车区域标志，识别完成后，把识别结果存储在全局变量stop_area中
     * - 全局变量stop_area的宏定义取值可以是AREA_NONE,AREA_A,AREA_B
     * - 随后根据识别到的区域规划左停车和右停车的路线
     * - 当认为可以停下时，则通知t_control进程"ReStop"(使用send_message("ReStop"))
     * 
     * 传入参数:
     * - 摄像机捕捉的一帧图像的克隆frame_clone
     *
     * 返回值:无
     */
}
//>>>>>>>>>识别二维码<<<<<<<<<
void SendCameraFrameToPC(const cv::Mat& frame_clone)
{
    /**
     *
     * 功能说明：
     * - 这个函数需要将摄像头画面回传给电脑
     *
     * 传入参数:
     * - 摄像机捕捉的一帧图像的克隆frame_clone
     *
     * 返回值:无
     */
}