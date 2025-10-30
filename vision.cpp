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

//��State����VisionTaskState�����ڱ�ʾ�Ӿ�����״̬
State VisionTaskState = State::BlueBarrier;
//���������������Ϣ�������
ZebraInfo ZebraInformation;
//����ת���־ʶ��������Ϣ�������
ArrowInfo ArrowInformation;
extern ThreadSafeQueue<cv::Mat> g_frame_queue; // �� main.cpp ����ȫ�ֶ���

void vision_loop()
{
    cv::VideoCapture cap("testline.mp4");
	//����һ���������ڴ�����Ƭ
	cv::Mat frame;



	while(!allfinishflag.load())
	{
		//������ͷ��ȡ��Ƭ������frame��
        cap.read(frame);
        cv::Mat frame_clone = frame.clone();//��¡һ��
		//ͼ����״̬��
		if (VisionTaskState == State::BlueBarrier)
		{
            isBlueBarrierRemoved(frame_clone);
            //����������ڲ���ѭ���м����ɫ�����Ƿ��ƿ�������ƿ���
            // ���֪ͨt_control����"Start"
            //�Ӷ���t_control�н�����ٶ�����Ϊһ����Ϊ0��ֵ
            //Ȼ���л�����һ��״̬ToBlueCone
		}
		else if(VisionTaskState == State::ToBlueCone)
		{
            //updateTargetRoute(frame_clone);
            //-------����--------
            IMfr.fetch_add(1);//֡�ʼ���
            std::tuple<double, cv::Vec4f, cv::Vec4f> res = DetectLeftRightLines(frame_clone);
            double error;
            cv::Vec4f left_line_fit, right_line_fit;
            std::tie(error, left_line_fit, right_line_fit) = res;
            lane_error.store(error);
            cv::Mat img = cv::Mat::zeros(96, 320, CV_8UC3);
            drawFitLine(img, left_line_fit, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            drawFitLine(img, right_line_fit, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            //�����е�
            cv::Point target_point(error+160, 60);
            cv::circle(img, target_point, 3, cv::Scalar(255), -1);
            //����Ŀ���е�
            cv::Point now_point(160, 60);
            cv::circle(img, now_point, 4, cv::Scalar(255), -1);
            // --- ����������֡������ ---
            if (!allfinishflag.load())
            {
                g_frame_queue.push(img);
            }
            // ------------------------
            std::lock_guard<std::mutex> lock(g_results_mutex);
            g_results.push_back({ error, left_line_fit, right_line_fit });
            //--------------------
            //��ʱ�����Ѿ���������·�ϻ�����һ��׶Ͱ��
            //������׶Ͱ֮ǰ������ͼ��ʶ���ܵ��������е�������µ�ȫ�ֱ���lane_error��ȥ
            //������׶Ͱ֮��ѡ�����߻����ұ��ƹ�ȥ������滮�õ�·�ߵ������µ�lane_error��ȥ
            //t_control���̻᲻��ѭ��������lane_error��ֵ��������Ƕȣ�ʵ������·����ʻ
            //���ж��Ѿ��ƹ�׶Ͱ֮���л�����һ��״̬ZebraCrossing
		}
		else if(VisionTaskState == State::ToZebraCrossing)
		{
            searchZebraCrossing(frame_clone);
            //��ʱ����·������������
            //������������֮ǰ������ͼ��ʶ���ܵ��������е�������µ�ȫ�ֱ���lane_error��ȥ
            //������������֮����֪ͨt_control����"Stop"(ʹ��send_message("Stop"))
            //���t_control���̻����С��ͣ����������¼�����뱾�����޹أ���
            //����л�����һ��״̬SearchLaneChangeMarker
		}
        else if (VisionTaskState == State::SearchLaneChangeMarker)
        {
            DetectLaneChangeSign(frame_clone);
            //�����������3�뵹��ʱ�����֪ͨt_control����"ReStart",ʹ��t_control�������·���
            //�������������Ҫ����3���ڣ�ʶ������ߺ����ת���־��������ȫ�ֱ���lane_change_direction��ֵ
            //ȫ�ֱ���lane_change_direction��ֵ�����Ǻ궨��LANE_CHANGE_NONE��LANE_CHANGE_LEFT��LANE_CHANGE_RIGHT
            //10��������л�����һ��״̬LaneChange
        }
        else if (VisionTaskState == State::LaneChange)
        {
            PlanLaneChangeRoute(frame_clone);
            //���������Ҫ�ȶ�ȡȫ�ֱ���lane_change_direction��ֵ�����鿴��һ����������ת������ת
            //����lane_change_direction��ֵ�ֱ����ͼ�����滮��·��
            //Ȼ�����滮�õ�·�ߵ������µ�lane_error��ȥ
            //���ж��뿪���������ʱ���л�����һ��״̬ToTheEnd
        }
        else if (VisionTaskState == State::ToTheEnd)
        {
            FollowLaneAndStop(frame_clone);
            //���������Ҫ�ڵ���ͣ������֮ǰ�����ܵ�ѭ���������е�������µ�ȫ�ֱ���lane_error��ȥ
            //ͬʱ����ʶ����Ͽ��ܳ��ֵ�ͣ�������־��ʶ����ɺ󣬰�ʶ�����洢��ȫ�ֱ���stop_area��
            //ȫ�ֱ���stop_area�ĺ궨��ȡֵ������AREA_NONE,AREA_A,AREA_B
            //������ʶ�𵽵�����滮��ͣ������ͣ����·��
            //����Ϊ����ͣ��ʱ����֪ͨt_control����"ReStop"(ʹ��send_message("ReStop"))
        }
        else if(VisionTaskState == State::QR_Code)
        {
            SendCameraFrameToPC(frame_clone);
            //���������Ҫ������ͷ����ش�������
            //Ȼ�����˹�ɨ��
        }
	}
    cap.release();
}


//>>>>>>ʶ����ɫ����<<<<<<<<<<
void isBlueBarrierRemoved(cv::Mat frame_clone)
{
    if (isFindBlueBarrier == false)
    {
        FindBlueBarrier(frame_clone, BlueBarrierConfig);
        isFindBlueBarrier = BlueBarrierConfig.is_bluebarrier_present;
        std::cout << "��δ��⵽��ɫ����" << std::endl;
    }
    else
    {
        if (isRemovedBlueBarrier == false)
        {
            FindBlueBarrier(frame_clone, BlueBarrierConfig);
            isRemovedBlueBarrier = !BlueBarrierConfig.is_bluebarrier_present;
            std::cout << "��⵽��ɫ����" << std::endl;
        }
        else
        {
            //�Ӿ������л�����һ��״̬
            VisionTaskState = State::ToBlueCone;
            //��t_control���̷�����Ϣ
            send_message("Start");
            std::cout << "����" << std::endl;
        }
    }
}
void FindBlueBarrier(cv::Mat frame_clone, BlueBarrierInit& config)
{
    // ����ԭͼһ��������ʾ
    cv::Mat display = frame_clone.clone();
    // ��frame_cloneת��Ϊ HSV ɫ�ʿռ�
    cv::cvtColor(frame_clone, frame_clone, cv::COLOR_BGR2HSV);
    // �� frame_clone ����ȡ ROI ���򣬵õ�һ����ͼ�� roiArea
    // �� roiArea ���޸Ļ�ֱ�������� frame_clone ��
    cv::Mat roiArea = frame_clone(config.roi);
    //����һ������mask������ȡ��ɫ��Ľ��
    cv::Mat mask;
    //��ROI�����з���Ҫ�����ɫ��Ϊ��ɫ
    cv::inRange(roiArea, config.lower_blue, config.upper_blue, mask);
    //ͳ��mask�а�ɫ���صĸ���
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
    //���Թ���
    if (config.enable_debug_display)
    {
        // ---------- ���� mask ��ԭͼ ----------
        // �� mask תΪ��ͨ������ɫ�������ֵ��
        cv::Mat mask_color;
        cv::cvtColor(mask, mask_color, cv::COLOR_GRAY2BGR);

        // ���� ROI ����ԭͼ����
        cv::Mat roi_display = display(config.roi);

        // �ð�ɫ������ԭͼ�ϱ������򵥵��ӣ�
        roi_display.setTo(cv::Scalar(255, 255, 255), mask);

        // ---------- ���� ROI �� ----------
        cv::rectangle(display, config.roi, cv::Scalar(0, 255, 0), 2); // ��ɫ��

        // ---------- ��ʾ��ɫ�������� ----------
        std::string text = "White Pixels: " + std::to_string(white_area);
        cv::putText(display, text, cv::Point(config.roi.x + 5, config.roi.y + 25),
            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        // ---------- ��ʾ����ͼ ----------
        cv::imshow("Blue Barrier Detection", display);
        cv::waitKey(1);
    }

}
//>>>>>�滮����׶Ͱ·��<<<<<<<
void updateTargetRoute(const cv::Mat& frame_clone) 
{
    /**
     * ��������
     * - ������׶Ͱ֮ǰ������ͼ��ʶ���ܵ��������е�������µ�ȫ�ֱ���lane_error��ȥ
     * - ������׶Ͱ֮��ѡ�����߻����ұ��ƹ�ȥ������滮�õ�·�ߵ������µ�lane_error��ȥ
     * - ���ƹ�׶Ͱ֮���л�����һ��״̬ToZebraCrossing
     *
     * ע�����
     *  - lane_error Ϊ�̹߳���ԭ�ӱ�������д���̰߳�ȫ��
     *  - �����߼�ⷽ�����ɺ�������������ѡ��ͼ��������ѧϰ��
     *
     * �������:
     * - �������׽��һ֡ͼ��Ŀ�¡frame_clone
     * 
     * ����ֵ:��
     */
}
//��������

void gammaCorrection(const cv::Mat& input, cv::Mat& output) {
    // ��ͼ��ת��Ϊ�Ҷ�ͼ���Լ���ƽ������
    cv::Mat gray_image;
    cv::cvtColor(input, gray_image, cv::COLOR_BGR2GRAY);
    double mean_intensity = cv::mean(gray_image)[0];

    // ����ƽ������ѡ��٤��ֵ
    double gamma = 0.5 + mean_intensity / 80.0;
    if (gamma > 2) gamma = 2;

    // �������ұ�
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

    // ��ͼ��Ķ����͵ײ���ȡһ�� y�����Ӧ�� x
    float y_top = 0.0f;
    float y_bot = static_cast<float>(img.rows - 1);

    cv::Point2f p1, p2;

    // ����ӽ�ˮƽ/��ֱ�������������� 0
    const float eps = 1e-6f;
    if (std::abs(vy) > eps) {
        // x = x0 + (y - y0) * (vx / vy)
        p1 = cv::Point2f(x0 + (y_top - y0) * (vx / vy), y_top);
        p2 = cv::Point2f(x0 + (y_bot - y0) * (vx / vy), y_bot);
    }
    else {
        // vy �� 0�����򼸺�ˮƽ��ֱ�������ұ߽�� x
        float x_left = 0.0f;
        float x_right = static_cast<float>(img.cols - 1);
        // y = y0 + (x - x0) * (vy / vx)����ʱ vy��0������ y��y0
        p1 = cv::Point2f(x_left, y0);
        p2 = cv::Point2f(x_right, y0);
    }

    // ���߶βü���ͼ��߽��ڣ�����Խ��
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

    // ���ߵ�ƽ��x
    double xavg_left;
    if (has_left) {
        double x1 = calcX(left_line, y1);
        double x2 = calcX(left_line, y2);
        xavg_left = (x1 + x2) / 2.0;
    }
    else {
        xavg_left = 0.0; // ��߽�
    }

    // ���ߵ�ƽ��x
    double xavg_right;
    if (has_right) {
        double x1 = calcX(right_line, y1);
        double x2 = calcX(right_line, y2);
        xavg_right = (x1 + x2) / 2.0;
    }
    else {
        xavg_right = 320.0; // �ұ߽�
    }

    // ��������ƽ��������
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

        if (angle_deg < 0)  angle_deg += 180;  // �ѽǶȷ�Χ������ [0,180)
        // ���ݽǶ�ɸѡ������
        if (angle_deg >= 18 && angle_deg <= 89)
        {
            right_lines.push_back(line);
        }
        else if (angle_deg >= 91 && angle_deg <= 175)
        {
            left_lines.push_back(line);
        }
    }
    //��С���˷�
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
    //��ƽ���е�
    double avgX = calcAverageX(left_line_fit, right_line_fit, has_left, has_right, 30, 60);


    //�������
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
//>>>>>>>>���߼�����ʻֱ��������ͣ��������<<<<<<<<<
void searchZebraCrossing(const cv::Mat& frame_clone) 
{
    //����ͼ����
    cv::Rect roi_rect(0, (frame_clone.rows / 2 - 90 + 60), frame_clone.cols, (frame_clone.rows / 2.5));
    cv::Mat cropped_image = frame_clone(roi_rect);
    gammaCorrection(cropped_image, cropped_image);
    //����ܵ��������д��lane_error
    std::tuple<double, cv::Vec4f, cv::Vec4f> res = DetectLeftRightLinesForZebra(cropped_image);
    double error;
    cv::Vec4f left_line_fit, right_line_fit;
    std::tie(error, left_line_fit, right_line_fit) = res;
    lane_error.store(error);
    //��������
    dect_rxd(cropped_image);
    if (ZebraInformation.final_found_rxd == true)
    {
        std::cout << "Rxd find" << std::endl;
        VisionTaskState = State::SearchLaneChangeMarker;
    }
    /**
     *
     * ��������
     * - ��ʱ����·������������
     * - ������������֮ǰ������ͼ��ʶ���ܵ��������е�������µ�ȫ�ֱ���lane_error��ȥ
     * - ������������֮����֪ͨt_control����"Stop"(ʹ��send_message("Stop"))
     * - ����л�����һ��״̬SearchLaneChangeMarker

     * ע�����
     *  - lane_error Ϊ�̹߳���ԭ�ӱ�������д���̰߳�ȫ��
     *  - �����߼�ⷽ�����ɺ�������������ѡ��ͼ��������ѧϰ��
     *
     * �������:
     * - �������׽��һ֡ͼ��Ŀ�¡frame_clone
     *
     * ����ֵ:��
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

        if (angle_deg < 0)  angle_deg += 180;  // �ѽǶȷ�Χ������ [0,180)
        // ���ݽǶ�ɸѡ������
        if (angle_deg >= 18 && angle_deg <= 89)
        {
            right_lines.push_back(line);
        }
        else if (angle_deg >= 91 && angle_deg <= 175)
        {
            left_lines.push_back(line);
        }
    }
    //��С���˷�
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


    //��ƽ���е�
    double avgX = calcAverageX(left_line_fit, right_line_fit, has_left, has_right, 30, 60);


    //�������
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

    ZebraInformation.now_num_rectangles = 0;    //������һ֡�ҵ��ľ�������

    //����ɸѡ��1���������һ����Χ��2����ȴ���ĳ��ֵ��3����������ĳ��ֵ�����������ж�Ϊ��֡���ְ�����
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
    //������֡���ְ����߲��ж�Ϊ����ҵ�������
    if (ZebraInformation.numbercounter  >= ZebraInformation.confirm_frames){
        ZebraInformation.final_found_rxd = true;
    }
}
//>>>>>>>>>ʶ������־<<<<<<<<<
void DetectLaneChangeSign(const cv::Mat& frame_clone)
{
    //������ʱ������¼��һ�ν����ʱ��
    if (ArrowInformation.time_initialized == false) {
        ArrowInformation.start_time = std::chrono::high_resolution_clock::now();
        ArrowInformation.time_initialized = true;
    }

    //ͼ��Ԥ����
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

    //��¼����ʱ��
    ArrowInformation.now_time = std::chrono::high_resolution_clock::now();
    //�����ʱ
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
        //�ҳ��������roiContour
        auto max_contour = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                return cv::contourArea(a) < cv::contourArea(b);
            });
        cv::Rect boundingBox = cv::boundingRect(*max_contour);
        cv::Mat roiContour = mask_arrow.clone()(boundingBox);
        //ȥ�����Һ��·���������ɫ
        for (int row = 0; row < roiContour.rows; row++) {
            for (int col = 0; col < roiContour.cols; col++) {
                if (roiContour.at<uchar>(row, col) == 0) {  // ��ɫ
                    roiContour.at<uchar>(row, col) = 255;  // ��ɰ�ɫ
                }
                else {
                    break;  // ������ɫ�������ڲ�ѭ�� 
                }
            }
        }

        for (int row = 0; row < roiContour.rows; row++) {
            for (int col = roiContour.cols - 1; col >= 0; col--) {
                if (roiContour.at<uchar>(row, col) == 0) {  // ��ɫ
                    roiContour.at<uchar>(row, col) = 255;  // ��ɰ�ɫ
                }
                else {
                    break;  // ������ɫ�������ڲ�ѭ�� 
                }
            }
        }

        for (int row = roiContour.rows - 1; row >= roiContour.rows - 5; row--) {
            for (int col = roiContour.cols - 1; col >= 0; col--) {
                if (roiContour.at<uchar>(row, col) == 0) {  // ��ɫ
                    roiContour.at<uchar>(row, col) = 255;  // ��ɰ�ɫ
                }
            }
        }
        roiContour = 255 - roiContour;        //��ɫ
        //�ٴ���ȡ����
        std::vector<std::vector<cv::Point>> contours_roi;
        cv::findContours(roiContour, contours_roi, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours_roi.empty()) {
            std::cout << "no arrow flag2" << std::endl;
        }
        else
        {
            //�ҳ��������
            auto contour = *std::max_element(contours_roi.begin(), contours_roi.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a) < cv::contourArea(b);
                });

            double area = cv::contourArea(contour);
            //ɸѡ���������MIN_AREA_THRESHOLD
            if (area > ArrowInformation.MIN_AREA_THRESHOLD)
            {
                //ɸѡ���������ASPECT_RATIO_MIN��ASPECT_RATIO_MAX֮��
                cv::Rect rect = cv::boundingRect(contour);
                float aspect_ratio = (float)rect.width / rect.height;
                if (ArrowInformation.ASPECT_RATIO_MIN < aspect_ratio && aspect_ratio < ArrowInformation.ASPECT_RATIO_MAX)
                {
                    std::vector<cv::Point> approx;
                    double epsilon = 0.03 * cv::arcLength(contour, true);
                    cv::approxPolyDP(contour, approx, epsilon, true);
                    //ɸѡ�������������4�����ҳ���ͷ��β
                    if (approx.size() >= 4)
                    {
                        //��������
                        cv::Moments M = cv::moments(contour);
                        int cx = int(M.m10 / M.m00);
                        int cy = int(M.m01 / M.m00);
                        // ѡ����ͷ�ļ�˺�β����
                        cv::Point tipPoint, tailPoint;
                        double maxTipDistance = 0;
                        double maxTailDistance = 0;
                        for (const cv::Point& p : approx)
                        {
                            double distance = std::sqrt(std::pow(p.x - cx, 2) + std::pow(p.y - cy, 2));
                            // Ѱ����Զ�ļ�ͷ���
                            if (p.y < cy && distance > maxTipDistance) {
                                maxTipDistance = distance;
                                tipPoint = p;
                            }
                            // Ѱ����Զ�ļ�β
                            if (p.y > cy && distance > maxTailDistance) {
                                maxTailDistance = distance;
                                tailPoint = p;
                            }
                        }
                        // �жϼ�ͷ�ķ���
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
//>>>>>>>>>�滮���·��<<<<<<<<<
void PlanLaneChangeRoute(const cv::Mat& frame_clone)
{
    /**
     *
     * ����˵����
     *���������Ҫ�ȶ�ȡȫ�ֱ���lane_change_direction��ֵ�����鿴��һ����������ת������ת
     * - ����lane_change_direction��ֵ�ֱ����ͼ�����滮��·��
     * - Ȼ�����滮�õ�·�ߵ������µ�lane_error��ȥ
     * - ���ж��뿪���������ʱ���л�����һ��״̬ToTheEnd
     *
     * �������:
     * - �������׽��һ֡ͼ��Ŀ�¡frame_clone
     *
     * ����ֵ:��
     */
}
//>>>>>>>>>ͣ��<<<<<<<<<
void FollowLaneAndStop(const cv::Mat& frame_clone)
{
    /**
     *
     * ����˵����
     * - ���������Ҫ�ڵ���ͣ������֮ǰ�����ܵ�ѭ���������е�������µ�ȫ�ֱ���lane_error��ȥ
     * - ͬʱ����ʶ����Ͽ��ܳ��ֵ�ͣ�������־��ʶ����ɺ󣬰�ʶ�����洢��ȫ�ֱ���stop_area��
     * - ȫ�ֱ���stop_area�ĺ궨��ȡֵ������AREA_NONE,AREA_A,AREA_B
     * - ������ʶ�𵽵�����滮��ͣ������ͣ����·��
     * - ����Ϊ����ͣ��ʱ����֪ͨt_control����"ReStop"(ʹ��send_message("ReStop"))
     * 
     * �������:
     * - �������׽��һ֡ͼ��Ŀ�¡frame_clone
     *
     * ����ֵ:��
     */
}
//>>>>>>>>>ʶ���ά��<<<<<<<<<
void SendCameraFrameToPC(const cv::Mat& frame_clone)
{
    /**
     *
     * ����˵����
     * - ���������Ҫ������ͷ����ش�������
     *
     * �������:
     * - �������׽��һ֡ͼ��Ŀ�¡frame_clone
     *
     * ����ֵ:��
     */
}