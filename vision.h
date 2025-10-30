#pragma once
#include <opencv2/core.hpp>
#include <tuple>
#include <mutex>

// 声明枚举State，用于表示视觉处理状态
enum class State {
    BlueBarrier,
    ToBlueCone,
    ToZebraCrossing,
    SearchLaneChangeMarker,
    LaneChange,
    ToTheEnd,
    QR_Code,
};
struct BlueBarrierInit
{
    cv::Scalar lower_blue; // HSV下界
    cv::Scalar upper_blue; // HSV上界
    cv::Rect roi;          // 感兴趣区域
    int white_threshold;   // ROI区域中白色像素数量（阈值）

    bool enable_debug_display = false;  // 启用调试显示(默认不启用)

    // 多帧计数器
    int consecutive_detected = 0;//连续发现蓝色挡板的次数
    int consecutive_missed = 0;//连续未发现蓝色挡板的次数

    // 阈值设置（默认3帧触发）
    int detect_threshold_frames = 3;//触发“发现蓝色挡板”
    int miss_threshold_frames = 3;//触发“未发现蓝色挡板”

    // 状态变量：当前是否检测到蓝色挡板（默认false）
    bool is_bluebarrier_present = false;
};
//用于储存锥桶任务信息
struct ConeInfo {
bool isbluerunway = true; //用于选择跑道颜色
bool findcone = false;        // 是否检测到锥桶
bool detection_over = false; // 是否检测结束
int cone_left_x = 0;              // 锥桶左边界
int cone_right_x = 0;             // 锥桶右边界
};
//用于储存斑马线任务信息
struct ZebraInfo {
    // === 逻辑区 ===
    bool final_found_rxd = false; //  最终判定为“找到斑马线”
    int  numbercounter = 0;       //  计数器
    int  confirm_frames = 3;      //  连续多少帧满足条件才算“找到”
    int  area_min = 90;           //  轮廓面积下界
    int  area_max = 1000;         //  轮廓面积上界
    int  num_rectangles = 4;      //  每帧达到多少矩形计为“满足一次”
    int  rect_width = 3;          //   满足条件的矩形的宽度必须大于这个值
    int  now_num_rectangles = 0;  // 本帧找到的矩形数量
    bool  now_found_rxd = false;  // 本帧是认为发现斑马线

    // === 调试区 ===
    bool debug_enabled = true;    // 调试开关
};
//用于储存转向标志识别任务信息
struct ArrowInfo {

    bool isbluerunway = true; //用于选择跑道颜色
    //计数器
    int left_count = 0;    //统计LR检测的次数
    int right_count = 0;
    //筛选参数
    int MIN_AREA_THRESHOLD = 200;
    float ASPECT_RATIO_MAX = 10.0;
    float ASPECT_RATIO_MIN = 0;
    //计时器(斑马线前停留三秒)
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time; // 开始时间
    std::chrono::time_point<std::chrono::high_resolution_clock> now_time;   // 当前时间
    std::chrono::milliseconds time_gap = std::chrono::milliseconds(3000);   // 计时时长
    bool time_initialized = false;                                          // 用于初始化计时器开始时间
};
//用于保存数据
struct FrameData {
    double error;
    cv::Vec4f left_line;
    cv::Vec4f right_line;
};


extern State VisionTaskState;
extern BlueBarrierInit BlueBarrierConfig;
extern ConeInfo ConeInformation;
extern ZebraInfo ZebraInformation;
extern ArrowInfo ArrowInformation;
extern std::vector<FrameData> g_results;
extern std::mutex g_results_mutex;

void vision_loop();
void FindBlueBarrier(cv::Mat frame_clone, BlueBarrierInit& config);
void isBlueBarrierRemoved(cv::Mat frame_clone);
//ToBlueCone任务主函数
void updateTargetRoute(const cv::Mat& frame_clone);
//ToBlueCone任务辅助函数
std::pair<double, double>  calcAverageX_left_right(
    const cv::Vec4f& left_line,
    const cv::Vec4f& right_line,
    bool has_left,
    bool has_right,
    double y1,
    double y2);
std::tuple<double, cv::Vec4f, cv::Vec4f, double, double> DetectLeftRightLinesForCone(cv::Mat& cropped_image);
bool Contour_Area(const std::vector<cv::Point>& contour1, const std::vector<cv::Point>& contour2);
std::pair<int, int> dect_cone(cv::Mat& mask_cone);
//ToZebraCrossing任务主函数
void searchZebraCrossing(const cv::Mat& frame_clone);
//ToZebraCrossing任务辅助函数
std::tuple<double, cv::Vec4f, cv::Vec4f> DetectLeftRightLinesForZebra(cv::Mat& cropped_image);
void dect_rxd(const cv::Mat& cropped_image);
//SearchLaneChangeMarker任务主函数
void DetectLaneChangeSign(const cv::Mat& frame_clone);
//SearchLaneChangeMarker任务辅助函数
void dect_arrow(cv::Mat mask_arrow);

void PlanLaneChangeRoute(const cv::Mat& frame_clone);
void FollowLaneAndStop(const cv::Mat& frame_clone);
void SendCameraFrameToPC(const cv::Mat& frame_clone);

//辅助函数
// 伽马校正函数
void gammaCorrection(const cv::Mat& input, cv::Mat& output);
// 绘制拟合直线
void drawFitLine(cv::Mat& img, const cv::Vec4f& linefit,
    const cv::Scalar& color, int thickness, int lineType);

// 计算左右车道线的平均横坐标
double calcAverageX(const cv::Vec4f& left_line,
    const cv::Vec4f& right_line,
    bool has_left,
    bool has_right,
    double y1,
    double y2);

// 检测左右车道线并返回误差与拟合结果
std::tuple<double, cv::Vec4f, cv::Vec4f> DetectLeftRightLines(cv::Mat& data);
//保存数据
void SaveResultsToCSV(const std::string& filename);