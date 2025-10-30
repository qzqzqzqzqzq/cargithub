#pragma once
#include <opencv2/core.hpp>
#include <tuple>
#include <mutex>

// ����ö��State�����ڱ�ʾ�Ӿ�����״̬
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
    cv::Scalar lower_blue; // HSV�½�
    cv::Scalar upper_blue; // HSV�Ͻ�
    cv::Rect roi;          // ����Ȥ����
    int white_threshold;   // ROI�����а�ɫ������������ֵ��

    bool enable_debug_display = false;  // ���õ�����ʾ(Ĭ�ϲ�����)

    // ��֡������
    int consecutive_detected = 0;//����������ɫ����Ĵ���
    int consecutive_missed = 0;//����δ������ɫ����Ĵ���

    // ��ֵ���ã�Ĭ��3֡������
    int detect_threshold_frames = 3;//������������ɫ���塱
    int miss_threshold_frames = 3;//������δ������ɫ���塱

    // ״̬��������ǰ�Ƿ��⵽��ɫ���壨Ĭ��false��
    bool is_bluebarrier_present = false;
};
//���ڴ���׶Ͱ������Ϣ
struct ConeInfo {
bool isbluerunway = true; //����ѡ���ܵ���ɫ
bool findcone = false;        // �Ƿ��⵽׶Ͱ
bool detection_over = false; // �Ƿ������
int cone_left_x = 0;              // ׶Ͱ��߽�
int cone_right_x = 0;             // ׶Ͱ�ұ߽�
};
//���ڴ��������������Ϣ
struct ZebraInfo {
    // === �߼��� ===
    bool final_found_rxd = false; //  �����ж�Ϊ���ҵ������ߡ�
    int  numbercounter = 0;       //  ������
    int  confirm_frames = 3;      //  ��������֡�����������㡰�ҵ���
    int  area_min = 90;           //  ��������½�
    int  area_max = 1000;         //  ��������Ͻ�
    int  num_rectangles = 4;      //  ÿ֡�ﵽ���پ��μ�Ϊ������һ�Ρ�
    int  rect_width = 3;          //   ���������ľ��εĿ�ȱ���������ֵ
    int  now_num_rectangles = 0;  // ��֡�ҵ��ľ�������
    bool  now_found_rxd = false;  // ��֡����Ϊ���ְ�����

    // === ������ ===
    bool debug_enabled = true;    // ���Կ���
};
//���ڴ���ת���־ʶ��������Ϣ
struct ArrowInfo {

    bool isbluerunway = true; //����ѡ���ܵ���ɫ
    //������
    int left_count = 0;    //ͳ��LR���Ĵ���
    int right_count = 0;
    //ɸѡ����
    int MIN_AREA_THRESHOLD = 200;
    float ASPECT_RATIO_MAX = 10.0;
    float ASPECT_RATIO_MIN = 0;
    //��ʱ��(������ǰͣ������)
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time; // ��ʼʱ��
    std::chrono::time_point<std::chrono::high_resolution_clock> now_time;   // ��ǰʱ��
    std::chrono::milliseconds time_gap = std::chrono::milliseconds(3000);   // ��ʱʱ��
    bool time_initialized = false;                                          // ���ڳ�ʼ����ʱ����ʼʱ��
};
//���ڱ�������
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
//ToBlueCone����������
void updateTargetRoute(const cv::Mat& frame_clone);
//ToBlueCone����������
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
//ToZebraCrossing����������
void searchZebraCrossing(const cv::Mat& frame_clone);
//ToZebraCrossing����������
std::tuple<double, cv::Vec4f, cv::Vec4f> DetectLeftRightLinesForZebra(cv::Mat& cropped_image);
void dect_rxd(const cv::Mat& cropped_image);
//SearchLaneChangeMarker����������
void DetectLaneChangeSign(const cv::Mat& frame_clone);
//SearchLaneChangeMarker����������
void dect_arrow(cv::Mat mask_arrow);

void PlanLaneChangeRoute(const cv::Mat& frame_clone);
void FollowLaneAndStop(const cv::Mat& frame_clone);
void SendCameraFrameToPC(const cv::Mat& frame_clone);

//��������
// ٤��У������
void gammaCorrection(const cv::Mat& input, cv::Mat& output);
// �������ֱ��
void drawFitLine(cv::Mat& img, const cv::Vec4f& linefit,
    const cv::Scalar& color, int thickness, int lineType);

// �������ҳ����ߵ�ƽ��������
double calcAverageX(const cv::Vec4f& left_line,
    const cv::Vec4f& right_line,
    bool has_left,
    bool has_right,
    double y1,
    double y2);

// ������ҳ����߲������������Ͻ��
std::tuple<double, cv::Vec4f, cv::Vec4f> DetectLeftRightLines(cv::Mat& data);
//��������
void SaveResultsToCSV(const std::string& filename);