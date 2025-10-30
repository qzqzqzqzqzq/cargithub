#include "globals.h"

// ==================== 路径循迹相关变量 ====================
std::atomic<int> lane_error = 0;  // 小车中心与车道中心的偏差（像素）

// ==================== 变道功能相关变量 ====================
std::atomic<int> lane_change_direction = LANE_CHANGE_NONE;//用于存储变道方向
// ==================== 停车功能相关变量 ====================
std::atomic<int> stop_area = AREA_NONE;		//用于储存停车区域
// ==================== 结束一切线程相关变量 ====================
std::atomic<bool> allfinishflag = false;