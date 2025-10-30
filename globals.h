#pragma once
#include <atomic>
// ==================== 路径循迹相关变量 ====================
//
// 全局变量 lane_error
// - 类型：std::atomic<int>
// - 功能：存储车辆当前与目标路线的偏差值
// - 用途：提供给 PID 控制模块，根据偏差值实时调整舵机角度，
//        使车辆能够沿着预定路线稳定行驶
// - 说明：lane_error 的值由视觉线程不断更新，控制线程读取使用
//
// ===========================================================
extern std::atomic<int> lane_error;
// ==================== 变道功能相关变量 ====================
//
// - 使用宏定义表示变道方向
//   LANE_CHANGE_NONE  = 0 （默认值，表示未触发变道）
//   LANE_CHANGE_LEFT  = 1 （表示需要左变道）
//   LANE_CHANGE_RIGHT = 2 （表示需要右变道）
//
// - 全局变量 lane_change_direction 用于存储当前变道方向
//   类型为 std::atomic<int>，确保多线程访问时线程安全
//   初始值为 LANE_CHANGE_NONE (0)，表示未触发变道
//
// ==========================================================

#define LANE_CHANGE_NONE   0
#define LANE_CHANGE_LEFT   1
#define LANE_CHANGE_RIGHT  2

extern std::atomic<int> lane_change_direction;
// ==================== 停车功能相关变量 ====================
#define AREA_NONE	0
#define	AREA_A		1
#define AREA_B		2

extern std::atomic<int> stop_area;
// ==================== 结束一切线程相关变量 ====================
extern std::atomic<bool> allfinishflag;