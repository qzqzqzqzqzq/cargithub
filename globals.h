#pragma once
#include <atomic>
// ==================== ·��ѭ����ر��� ====================
//
// ȫ�ֱ��� lane_error
// - ���ͣ�std::atomic<int>
// - ���ܣ��洢������ǰ��Ŀ��·�ߵ�ƫ��ֵ
// - ��;���ṩ�� PID ����ģ�飬����ƫ��ֵʵʱ��������Ƕȣ�
//        ʹ�����ܹ�����Ԥ��·���ȶ���ʻ
// - ˵����lane_error ��ֵ���Ӿ��̲߳��ϸ��£������̶߳�ȡʹ��
//
// ===========================================================
extern std::atomic<int> lane_error;
// ==================== ���������ر��� ====================
//
// - ʹ�ú궨���ʾ�������
//   LANE_CHANGE_NONE  = 0 ��Ĭ��ֵ����ʾδ���������
//   LANE_CHANGE_LEFT  = 1 ����ʾ��Ҫ������
//   LANE_CHANGE_RIGHT = 2 ����ʾ��Ҫ�ұ����
//
// - ȫ�ֱ��� lane_change_direction ���ڴ洢��ǰ�������
//   ����Ϊ std::atomic<int>��ȷ�����̷߳���ʱ�̰߳�ȫ
//   ��ʼֵΪ LANE_CHANGE_NONE (0)����ʾδ�������
//
// ==========================================================

#define LANE_CHANGE_NONE   0
#define LANE_CHANGE_LEFT   1
#define LANE_CHANGE_RIGHT  2

extern std::atomic<int> lane_change_direction;
// ==================== ͣ��������ر��� ====================
#define AREA_NONE	0
#define	AREA_A		1
#define AREA_B		2

extern std::atomic<int> stop_area;
// ==================== ����һ���߳���ر��� ====================
extern std::atomic<bool> allfinishflag;