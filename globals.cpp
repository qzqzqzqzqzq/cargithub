#include "globals.h"

// ==================== ·��ѭ����ر��� ====================
std::atomic<int> lane_error = 0;  // С�������복�����ĵ�ƫ����أ�

// ==================== ���������ر��� ====================
std::atomic<int> lane_change_direction = LANE_CHANGE_NONE;//���ڴ洢�������
// ==================== ͣ��������ر��� ====================
std::atomic<int> stop_area = AREA_NONE;		//���ڴ���ͣ������
// ==================== ����һ���߳���ر��� ====================
std::atomic<bool> allfinishflag = false;