#pragma once

#include <windows.h>
#include <Shlobj.h>
#include <Kinect.h>
#include <vector>

static std::vector<HMONITOR> hm;

class CBodyBasics
{
public:
	CBodyBasics();
	~CBodyBasics();
	int Run();
private:
	IKinectSensor*          m_pKinectSensor;
	IBodyFrameReader*       m_pBodyFrameReader;

	void update();
	HRESULT InitializeDefaultSensor();


	//������flag�ͼ�����У׼���Ƶ�����������������(�貹����������
	bool bodydetected;
	bool flaglock;
	short flag;
	void PreProcess(IBody** ppBodies);


	//�����У׼������������������(�貹����������
	double x_min, x_max, y_min, y_max;
	int px_min, px_max, py_min, py_max;
	double joint_x, joint_y;
	double x_step, y_step;
	bool is_leftup_finished,is_rightdown_finished;
	bool mouselock;
	bool processing;
	bool is_monitor_tracked;
	short monitor_num;
	static int CALLBACK MonitorEnumProc(HMONITOR hMonitor, HDC hdc, LPRECT lpRMonitor, LPARAM dwData);
	void MouseCorrect(IBody** ppBodies);



	//�������Ƶ�����������������(�貹����������
	//ps�����У׼���ֶ���� x_min,x_max,y_min,y_max ��ΪУ׼�Ľ��
	bool clicklock;
	void Processbody_1(IBody** ppBodies);


	//�����̿��Ƶ�����������������貹����������
	void Processbody_2(IBody** ppBodies);
	bool lefthandstate;
	bool righthandstate;
	bool leftfootstate;
	bool rightfootstate;
	bool leftrotationstate;
	bool rightrotationstate;
	bool rstate;
};