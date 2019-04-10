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


	//将处理flag和检查鼠标校准手势的相关内容添加在这里(需补充锁变量）
	bool bodydetected;
	bool flaglock;
	short flag;
	void PreProcess(IBody** ppBodies);


	//将鼠标校准的相关内容添加在这里(需补充锁变量）
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



	//将鼠标控制的相关内容添加在这里(需补充锁变量）
	//ps：鼠标校准部分定义的 x_min,x_max,y_min,y_max 即为校准的结果
	bool clicklock;
	void Processbody_1(IBody** ppBodies);


	//将键盘控制的相关内容添加在这里（需补充锁变量）
	void Processbody_2(IBody** ppBodies);
	bool lefthandstate;
	bool righthandstate;
	bool leftfootstate;
	bool rightfootstate;
	bool leftrotationstate;
	bool rightrotationstate;
	bool rstate;
};