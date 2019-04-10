#include"CBodyBasics.h"
#include"iostream"
#include "mmsystem.h"
#pragma comment(lib,"winmm.lib")

using namespace std;

//一个辅助的用于释放资源的函数
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

//构造函数，需要修改
CBodyBasics::CBodyBasics() :
	m_pBodyFrameReader(NULL),
	m_pKinectSensor(NULL)
{
	//将flag和检查鼠标校准手势相关变量的初始化添加在这里
	bodydetected = 0;
	flaglock = 0;
	flag = -1;

	//将鼠标校准相关变量的初始化添加在这里
	x_min = 0;
	x_max = 0;
	y_min = 0;
	y_max = 0;
	px_min = 0;
	py_min = 0;
	px_max = 0;
	py_max = 0;
	mouselock = 0;
	processing = 0;
	is_leftup_finished = 0;
	is_rightdown_finished = 0;
	is_monitor_tracked = 0;
	monitor_num = 0;
	x_step = 0;
	y_step = 0;

	//将鼠标控制相关变量的初始化添加在这里
	clicklock = 0;

	//将键盘控制相关变量的初始化添加在这里
	lefthandstate = 0;
	righthandstate = 0;
	leftfootstate = 0;
	rightfootstate = 0;
	leftrotationstate = 0;
	rightrotationstate = 0;
	rstate = 0;

}

//析构函数
CBodyBasics::~CBodyBasics()
{
	SafeRelease(m_pBodyFrameReader);
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}


//初始化函数
HRESULT CBodyBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect  and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			cout << "Kinect 传感器已成功初始化..." << endl;
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}
		SafeRelease(pBodyFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		cout << "未找到可用的 Kinect！" << endl;
		return E_FAIL;
	}

	return hr;
}

//Run函数
int CBodyBasics::Run()
{
	InitializeDefaultSensor();
	while (true)
	{
		update();
	}
}


//update函数
void CBodyBasics::update()
{
	if (!m_pBodyFrameReader)
	{
		return;
	}

	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	if (SUCCEEDED(hr))
	{
		IBody* ppBodies[BODY_COUNT] = { 0 };

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}

		if (SUCCEEDED(hr) && flag != 0)
		{
			PreProcess(ppBodies);
		}

		if (SUCCEEDED(hr)  && (!flaglock || flag==0) )
		{
			switch (flag)
			{
			case 0: 
				//cout << "1" << endl;
				MouseCorrect(ppBodies); break;
			case 1: 
				Processbody_1(ppBodies); break;
			case 2: 
				Processbody_2(ppBodies); break;
			default:
				break;
			}
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);
}

//三维坐标结构体
struct coordinate
{
	double x;
	double y;
	double z;
};

//计算斜率，如果flag为1，计算xy平面斜率，如果flag为0，计算yz平面斜率
double gradient(coordinate a, coordinate b, bool flag)
{
	double k;
	if (flag == 1)
	{
		if (a.x == b.x)
		{
			k = 1000;
		}
		else
		{
			k = (b.y - a.y) / (b.x - a.x);
		}
	}
	else
	{
		if (a.z == b.z)
		{
			k = 1000;
		}
		else
		{
			k = (b.y - a.y) / (b.z - a.z);
		}
	}

	return k;
}

int a = 0;

//将处理flag和检查鼠标校准手势的具体实现过程添加在这里
void CBodyBasics::PreProcess(IBody** ppBodies)
{
	if (bodydetected == 0)
	{
		a = a + 1;
		cout << a << endl;
	}
	HRESULT hr;
	IBody* pbody = ppBodies[0];
	if (pbody)
	{
		BOOLEAN bTracked = false;
		hr = pbody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			if (bodydetected == 0)
			{
				bodydetected = 1;
				cout << "成功检测到人体..." << endl;
				cout << "下面开始首次鼠标校准..." << endl;
				PlaySound(TEXT(".\\Audio\\body_detected.wav"), NULL, SND_FILENAME | SND_SYNC);
				flag = 0;
				return;
			}

			//检测代码
			//高举右手以将flag改为1/2，高举左手以将flag改为0			
			double k_threshold = 5;

			Joint joints[JointType_Count];
			hr = pbody->GetJoints(_countof(joints), joints);

			coordinate left[3], right[3];
			left[0].x = joints[JointType_HandLeft].Position.X;
			left[0].y = joints[JointType_HandLeft].Position.Y;
			left[0].z = joints[JointType_HandLeft].Position.Z;
			left[1].x = joints[JointType_ElbowLeft].Position.X;
			left[1].y = joints[JointType_ElbowLeft].Position.Y;
			left[1].z = joints[JointType_ElbowLeft].Position.Z;
			left[2].x = joints[JointType_ShoulderLeft].Position.X;
			left[2].y = joints[JointType_ShoulderLeft].Position.Y;
			left[2].z = joints[JointType_ShoulderLeft].Position.Z;
			right[0].x = joints[JointType_HandRight].Position.X;
			right[0].y = joints[JointType_HandRight].Position.Y;
			right[0].z = joints[JointType_HandRight].Position.Z;
			right[1].x = joints[JointType_ElbowRight].Position.X;
			right[1].y = joints[JointType_ElbowRight].Position.Y;
			right[1].z = joints[JointType_ElbowRight].Position.Z;
			right[2].x = joints[JointType_ShoulderRight].Position.X;
			right[2].y = joints[JointType_ShoulderRight].Position.Y;
			right[2].z = joints[JointType_ShoulderRight].Position.Z;

			if (right[1].y > right[2].y)
			{
				double k1_xy = abs(gradient(right[0], right[1], 1));
				double k1_yz = abs(gradient(right[0], right[1], 0));
				double k2_xy = abs(gradient(right[0], right[2], 1));
				double k2_yz = abs(gradient(right[0], right[2], 0));
				if (k1_xy>k_threshold && k1_yz>k_threshold && k2_xy>k_threshold && k2_yz>k_threshold && flaglock == 0)
				{
					flaglock = 1;
					if (flag == 1)
					{

						flag = 2;
						cout << "已进入游戏控制模式。" << endl;
						PlaySound(TEXT(".\\Audio\\game_mode.wav"), NULL, SND_FILENAME | SND_ASYNC);
					}
					else if (flag == 2)
					{
						flag = 1;
						cout << "已进入鼠标控制模式。" << endl;
						PlaySound(TEXT(".\\Audio\\mouse_mode.wav"), NULL, SND_FILENAME | SND_ASYNC);
					}
				}
			}
			else if (left[1].y > left[2].y)
			{
				double k1_xy = abs(gradient(left[0], left[1], 1));
				double k1_yz = abs(gradient(left[0], left[1], 0));
				double k2_xy = abs(gradient(left[0], left[2], 1));
				double k2_yz = abs(gradient(left[0], left[2], 0));
				if (k1_xy>k_threshold && k1_yz>k_threshold && k2_xy>k_threshold && k2_yz>k_threshold && flaglock == 0)
				{
					flag = 0;
					flaglock = 1;
					cout << "进入鼠标校准模式..." << endl;
					PlaySound(TEXT(".\\Audio\\mouse_correcting.wav"), NULL, SND_FILENAME | SND_SYNC);
				}
			}
			else if (flaglock == 1)
			{
				flaglock = 0;
				//cout << "0" << endl;
			}
		}
	}
}

//将处理鼠标校准的实现过程添加在这里
int CALLBACK CBodyBasics::MonitorEnumProc(HMONITOR hMonitor, HDC hdc, LPRECT lpRMonitor, LPARAM dwData)
{
	hm.push_back(hMonitor);
	return 1;
}

void CBodyBasics::MouseCorrect(IBody** ppBodies)
{
	if (processing == 0)
	{
		cout << "请将右手举起至到左上方较为舒适的位置，然后将右手张开...";
		PlaySound(TEXT(".\\Audio\\left_up.wav"), NULL, SND_FILENAME | SND_ASYNC);
		processing = 1;
	}

	HRESULT hr;
	IBody* pbody = ppBodies[0];
	if (pbody)
	{
		BOOLEAN bTracked = false;
		hr = pbody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			HandState rightHandState = HandState_Unknown;


			hr = pbody->GetJoints(_countof(joints), joints);
			pbody->get_HandRightState(&rightHandState);


			if (!is_leftup_finished)
			{
				if (rightHandState == HandState_Open && mouselock == 0)
				{
					x_min = joints[JointType_HandRight].Position.X;
					y_max = joints[JointType_HandRight].Position.Y;
					cout << "完成" << endl;
					PlaySound(TEXT(".\\Audio\\finish_temp.wav"), NULL, SND_FILENAME | SND_SYNC);
					cout << "请将右手举起至到右下方较为舒适的位置，然后将右手张开...";
					PlaySound(TEXT(".\\Audio\\right_down.wav"), NULL, SND_FILENAME | SND_ASYNC);
					mouselock = 1;
					is_leftup_finished = 1;
				}
			}
			else if (!is_rightdown_finished)
			{
				if (rightHandState == HandState_Open && mouselock == 0)
				{
					x_max = joints[JointType_HandRight].Position.X;
					y_min = joints[JointType_HandRight].Position.Y;
					mouselock = 1;
					is_rightdown_finished = 1;
					cout << "完成" << endl;
					PlaySound(TEXT(".\\Audio\\finish_temp.wav"), NULL, SND_FILENAME | SND_SYNC);
					cout << x_min << " " << x_max << " " << y_min << " " << y_max << endl;
				}
				else if (rightHandState == HandState_Closed)
				{
					mouselock = 0;
				}
			}
			else
			{
				HandState leftHandState = HandState_Unknown;
				pbody->get_HandLeftState(&leftHandState);
				//获取显示器信息的相关代码
				if (!is_monitor_tracked)
				{
					hm.clear();
					::EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, 0);
					is_monitor_tracked = 1;

					//如果只有一个显示器，就不需要这个操作
					if (hm.size() == 1)
					{
						rightHandState = HandState_Open;
						mouselock = 0;
					}
					else
					{
						cout << "请问鼠标现在是否在游戏所在屏幕上移动?" << endl;
						cout << " 张开右手表示在，张开左手表示不在...";
						PlaySound(TEXT(".\\Audio\\choose_monitor.wav"), NULL, SND_FILENAME | SND_ASYNC);
					}
				}

				MONITORINFO mixTemp;
				mixTemp.cbSize = sizeof(MONITORINFO);
				GetMonitorInfo(hm[monitor_num], &mixTemp);
				RECT REC = mixTemp.rcMonitor;

				if (mixTemp.dwFlags == MONITORINFOF_PRIMARY) //是否是主显示器
				{
					x_step = 65536.0 / (REC.right - REC.left);
					y_step = 65536.0 / (REC.bottom - REC.top);
				}

				//鼠标移动相关代码
				int y_tem = (REC.bottom + REC.top) / 2 + REC.top;
				for (size_t i = REC.left; i < REC.right; i = i + 2)
				{
					mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, i*x_step, y_tem*y_step, 0, 0);
				}

				if (rightHandState == HandState_Open && mouselock == 0)
				{
					px_min = REC.left; px_max = REC.right;
					py_min = REC.bottom; py_max = REC.top;
					cout << px_min << " " << px_max << " " << py_min << " " << py_max << endl;
					//成功的一系列输出
					cout << "校准成功！您可以高举左手以重新校准。" << endl;
					cout << "校准后默认进入鼠标操作模式..." << endl;
					cout << "您可以高举右手以进入游戏操作模式。" << endl;
					PlaySound(TEXT(".\\Audio\\finish_all.wav"), NULL, SND_FILENAME | SND_ASYNC);

					flag = 1;

					processing = 0;
					is_leftup_finished = 0;
					is_rightdown_finished = 0;
					is_monitor_tracked = 0;
					monitor_num = 0;

				}
				else if (leftHandState == HandState_Open && mouselock == 0)
				{
					mouselock = 1;
					if (monitor_num == hm.size() - 1)
					{
						monitor_num = 0;
					}
					else
					{
						monitor_num += 1;
					}
					cout << "该显示器已排除..." << endl;
					PlaySound(TEXT(".\\Audio\\exclude.wav"), NULL, SND_FILENAME | SND_SYNC);
					cout << "请问鼠标现在是否在游戏所在屏幕上移动?" << endl;
					cout << " 张开右手表示在，张开左手表示不在...";
					PlaySound(TEXT(".\\Audio\\choose_monitor.wav"), NULL, SND_FILENAME | SND_ASYNC);
					//失败，切换至下一个显示器
				}
				else if (rightHandState == HandState_Closed && leftHandState == HandState_Closed)
				{
					mouselock = 0;
				}
			}
		}
	}

}


//将实现鼠标控制的具体过程添加在这里
//ps：processbody_1中不再需要那句“body_detected”输出（153~157行）
//校准坐标：x_min, x_max, y_min, y_max;ASFD
//获取的屏幕分辨率：px_min, px_max, py_min, py_max;
void CBodyBasics::Processbody_1(IBody** ppbodies)
{
	//将当前的x,y坐标转换为屏幕上的绝对坐标
	HRESULT hr;
	IBody* pbody = ppbodies[0];
	if (pbody)
	{
		BOOLEAN bTracked = false;
		hr = pbody->get_IsTracked(&bTracked);

		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			HandState rightHandState = HandState_Unknown;
			HandState leftHandState = HandState_Unknown;
			hr = pbody->GetJoints(_countof(joints), joints);
			pbody->get_HandRightState(&rightHandState);
			pbody->get_HandLeftState(&leftHandState);
			joint_x = joints[JointType_HandRight].Position.X;
			joint_y = joints[JointType_HandRight].Position.Y;
			//通过左手状态来操控滚轮
			coordinate left, right;
			left.x = joints[JointType_HandLeft].Position.X;
			left.y = joints[JointType_HandLeft].Position.Y;
			left.z = joints[JointType_HandLeft].Position.Z;
			right.x = joints[JointType_HandRight].Position.X;
			right.y = joints[JointType_HandRight].Position.Y;
			right.z = joints[JointType_HandRight].Position.Z;
			double k_hand = gradient(left, right, 1);
			if (leftHandState == HandState_Open)
			{
				if (k_hand < -0.3)
					mouse_event(MOUSEEVENTF_WHEEL, 0, 0, 50, 0);
				else if (k_hand > 0.3)
					mouse_event(MOUSEEVENTF_WHEEL, 0, 0, -50, 0);
			}
			//通过右手状态来控制鼠标移动和点击
			if (joint_x < x_min)
			{
				joint_x = x_min;
			}
			else if (joint_x > x_max)
			{
				joint_x = x_max;
			}
			if (joint_y < y_min)
			{
				joint_y = y_min;
			}
			else if (joint_y > y_max)
			{
				joint_y = y_max;
			}
			

			int mx1 = int((joint_x - x_min)*(px_max - px_min) / (x_max - x_min) + px_min);
			int my1 = int((joint_y - y_min)*(py_max - py_min) / (y_max - y_min) + py_min);
			//cout << mx1 << " " << my1 << endl;
			int mx = mx1*x_step;
			int my = my1*y_step;
			if (rightHandState == HandState_Open)
			{
				mouse_event(MOUSEEVENTF_ABSOLUTE | MOUSEEVENTF_MOVE, mx, my, 0, 0);
			}
			if (rightHandState == HandState_Closed && clicklock == 0)
			{
				mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, GetMessageExtraInfo());
				mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, GetMessageExtraInfo());
				clicklock = 1;
				cout << "屏幕像素坐标(" << mx1 << "," << my1 << ")Clicked!" << endl;
			}
			else if (rightHandState == HandState_Open)
			{
				clicklock = 0;

			}

		}
	}
}


//将实现键盘控制的具体过程添加在这里
//ps：processbody_2中不再需要那句“body_detected”输出（153~157行）
void CBodyBasics::Processbody_2(IBody** ppbodies)
{
	HRESULT hr;
	IBody* pbody = ppbodies[0];//取第一个人的信息
	if (pbody)
	{
		BOOLEAN bTracked = false;
		hr = pbody->get_IsTracked(&bTracked);
		if (SUCCEEDED(hr) && bTracked)
		{
			Joint joints[JointType_Count];
			HandState leftHandState = HandState_Unknown;
			HandState rightHandState = HandState_Unknown;
			pbody->get_HandLeftState(&leftHandState);
			pbody->get_HandRightState(&rightHandState);

			hr = pbody->GetJoints(_countof(joints), joints);

			//左手张开切换视角
			if (leftHandState == HandState_Open && lefthandstate == 0)
			{
				keybd_event(0x43, 0, 0, 0);
				lefthandstate = 1;
				cout << "切换视角" << endl;
			}
			else if (leftHandState == HandState_Closed && lefthandstate == 1)
			{
				lefthandstate = 0;
				keybd_event(0x43, 0, KEYEVENTF_KEYUP, 0);
			}

			//右手张开暂停游戏
			if (rightHandState == HandState_Open && righthandstate == 0)
			{
				keybd_event(0x1B, 0, 0, 0);
				keybd_event(0x1B, 0, KEYEVENTF_KEYUP, 0);
				righthandstate = 1;
				cout << "暂停游戏" << endl;
			}
			else if (rightHandState == HandState_Closed && righthandstate == 1)
			{
				righthandstate = 0;
			}

			//手斜率控制方向盘 左脚斜率控制刹车 右脚斜率控制氮气加速
			double k_hand_threshold = 0.5;
			double k_foot_threshold_up = 3;
			double k_foot_threshold_down = 2;
			coordinate leftHand, rightHand, leftFoot, rightFoot, leftKnee, rightKnee, neck, leftelbow, rightelbow;
			leftHand.x = joints[JointType_HandLeft].Position.X;
			leftHand.y = joints[JointType_HandLeft].Position.Y;
			rightHand.x = joints[JointType_HandRight].Position.X;
			rightHand.y = joints[JointType_HandRight].Position.Y;

			leftelbow.x = joints[JointType_ElbowLeft].Position.X;
			leftelbow.y = joints[JointType_ElbowLeft].Position.Y;
			
			rightelbow.x = joints[JointType_ElbowRight].Position.X;
			rightelbow.y = joints[JointType_ElbowRight].Position.Y;

			leftFoot.z = joints[JointType_AnkleLeft].Position.Z;
			leftFoot.y = joints[JointType_AnkleLeft].Position.Y;
			rightFoot.z = joints[JointType_AnkleRight].Position.Z;
			rightFoot.y = joints[JointType_AnkleRight].Position.Y;

			leftKnee.z = joints[JointType_KneeLeft].Position.Z;
			leftKnee.y = joints[JointType_KneeLeft].Position.Y;
			rightKnee.z = joints[JointType_KneeRight].Position.Z;
			rightKnee.y = joints[JointType_KneeRight].Position.Y;

			//R键复位
			double k_lefthand = gradient(leftHand, leftelbow, 1);
			double k_righthand = gradient(rightHand, rightelbow, 1);
			//cout << k_lefthand << " " << k_righthand << endl;
			if (abs(k_lefthand) > 4 && abs(k_righthand) > 4 && rstate == 0)
			{
				keybd_event(82, 0, 0, 0);
				rstate = 1;
				cout << "复位" << endl;
			}
			else if (abs(k_lefthand) < 4 && abs(k_righthand) < 4)
			{
				rstate = 0;
				keybd_event(82, 0, KEYEVENTF_KEYUP, 0);
			}

			//方向盘控制
			double k_hand = gradient(leftHand, rightHand, 1);
			if (abs(k_hand) > k_hand_threshold)
			{
				if (rightHand.y > leftHand.y && leftrotationstate == 0)
				{
					keybd_event(0x41, 0, 0, 0);
					leftrotationstate = 1;
					cout << "左转" << endl;
				}
				if (rightHand.y < leftHand.y && rightrotationstate == 0)
				{
					keybd_event(0x44, 0, 0, 0);
					rightrotationstate = 1;
					cout << "右转" << endl;
				}
			}
			else
			{
				if (leftrotationstate == 1)
				{
					keybd_event(0x41, 0, KEYEVENTF_KEYUP, 0);
					leftrotationstate = 0;
					cout << "停止左转" << endl;
				}
				if (rightrotationstate == 1)
				{
					keybd_event(0x44, 0, KEYEVENTF_KEYUP, 0);
					rightrotationstate = 0;
					cout << "停止右转" << endl;
				}
			}

			//左脚刹车控制
			double k_leftfoot = abs( gradient(leftKnee, leftFoot, 0) );
			//cout <<  k_leftfoot  << endl;
			if ( k_leftfoot < k_foot_threshold_down && leftfootstate == 0)
			{
				keybd_event(0x53, 0, 0, 0);
				leftfootstate = 1;
				cout << "刹车" << endl;

			}
			if ( k_leftfoot > k_foot_threshold_up && leftfootstate == 1)
			{
				keybd_event(0x53, 0, KEYEVENTF_KEYUP, 0);
				leftfootstate = 0;
				cout << "停止刹车" << endl;
			}

			//右脚氮气加速控制
			double k_rightfoot = abs( gradient(rightKnee, rightFoot, 0) );
			if (k_rightfoot < k_foot_threshold_down && rightfootstate == 0)
			{
				keybd_event(0x46, 0, 0, 0);
				//keybd_event(0x46, 0, KEYEVENTF_KEYUP, 0);
				rightfootstate = 1;
				cout << "氮气加速" << endl;
			}
			else if (k_rightfoot > k_foot_threshold_up && rightfootstate == 1)
			{
				rightfootstate = 0;
				keybd_event(0x46, 0, KEYEVENTF_KEYUP, 0);

			}

		}

	}


}