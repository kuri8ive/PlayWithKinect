#pragma once
#include <Windows.h>
#include <NuiApi.h>
#include <FaceTrackLib.h>
#include <opencv2/opencv.hpp>
#include <sstream>

inline HRESULT ERROR_CHECK(int ret) {
	if(ret != S_OK) {		
		std::stringstream ss;		
		ss << "ERROR_CHECK: " << std::hex << ret << std::endl;	
		throw std::runtime_error(ss.str().c_str());		
	}
}

class CKinectControl
{
public:
	CKinectControl(void);
	~CKinectControl(void);
	int initializeSensor(void);
	int initializeFaceTracker(void);
	void run(void);
	void processRGBImage(void);
	void processDepthImage(void);
	void processFaceTracker(void);
	void classifyFaceExpression(FT_VECTOR2D *pointlist, cv::Point topleft,cv::Point bottomright);

	DWORD widthColor;  // 640
	DWORD heightColor; // 480

	DWORD widthDepth;  // 320
	DWORD heightDepth; // 240

private:
	INuiSensor* kinect;
	HANDLE rgbHandle;
	HANDLE depthHandle;
	HANDLE streamEvent;

	IFTImage* pColorFrame;
	IFTImage* pDepthFrame;
	FT_CAMERA_CONFIG videoCameraConfig;
	FT_CAMERA_CONFIG depthCameraConfig;
	IFTFaceTracker* pFaceTracker;
	IFTResult *pFTResult;
	FT_SENSOR_DATA sensorData;
	

	bool isFaceTracked;
	const int LIPLEFT = 48;
	const int LIPRIGHT = 54;
	const int LIPTOP = 51;
	const int LIPBOTTOM = 57;
	const double SMILERATIO = 2.4;
	const double SURPRISERATIO = 4.6;

	cv::Mat rgbIm;
	//cv::Mat depthIm;
	//cv::Mat playerIm;
};

