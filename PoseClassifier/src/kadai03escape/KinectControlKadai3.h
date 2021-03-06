#pragma once

#include <iostream>
#include <sstream>

#include <Windows.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>



#define _USE_MATH_defINEs
#include <math.h>

#define ERROR_CHECK(ret)		\
	if(ret != S_OK){		\
	std::stringstream ss;		\
	ss << "failed " #ret "" << std::hex << ret << std::endl;	\
	throw std::runtime_error(ss.str().c_str());		\
	}

const NUI_IMAGE_RESOLUTION CAMERA_RESOLUTION = NUI_IMAGE_RESOLUTION_640x480;


const int PROCESS_NONE = 0;
const int PROCESS_SAVEPOSE1 = 1;
const int PROCESS_SAVEPOSE2 = 2;
const int PROCESS_SAVEPOSE3 = 3;
const int PROCESS_SAVEPOSE4 = 4;
const int PROCESS_RECOGNITION = 5;
const int MAXFRAME = 20;

class KinectControl
{
public:
	KinectControl(void);
	~KinectControl(void);

	void initialize();
	void run();

private:
	typedef struct _NUI_SKELETON_FRAME {
		LARGE_INTEGER liTimeStamp;
		DWORD dwFrameNumber;
		DWORD dwFlags;
		Vector4 vFloorClipPlane;
		Vector4 vNormalToGravity;
		NUI_SKELETON_DATA SkeletonData[NUI_SKELETON_COUNT];
	} NUI_SKELETON_FRAME;

	typedef struct _NUI_SKELETON_DATA {
		NUI_SKELETON_TRACKING_STATE eTrackingState;
		DWORD dwTrackingID;
		DWORD dwEnrollmentIndex;
		DWORD dwUserIndex;
		Vector4 Position;
		Vector4 SkeletonPositions[20];
		NUI_SKELETON_POSITION_TRACKING_STATE eSkeletonPositionTrackingState[20];
		DWORD dwQualityFlags;
	} NUI_SKELETON_DATA;


	INuiSensor *kinect;
	//NUI_SKELETON_DATA trackedSkeleton;
	NUI_SKELETON_DATA skeleton;
	NUI_SKELETON_DATA skeletonPositions;

	HANDLE imageStreamHandle; //カラー画像のイベント
	HANDLE depthStreamHandle; //距離画像のイベント
	HANDLE streamEvent;

	DWORD width;
	DWORD height;

	void createInstance(); //キネクトの初期化
	void setRgbImage(); //カラー画像の取得
	void setDepthImage(); //距離画像の取得
	void setPlayerIndex(USHORT* depth); //ユーザーインデックス
	void setSkeletonImage();

	void drawTrackedSkeleton(cv::Mat& image, const NUI_SKELETON_DATA& skeleton);
	void drawBone(cv::Mat& image, const NUI_SKELETON_DATA & skeleton, NUI_SKELETON_POSITION_INDEX jointFrom, NUI_SKELETON_POSITION_INDEX jointTo);
	void drawLine(cv::Mat& image, Vector4 pos1, Vector4 pos2);
	void drawPoint(cv::Mat& image, Vector4 position);
	void SaveSkeleton();

	//必要があれば、新しい関数や変数を作りましょう

	cv::Mat rgbIm; //カラー画像行列
	cv::Mat depthIm; //距離画像行列
	cv::Mat playerIm; //ユーザーインデックス
	cv::Mat skeletonIm; //スケルトンデータ

	int processFlag;
	int savedFrameIdx;


	
};

