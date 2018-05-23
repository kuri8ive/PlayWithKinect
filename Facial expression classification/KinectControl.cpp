#include "KinectControl.h"


CKinectControl::CKinectControl(void):widthDepth(0),heightDepth(0),widthColor(0),heightColor(0),pFTResult(NULL), kinect(NULL), isFaceTracked(false)
{
}


CKinectControl::~CKinectControl(void)
{
	if (this->kinect) {
		this->kinect->NuiShutdown();
		this->kinect->Release();
	}
	if (this->pColorFrame) {
		this->pColorFrame->Release();
		this->pColorFrame = NULL;
	}
	if (this->pDepthFrame) {
		this->pDepthFrame->Release();
		this->pDepthFrame = NULL;
	}

}

int CKinectControl::initializeSensor(void)
{
	int numSensor = 0;
	ERROR_CHECK(NuiGetSensorCount(&numSensor));
	if (numSensor == 0) {
		throw std::runtime_error("Kinectを接続してください");
	}
	ERROR_CHECK(NuiCreateSensorByIndex(0,&this->kinect));
	ERROR_CHECK(this->kinect->NuiStatus());

	ERROR_CHECK(this->kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | 
		NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX));

	//カラー
	ERROR_CHECK(kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR,
		NUI_IMAGE_RESOLUTION_640x480, 0, 2, 0, &this->rgbHandle));
	NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_640x480,this->widthColor,this->heightColor);

	//距離
	ERROR_CHECK(this->kinect->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX,
		NUI_IMAGE_RESOLUTION_320x240, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE, 2, NULL, &this->depthHandle));
	NuiImageResolutionToSize(NUI_IMAGE_RESOLUTION_320x240,this->widthDepth,this->heightDepth);
 
	this->streamEvent = ::CreateEventA(0,TRUE,FALSE,0);
	ERROR_CHECK(this->kinect->NuiSetFrameEndEvent(this->streamEvent,0));
	
	return numSensor;
}

void CKinectControl::run(void)
{
	DWORD ret;
	int key;
	while(1) {
		ret = WaitForSingleObject(this->streamEvent,INFINITE);
		ResetEvent(this->streamEvent);

		this->processRGBImage();
		this->processDepthImage();
		this->processFaceTracker();

		cv::imshow("rgbIm", this->rgbIm);

		key = cv::waitKey(10);
		if (key == 'q') {
			break;
		} 

		
	}
}

int CKinectControl::initializeFaceTracker(void)
{
	if (this->kinect == NULL) {
		throw std::runtime_error("initialize Kinect first");
		return -1;
	}

	this->pFaceTracker = FTCreateFaceTracker();
	if (!this->pFaceTracker) {
		throw std::runtime_error("CKinectControl::initializeFaceTracker: FTCreateFaceTracker return NULL");
	}

	this->videoCameraConfig.Width = this->widthColor;
	this->videoCameraConfig.Height = this->heightColor;
	this->videoCameraConfig.FocalLength = NUI_CAMERA_COLOR_NOMINAL_FOCAL_LENGTH_IN_PIXELS * (this->widthColor / 640.0);

	this->depthCameraConfig.Width = this->widthDepth;
	this->depthCameraConfig.Height = this->heightDepth;
	this->depthCameraConfig.FocalLength = NUI_CAMERA_DEPTH_NOMINAL_FOCAL_LENGTH_IN_PIXELS * (this->widthDepth / 320.0);

	ERROR_CHECK(this->pFaceTracker->Initialize(&videoCameraConfig, &depthCameraConfig, NULL, NULL));
	ERROR_CHECK(this->pFaceTracker->CreateFTResult(&this->pFTResult));

	this->pColorFrame = FTCreateImage();
	this->pDepthFrame = FTCreateImage();
	if (!this->pColorFrame || !this->pDepthFrame) {
		throw std::runtime_error("CKinectControl::initializeFaceTracker FTCreateImage failed");
	}
	ERROR_CHECK(this->pColorFrame->Allocate(this->widthColor, this->heightColor, FTIMAGEFORMAT_UINT8_B8G8R8X8));
	ERROR_CHECK(this->pDepthFrame->Allocate(this->widthDepth, this->heightDepth, FTIMAGEFORMAT_UINT16_D13P3));
	
	this->sensorData.pVideoFrame = this->pColorFrame;
	this->sensorData.pDepthFrame = this->pDepthFrame;
	this->sensorData.ZoomFactor = 1.0f; 
	this->sensorData.ViewOffset.x = 0;
	this->sensorData.ViewOffset.y = 0;

	return 0;
}


void CKinectControl::processRGBImage(void)
{
	NUI_IMAGE_FRAME imageFrame = {0};
	ERROR_CHECK(this->kinect->NuiImageStreamGetNextFrame(this->rgbHandle,0,&imageFrame));

	NUI_LOCKED_RECT nuirgb;
	imageFrame.pFrameTexture->LockRect(0,&nuirgb,0,0);
	INuiFrameTexture *pTexture = imageFrame.pFrameTexture;

	if (nuirgb.Pitch) {
		memcpy(this->pColorFrame->GetBuffer(), PBYTE(nuirgb.pBits), (this->pColorFrame->GetBufferSize() < UINT(pTexture->BufferLen()) ? this->pColorFrame->GetBufferSize() : UINT(pTexture->BufferLen())));
	}
	this->rgbIm = cv::Mat(this->heightColor,this->widthColor,CV_8UC4, nuirgb.pBits);

	ERROR_CHECK(this->kinect->NuiImageStreamReleaseFrame(this->rgbHandle,&imageFrame));
}


void CKinectControl::processDepthImage(void)
{
	NUI_IMAGE_FRAME depthFrame = {0};
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(this->depthHandle,0,&depthFrame));

	NUI_LOCKED_RECT nuidepth;
	depthFrame.pFrameTexture->LockRect(0,&nuidepth,0,0);
	INuiFrameTexture *pTexture = depthFrame.pFrameTexture;
	if (nuidepth.Pitch) {
		memcpy(this->pDepthFrame->GetBuffer(), PBYTE(nuidepth.pBits), (this->pDepthFrame->GetBufferSize() < UINT(pTexture->BufferLen()) ? this->pDepthFrame->GetBufferSize() : UINT(pTexture->BufferLen())));
	}

	depthFrame.pFrameTexture->UnlockRect(0);
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(this->depthHandle,&depthFrame));
}


void CKinectControl::processFaceTracker(void)
{
	HRESULT hr;
	if (!this->isFaceTracked) {
		hr = this->pFaceTracker->StartTracking(&this->sensorData,NULL,NULL,this->pFTResult);
		if (SUCCEEDED(hr) && SUCCEEDED(this->pFTResult->GetStatus())) this->isFaceTracked = true;
		else this->isFaceTracked = false;
	}
	else {
		hr = this->pFaceTracker->ContinueTracking(&this->sensorData,NULL, this->pFTResult);
		if (FAILED(hr) || FAILED(this->pFTResult->GetStatus())) this->isFaceTracked = false;
	}

	if (this->isFaceTracked) {
		UINT numPoint;
		FT_VECTOR2D *pointlist;
		this->pFTResult->Get2DShapePoints(&pointlist, &numPoint);
		cv::Point topleft;
		cv::Point bottomright;
		topleft.x = pointlist[0].x;
		topleft.y = pointlist[0].y;
		bottomright.x = pointlist[0].x;
		bottomright.y = pointlist[0].y;
		for (UINT i=0; i < numPoint; i++) {
			this->rgbIm.at<cv::Vec4b>(pointlist[i].y, pointlist[i].x)[0] = 0;
			this->rgbIm.at<cv::Vec4b>(pointlist[i].y, pointlist[i].x)[1] = 255;
			this->rgbIm.at<cv::Vec4b>(pointlist[i].y, pointlist[i].x)[2] = 255;
			this->rgbIm.at<cv::Vec4b>(pointlist[i].y, pointlist[i].x)[3] = 255;

			if (pointlist[i].x < topleft.x) topleft.x = pointlist[i].x;
			if (pointlist[i].y < topleft.y) topleft.y = pointlist[i].y;
			if (pointlist[i].x > bottomright.x) bottomright.x = pointlist[i].x;
			if (pointlist[i].y > bottomright.y) bottomright.y = pointlist[i].y;

		}
		_freea(pointlist);
		cv::rectangle(this->rgbIm, cv::Rect(topleft.x,topleft.y,bottomright.x-topleft.x,bottomright.y-topleft.y),cv::Scalar(0,255,255,255));

		classifyFaceExpression(pointlist,topleft, bottomright);
	}
}

void CKinectControl::classifyFaceExpression(FT_VECTOR2D *pointlist, cv::Point topleft, cv::Point bottomright){
	

	if ((pointlist[LIPRIGHT].x - pointlist[LIPLEFT].x) > (bottomright.x - topleft.x) / SMILERATIO){
		std::cout << "その笑顔で何人落としてきたのよ...///" << std::endl;
	}
	else if (pointlist[LIPBOTTOM].y - pointlist[LIPTOP].y > (bottomright.y - topleft.y) / SURPRISERATIO){
		std::cout << "一体何があった!?応答してくれ!!!" << std::endl;
	}
	else{
		std::cout << "何このイケメン...!!!" << std::endl;
	}
	

}