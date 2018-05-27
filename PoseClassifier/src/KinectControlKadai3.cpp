#include "KinectControlKadai3.h"

int skeletonList[20] = { NUI_SKELETON_POSITION_HIP_CENTER,
NUI_SKELETON_POSITION_SPINE,
NUI_SKELETON_POSITION_SHOULDER_CENTER,
NUI_SKELETON_POSITION_HEAD,
NUI_SKELETON_POSITION_SHOULDER_LEFT,
NUI_SKELETON_POSITION_ELBOW_LEFT,
NUI_SKELETON_POSITION_WRIST_LEFT,
NUI_SKELETON_POSITION_HAND_LEFT,
NUI_SKELETON_POSITION_SHOULDER_RIGHT,
NUI_SKELETON_POSITION_ELBOW_RIGHT,
NUI_SKELETON_POSITION_WRIST_RIGHT,
NUI_SKELETON_POSITION_HAND_RIGHT,
NUI_SKELETON_POSITION_HIP_LEFT,
NUI_SKELETON_POSITION_KNEE_LEFT,
NUI_SKELETON_POSITION_ANKLE_LEFT,
NUI_SKELETON_POSITION_FOOT_LEFT,
NUI_SKELETON_POSITION_HIP_RIGHT,
NUI_SKELETON_POSITION_KNEE_RIGHT,
NUI_SKELETON_POSITION_ANKLE_RIGHT,
NUI_SKELETON_POSITION_FOOT_RIGHT };

KinectControl::KinectControl()
{
	skeleton.eTrackingState = NUI_SKELETON_NOT_TRACKED;
	processFlag = PROCESS_NONE;
}

KinectControl::~KinectControl()
{
	//終了処理
	if(kinect != 0){
		kinect->NuiShutdown();
		kinect->Release();
	}
}


void KinectControl::initialize()
{
	createInstance();

	//Kinectの初期設定
	ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON));

	//RGB初期化
	ERROR_CHECK(kinect->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,CAMERA_RESOLUTION,0,2,0,&imageStreamHandle));

	//Depth初期化
	ERROR_CHECK(kinect->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle));

	//Nearモードを使用するには、Nearモードのフラグをここに設定する
	ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE));

	//skeleton初期化
	//ERROR_CHECK(kinect->NuiSkeletonTrackingEnable(0,NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT));
	ERROR_CHECK(kinect->NuiSkeletonTrackingEnable(0,NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE));// | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT));
		

	//フレーム更新イベントの作成
	streamEvent = ::CreateEventA(0,TRUE,FALSE,0);
	ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent,0));

	::NuiImageResolutionToSize(CAMERA_RESOLUTION,width,height);
}

void KinectControl::run()
{
	//メインループ
	while(1){
		//更新待ち
		DWORD ret = ::WaitForSingleObject(streamEvent,INFINITE);
		::ResetEvent(streamEvent);



		//キーウェイト
		int key = cv::waitKey(10);
		if(key == 'q') {
			break;
		}
		switch(key) {
		case 'x':
			//複数データを保存する前に、一枚データを保存してみましょう
			//うまくできたら、case '1', '2', ...　に進んでください
			save1Skeleton();
			break;
		case '1':
			std::cout << "ポーズ1の辞書データの取得を開始する" << std::endl;
			//フラグを設定する、保存されたフレーム数(savedFrameIdx)を初期化する
			//...
			processFlag = PROCESS_SAVEPOSE1;
			setUpSavePose();

			break;
		case '2':
			std::cout << "ポーズ2の辞書データの取得を開始する" << std::endl;
			//フラグを設定する、保存されたフレーム数を初期化する
			//...
			processFlag = PROCESS_SAVEPOSE2;
			setUpSavePose();

			break;
		case '3':
			std::cout << "ポーズ3の辞書データの取得を開始する" << std::endl;
			//フラグを設定する、保存されたフレーム数を初期化する
			//...
			processFlag = PROCESS_SAVEPOSE3;
			setUpSavePose();

			break;
		case '4':
			std::cout << "ポーズ4の辞書データの取得を開始する" << std::endl;
			//フラグを設定する、保存されたフレーム数を初期化する
			//...
			processFlag = PROCESS_SAVEPOSE4;
			setUpSavePose();

			break;
		case 'a':
			std::cout << "認識を開始する" << std::endl;
			//保存された辞書データを読み込む
			//フラグを設定する
			//...

			processFlag = PROCESS_RECOGNITION;

			break;
		case 'b':
			std::cout << "認識を停止する" << std::endl;
			processFlag = PROCESS_NONE;
			break;
		}

		//カラー画像取得と表示
		setRgbImage();
		setDepthImage();
		setSkeletonImage();

		cv::imshow("RGB Image", rgbIm);
		cv::imshow("Depth", depthIm);
		cv::imshow("Player", playerIm);
		cv::imshow("Skeleton", skeletonIm);


		//ポーズ1,2,3,4をそれぞれ保存
		if (processFlag != PROCESS_RECOGNITION && processFlag != PROCESS_NONE && savedFrameIdx < MAXFRAME){
				cv::waitKey(500);
				poseRecognize();
				std::cout <<"フレーム" + std::to_string(savedFrameIdx + 1) + "取得OK..." << std::endl;
				savedFrameIdx++;
		}

		if (savedFrameIdx == MAXFRAME && processFlag != PROCESS_RECOGNITION &&  processFlag != PROCESS_NONE){
			if (processFlag == PROCESS_SAVEPOSE1) {
				cv::FileStorage cfs("pose1.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\nなんというかっこいいポーズなんだ...！！\n" << std::endl;
			}
			else if (processFlag == PROCESS_SAVEPOSE2) {
				cv::FileStorage cfs("pose2.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\nそんなにかっこいいとスカウトされちゃいそうだねっ！...！！\n" << std::endl;
			}
			else if (processFlag == PROCESS_SAVEPOSE3) {
				cv::FileStorage cfs("pose3.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\n初めて会った時から素敵だなって思ってました...///\n" << std::endl;
			}
			else if (processFlag == PROCESS_SAVEPOSE4) {
				cv::FileStorage cfs("pose4.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\nご褒美に単位あげます...！！\n" << std::endl;
			}

			processFlag = PROCESS_NONE;
		}


		//カメラが捉えたポーズと、予め保存したポーズ1,2,3,4がどれくらい似てるかな～
		if (processFlag == PROCESS_RECOGNITION){
			
			//if (savedFrameIdx < MAXFRAME){
				//cv::waitKey(500);
				//poseRecognize();
				//std::cout << "フレーム" + std::to_string(savedFrameIdx + 1) + "取得OK..." << std::endl;
				//savedFrameIdx++;
			//}
			//else{
				//std::cout << "全フレーム取得しました！ 類似度計算に入りますねっ！" << std::endl;
				//testData = poseData;
				cv::waitKey(500);
				classifyPose();
				//processFlag = PROCESS_NONE;
			//}

		}


		
	}
}

void KinectControl::setUpSavePose(){
	savedFrameIdx = 0;
	poseData = cv::Mat::zeros(3 * 20, 20, CV_32F);
	cv::waitKey(3000);
}

void KinectControl::poseRecognize(){
	
	for (int i = 0; i<20; i++)
	{
		poseData.at<FLOAT>(i * 3 + 0 ,savedFrameIdx) = skeleton.SkeletonPositions[skeletonList[i]].x;
		poseData.at<FLOAT>(i * 3 + 1, savedFrameIdx) = skeleton.SkeletonPositions[skeletonList[i]].y;
		poseData.at<FLOAT>(i * 3 + 2, savedFrameIdx) = skeleton.SkeletonPositions[skeletonList[i]].z;
	}
	
}

void KinectControl::classifyPose(){
	
	//cv::Mat tmp1 = cv::Mat::zeros(20, 20, CV_32F);
	//cv::Mat tmp2 = cv::Mat::zeros(20, 20, CV_32F);
	//cv::Mat tmp3 = cv::Mat::zeros(20, 20, CV_32F);
	//cv::Mat tmp4 = cv::Mat::zeros(20, 20, CV_32F);

	testData = cv::Mat::zeros(3 * 20, 1, CV_32F);

	for (int i = 0; i < 20; i++)
	{
		testData.at<FLOAT>(i * 3 + 0) = skeleton.SkeletonPositions[skeletonList[i]].x;
		testData.at<FLOAT>(i * 3 + 1) = skeleton.SkeletonPositions[skeletonList[i]].y;
		testData.at<FLOAT>(i * 3 + 2) = skeleton.SkeletonPositions[skeletonList[i]].z;
	}

	double calcResult1 = 0.0, calcResult2 = 0.0, calcResult3 = 0.0, calcResult4 = 0.0;
	cv::Mat pose= cv::Mat::zeros(3 * 20, 1, CV_32F);

	cv::FileStorage cfs1("pose1.xml", cv::FileStorage::READ);
	cfs1["root"] >> pose1;
	for (int j = 0; j < 20; j++){
		for (int i = 0; i<20; i++)
		{
			pose.at<FLOAT>(i * 3 + 0) = pose1.at<FLOAT>(i * 3 + 0, j);
			pose.at<FLOAT>(i * 3 + 1) = pose1.at<FLOAT>(i * 3 + 1, j);
			pose.at<FLOAT>(i * 3 + 2) = pose1.at<FLOAT>(i * 3 + 2, j);
		}
		calcResult1 += testData.dot(pose) / (cv::norm(testData)*cv::norm(pose));
	}
	calcResult1 /= MAXFRAME;
	
	cfs1.release();
	
	
	cv::FileStorage cfs2("pose2.xml", cv::FileStorage::READ);
	cfs2["root"] >> pose2;
	for (int j = 0; j < 20; j++){
		for (int i = 0; i<20; i++)
		{
			pose.at<FLOAT>(i * 3 + 0) = pose2.at<FLOAT>(i * 3 + 0, j);
			pose.at<FLOAT>(i * 3 + 1) = pose2.at<FLOAT>(i * 3 + 1, j);
			pose.at<FLOAT>(i * 3 + 2) = pose2.at<FLOAT>(i * 3 + 2, j);
		}
		calcResult2 += testData.dot(pose) / (cv::norm(testData)*cv::norm(pose));
	}
	calcResult2 /= MAXFRAME;
	cfs2.release();


	cv::FileStorage cfs3("pose3.xml", cv::FileStorage::READ);
	cfs3["root"] >> pose3;
	for (int j = 0; j < 20; j++){
		for (int i = 0; i<20; i++)
		{
			pose.at<FLOAT>(i * 3 + 0) = pose3.at<FLOAT>(i * 3 + 0, j);
			pose.at<FLOAT>(i * 3 + 1) = pose3.at<FLOAT>(i * 3 + 1, j);
			pose.at<FLOAT>(i * 3 + 2) = pose3.at<FLOAT>(i * 3 + 2, j);
		}
		calcResult3 += testData.dot(pose) / (cv::norm(testData)*cv::norm(pose));
	}
	calcResult3 /= MAXFRAME;
	cfs3.release();


	cv::FileStorage cfs4("pose4.xml", cv::FileStorage::READ);
	cfs4["root"] >> pose4;
	for (int j = 0; j < 20; j++){
		for (int i = 0; i<20; i++)
		{
			pose.at<FLOAT>(i * 3 + 0) = pose4.at<FLOAT>(i * 3 + 0, j);
			pose.at<FLOAT>(i * 3 + 1) = pose4.at<FLOAT>(i * 3 + 1, j);
			pose.at<FLOAT>(i * 3 + 2) = pose4.at<FLOAT>(i * 3 + 2, j);
		}
		calcResult4 += testData.dot(pose) / (cv::norm(testData)*cv::norm(pose));
	}
	calcResult4 /= MAXFRAME;
	cfs4.release();

	/*pose1~4をなんとかする処理が必要
	for (int i = 0; i<20; i++)
	{
		pose.at<FLOAT>(i * 3 + 0) = pose1.at<FLOAT>(i * 3 + 0, i);
		pose.at<FLOAT>(i * 3 + 1) = pose1.at<FLOAT>(i * 3 + 1, i);
		pose.at<FLOAT>(i * 3 + 2) = pose1.at<FLOAT>(i * 3 + 2, i);
	}

	calcResult1 = testData.dot(pose1) / (cv::norm(testData)*cv::norm(pose1));
	calcResult2 = testData.dot(pose2) / (cv::norm(testData)*cv::norm(pose2));
	calcResult3 = testData.dot(pose3) / (cv::norm(testData)*cv::norm(pose3));
	calcResult4 = testData.dot(pose4) / (cv::norm(testData)*cv::norm(pose4));
	*/

	double winner = cv::max(cv::max(calcResult1, calcResult2), cv::max(calcResult3, calcResult4));
	if (winner == calcResult1){
		std::cout << "MAX大変身！！！" << std::endl;
	}
	else if (winner == calcResult2){
		std::cout << "宇宙ｷﾀ━━━(ﾟ∀ﾟ)━━━！！" << std::endl;
	}
	else if (winner == calcResult3){
		std::cout << "おばあちゃんが言っていた..." << std::endl;
	}
	else if (winner == calcResult4){
		std::cout << "超変身！！！" << std::endl;
	}

}

void KinectControl::createInstance()
{
	//Kinectの数を取得
	int count = 0;
	ERROR_CHECK(::NuiGetSensorCount(&count));
	if(count == 0){
		throw std::runtime_error("Kinectを接続してください");
	}

	//最初のインスタンス作成
	ERROR_CHECK(::NuiCreateSensorByIndex(0,&kinect));

	//Kinectの状態を取得
	HRESULT status = kinect->NuiStatus();
	if(status!=S_OK){
		throw std::runtime_error("Kinectが利用可能ではありません");
	}
}

void KinectControl::setRgbImage()
{
	// RGBカメラのフレームデータを取得する
	NUI_IMAGE_FRAME imageFrame = {0};
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(
		imageStreamHandle,0,&imageFrame));

	//画像取得
	NUI_LOCKED_RECT colorData;
	imageFrame.pFrameTexture->LockRect(0,&colorData,0,0);

	//画像コピー
	rgbIm = cv::Mat(height,width,CV_8UC4,colorData.pBits);
	cv::cvtColor(rgbIm,rgbIm,1);//CV_BGRA2BGR);

	//フレーム解放
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(
		imageStreamHandle,&imageFrame));
}

void KinectControl::setDepthImage()
{
	NUI_IMAGE_FRAME depthFrame = {0};
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(
		depthStreamHandle, 0, &depthFrame));
	
	//距離画像取得
	NUI_LOCKED_RECT depthData = {0};
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	USHORT* depth = (USHORT*)depthData.pBits;
	
	depthIm = cv::Mat(height, width, CV_16UC1, depth);
	double minVal, maxVal;
	cv::minMaxIdx(depthIm, &minVal, &maxVal);
	depthIm.convertTo(depthIm, CV_8UC1, 255.0/maxVal - minVal);	

	setPlayerIndex(depth);

	//フレーム解放
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(
		depthStreamHandle,&depthFrame));
}

void KinectControl::setPlayerIndex(USHORT* depth)
{
	playerIm = cv::Mat::zeros(height, width, CV_8UC1);
	int i = 0;
	int y = 0;
	int x = 0;
	for (y=0; y<height; y++)
	{
		for (x=0; x<width; x++)
		{
			USHORT distance = ::NuiDepthPixelToDepth(depth[i]);
		    USHORT player = ::NuiDepthPixelToPlayerIndex( depth[i] );

			LONG colorX = 0;
			LONG colorY = 0;
			kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, x, y, depth[i], &colorX, &colorY);
			if (player != 0) {
				colorX = (colorX >= width) ? width-1 : (colorX<0 ? 0 : colorX);
				colorY = (colorY >= height) ? height-1 : (colorY<0 ? 0 : colorY);
				playerIm.at<UCHAR>(colorY,colorX) = 255;
			}
			i++;
		}
	}
}

void KinectControl::setSkeletonImage()
{
	NUI_SKELETON_FRAME skeletonFrame = {0};
	kinect->NuiSkeletonGetNextFrame(0, &skeletonFrame);

	skeletonIm = cv::Mat::zeros(height,width,CV_8UC3);
	rgbIm.copyTo(skeletonIm,playerIm);
	
	for (int i=0; i < NUI_SKELETON_COUNT; i++) {
        skeleton = skeletonFrame.SkeletonData[i];
 
        switch (skeleton.eTrackingState)
        {
        case NUI_SKELETON_TRACKED: //詳細スケルトンデータを得られる
			drawTrackedSkeleton(skeletonIm, skeleton);
			break;
 
        case NUI_SKELETON_POSITION_ONLY: //重心だけ
            drawPoint(skeletonIm, skeleton.Position);
            break;
        }
		if (skeleton.eTrackingState == NUI_SKELETON_TRACKED)
			break;
	}
}


void KinectControl::drawTrackedSkeleton(cv::Mat& image, const NUI_SKELETON_DATA& skeleton)
{
    // 胴体
    drawBone(image, skeleton, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
    drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
    drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
	drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
	drawBone(image, skeleton, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);

    // 腕や足などの描画
    drawBone(image,skeleton, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);

    drawBone(image,skeleton, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
    drawBone(image,skeleton, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
}

void KinectControl::drawBone(cv::Mat& image, const NUI_SKELETON_DATA & skeleton, NUI_SKELETON_POSITION_INDEX jointFrom, NUI_SKELETON_POSITION_INDEX jointTo)
{
    NUI_SKELETON_POSITION_TRACKING_STATE jointFromState = skeleton.eSkeletonPositionTrackingState[jointFrom];
    NUI_SKELETON_POSITION_TRACKING_STATE jointToState = skeleton.eSkeletonPositionTrackingState[jointTo];
    
    // 追跡されたポイントのみを描く
    if ((jointFromState == NUI_SKELETON_POSITION_INFERRED || jointToState == NUI_SKELETON_POSITION_INFERRED) ||
		(jointFromState == NUI_SKELETON_POSITION_TRACKED && jointToState == NUI_SKELETON_POSITION_TRACKED))
    {
		const Vector4 jointFromPosition = skeleton.SkeletonPositions[jointFrom];
		const Vector4 jointToPosition = skeleton.SkeletonPositions[jointTo];
		drawLine(image, jointFromPosition, jointToPosition);
    }
}

void KinectControl::drawLine( cv::Mat& image, Vector4 pos1, Vector4 pos2)
{
	// ３次元の位置から距離画像での位置に変換
	FLOAT depthX1 = 0, depthY1 = 0;
	FLOAT depthX2 = 0, depthY2 = 0;

	NuiTransformSkeletonToDepthImage(pos1, &depthX1, &depthY1, CAMERA_RESOLUTION);
	NuiTransformSkeletonToDepthImage(pos2, &depthX2, &depthY2, CAMERA_RESOLUTION);

	// 距離画像での位置からRGB画像での位置に変換
	LONG colorX1 = 0, colorY1 = 0;
	LONG colorX2 = 0, colorY2 = 0;
	kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
		CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX1, (LONG)depthY1, 0, &colorX1, &colorY1 );
	kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
		CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX2, (LONG)depthY2, 0, &colorX2, &colorY2 );
	
	// RGB画像での位置に線分を描画
	cv::line(image, cv::Point(colorX1,colorY1), cv::Point(colorX2,colorY2), cv::Scalar(50,255,50), 5);
}


void KinectControl::drawPoint( cv::Mat& image, Vector4 position )
{
	// ３次元の位置から距離画像での位置に変換
	FLOAT depthX = 0, depthY = 0;
	NuiTransformSkeletonToDepthImage(position, &depthX, &depthY, CAMERA_RESOLUTION);

	// 距離画像での位置からRGB画像での位置に変換
	LONG colorX = 0;
	LONG colorY = 0;
	kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
		CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0,
		(LONG)depthX, (LONG)depthY, 0, &colorX, &colorY );
	
	// RGB画像での位置に丸を描画
	cv::circle( image, cv::Point(colorX,colorY), 10, cv::Scalar( 0, 255, 0), 5);
}

void KinectControl::save1Skeleton()
{
	if (skeleton.eTrackingState != NUI_SKELETON_TRACKED) return;

	cv::Mat skeleton1 = cv::Mat::zeros(3*20,1,CV_32F);

	for (int i=0;i<20;i++)
	{
		skeleton1.at<FLOAT>(i*3+0) = skeleton.SkeletonPositions[skeletonList[i]].x;
		skeleton1.at<FLOAT>(i*3+1) = skeleton.SkeletonPositions[skeletonList[i]].y;
		skeleton1.at<FLOAT>(i*3+2) = skeleton.SkeletonPositions[skeletonList[i]].z;
	}
	//保存
	cv::FileStorage cfs("skeleton.xml",cv::FileStorage::WRITE);
	cfs << "root" << skeleton1;
	cfs.release();

}