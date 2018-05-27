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
	//�I������
	if(kinect != 0){
		kinect->NuiShutdown();
		kinect->Release();
	}
}


void KinectControl::initialize()
{
	createInstance();

	//Kinect�̏����ݒ�
	ERROR_CHECK(kinect->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON));

	//RGB������
	ERROR_CHECK(kinect->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR,CAMERA_RESOLUTION,0,2,0,&imageStreamHandle));

	//Depth������
	ERROR_CHECK(kinect->NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, CAMERA_RESOLUTION, 0, 2, 0, &depthStreamHandle));

	//Near���[�h���g�p����ɂ́ANear���[�h�̃t���O�������ɐݒ肷��
	ERROR_CHECK(kinect->NuiImageStreamSetImageFrameFlags(depthStreamHandle, NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE));

	//skeleton������
	//ERROR_CHECK(kinect->NuiSkeletonTrackingEnable(0,NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT));
	ERROR_CHECK(kinect->NuiSkeletonTrackingEnable(0,NUI_SKELETON_TRACKING_FLAG_ENABLE_IN_NEAR_RANGE));// | NUI_SKELETON_TRACKING_FLAG_ENABLE_SEATED_SUPPORT));
		

	//�t���[���X�V�C�x���g�̍쐬
	streamEvent = ::CreateEventA(0,TRUE,FALSE,0);
	ERROR_CHECK(kinect->NuiSetFrameEndEvent(streamEvent,0));

	::NuiImageResolutionToSize(CAMERA_RESOLUTION,width,height);
}

void KinectControl::run()
{
	//���C�����[�v
	while(1){
		//�X�V�҂�
		DWORD ret = ::WaitForSingleObject(streamEvent,INFINITE);
		::ResetEvent(streamEvent);



		//�L�[�E�F�C�g
		int key = cv::waitKey(10);
		if(key == 'q') {
			break;
		}
		switch(key) {
		case 'x':
			//�����f�[�^��ۑ�����O�ɁA�ꖇ�f�[�^��ۑ����Ă݂܂��傤
			//���܂��ł�����Acase '1', '2', ...�@�ɐi��ł�������
			save1Skeleton();
			break;
		case '1':
			std::cout << "�|�[�Y1�̎����f�[�^�̎擾���J�n����" << std::endl;
			//�t���O��ݒ肷��A�ۑ����ꂽ�t���[����(savedFrameIdx)������������
			//...
			processFlag = PROCESS_SAVEPOSE1;
			setUpSavePose();

			break;
		case '2':
			std::cout << "�|�[�Y2�̎����f�[�^�̎擾���J�n����" << std::endl;
			//�t���O��ݒ肷��A�ۑ����ꂽ�t���[����������������
			//...
			processFlag = PROCESS_SAVEPOSE2;
			setUpSavePose();

			break;
		case '3':
			std::cout << "�|�[�Y3�̎����f�[�^�̎擾���J�n����" << std::endl;
			//�t���O��ݒ肷��A�ۑ����ꂽ�t���[����������������
			//...
			processFlag = PROCESS_SAVEPOSE3;
			setUpSavePose();

			break;
		case '4':
			std::cout << "�|�[�Y4�̎����f�[�^�̎擾���J�n����" << std::endl;
			//�t���O��ݒ肷��A�ۑ����ꂽ�t���[����������������
			//...
			processFlag = PROCESS_SAVEPOSE4;
			setUpSavePose();

			break;
		case 'a':
			std::cout << "�F�����J�n����" << std::endl;
			//�ۑ����ꂽ�����f�[�^��ǂݍ���
			//�t���O��ݒ肷��
			//...

			processFlag = PROCESS_RECOGNITION;

			break;
		case 'b':
			std::cout << "�F�����~����" << std::endl;
			processFlag = PROCESS_NONE;
			break;
		}

		//�J���[�摜�擾�ƕ\��
		setRgbImage();
		setDepthImage();
		setSkeletonImage();

		cv::imshow("RGB Image", rgbIm);
		cv::imshow("Depth", depthIm);
		cv::imshow("Player", playerIm);
		cv::imshow("Skeleton", skeletonIm);


		//�|�[�Y1,2,3,4�����ꂼ��ۑ�
		if (processFlag != PROCESS_RECOGNITION && processFlag != PROCESS_NONE && savedFrameIdx < MAXFRAME){
				cv::waitKey(500);
				poseRecognize();
				std::cout <<"�t���[��" + std::to_string(savedFrameIdx + 1) + "�擾OK..." << std::endl;
				savedFrameIdx++;
		}

		if (savedFrameIdx == MAXFRAME && processFlag != PROCESS_RECOGNITION &&  processFlag != PROCESS_NONE){
			if (processFlag == PROCESS_SAVEPOSE1) {
				cv::FileStorage cfs("pose1.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\n�Ȃ�Ƃ��������������|�[�Y�Ȃ�...�I�I\n" << std::endl;
			}
			else if (processFlag == PROCESS_SAVEPOSE2) {
				cv::FileStorage cfs("pose2.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\n����Ȃɂ����������ƃX�J�E�g���ꂿ�Ⴂ�������˂��I...�I�I\n" << std::endl;
			}
			else if (processFlag == PROCESS_SAVEPOSE3) {
				cv::FileStorage cfs("pose3.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\n���߂ĉ����������f�G���Ȃ��Ďv���Ă܂���...///\n" << std::endl;
			}
			else if (processFlag == PROCESS_SAVEPOSE4) {
				cv::FileStorage cfs("pose4.xml", cv::FileStorage::WRITE);
				cfs << "root" << poseData;
				cfs.release();
				std::cout << "\n���J���ɒP�ʂ����܂�...�I�I\n" << std::endl;
			}

			processFlag = PROCESS_NONE;
		}


		//�J�������������|�[�Y�ƁA�\�ߕۑ������|�[�Y1,2,3,4���ǂꂭ�炢���Ă邩�ȁ`
		if (processFlag == PROCESS_RECOGNITION){
			
			//if (savedFrameIdx < MAXFRAME){
				//cv::waitKey(500);
				//poseRecognize();
				//std::cout << "�t���[��" + std::to_string(savedFrameIdx + 1) + "�擾OK..." << std::endl;
				//savedFrameIdx++;
			//}
			//else{
				//std::cout << "�S�t���[���擾���܂����I �ގ��x�v�Z�ɓ���܂��˂��I" << std::endl;
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

	/*pose1~4���Ȃ�Ƃ����鏈�����K�v
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
		std::cout << "MAX��ϐg�I�I�I" << std::endl;
	}
	else if (winner == calcResult2){
		std::cout << "�F����������(߁��)�������I�I" << std::endl;
	}
	else if (winner == calcResult3){
		std::cout << "���΂�����񂪌����Ă���..." << std::endl;
	}
	else if (winner == calcResult4){
		std::cout << "���ϐg�I�I�I" << std::endl;
	}

}

void KinectControl::createInstance()
{
	//Kinect�̐����擾
	int count = 0;
	ERROR_CHECK(::NuiGetSensorCount(&count));
	if(count == 0){
		throw std::runtime_error("Kinect��ڑ����Ă�������");
	}

	//�ŏ��̃C���X�^���X�쐬
	ERROR_CHECK(::NuiCreateSensorByIndex(0,&kinect));

	//Kinect�̏�Ԃ��擾
	HRESULT status = kinect->NuiStatus();
	if(status!=S_OK){
		throw std::runtime_error("Kinect�����p�\�ł͂���܂���");
	}
}

void KinectControl::setRgbImage()
{
	// RGB�J�����̃t���[���f�[�^���擾����
	NUI_IMAGE_FRAME imageFrame = {0};
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(
		imageStreamHandle,0,&imageFrame));

	//�摜�擾
	NUI_LOCKED_RECT colorData;
	imageFrame.pFrameTexture->LockRect(0,&colorData,0,0);

	//�摜�R�s�[
	rgbIm = cv::Mat(height,width,CV_8UC4,colorData.pBits);
	cv::cvtColor(rgbIm,rgbIm,1);//CV_BGRA2BGR);

	//�t���[�����
	ERROR_CHECK(kinect->NuiImageStreamReleaseFrame(
		imageStreamHandle,&imageFrame));
}

void KinectControl::setDepthImage()
{
	NUI_IMAGE_FRAME depthFrame = {0};
	ERROR_CHECK(kinect->NuiImageStreamGetNextFrame(
		depthStreamHandle, 0, &depthFrame));
	
	//�����摜�擾
	NUI_LOCKED_RECT depthData = {0};
	depthFrame.pFrameTexture->LockRect(0, &depthData, 0, 0);

	USHORT* depth = (USHORT*)depthData.pBits;
	
	depthIm = cv::Mat(height, width, CV_16UC1, depth);
	double minVal, maxVal;
	cv::minMaxIdx(depthIm, &minVal, &maxVal);
	depthIm.convertTo(depthIm, CV_8UC1, 255.0/maxVal - minVal);	

	setPlayerIndex(depth);

	//�t���[�����
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
        case NUI_SKELETON_TRACKED: //�ڍ׃X�P���g���f�[�^�𓾂���
			drawTrackedSkeleton(skeletonIm, skeleton);
			break;
 
        case NUI_SKELETON_POSITION_ONLY: //�d�S����
            drawPoint(skeletonIm, skeleton.Position);
            break;
        }
		if (skeleton.eTrackingState == NUI_SKELETON_TRACKED)
			break;
	}
}


void KinectControl::drawTrackedSkeleton(cv::Mat& image, const NUI_SKELETON_DATA& skeleton)
{
    // ����
    drawBone(image, skeleton, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
    drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
    drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
	drawBone(image, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
	drawBone(image, skeleton, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);

    // �r�⑫�Ȃǂ̕`��
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
    
    // �ǐՂ��ꂽ�|�C���g�݂̂�`��
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
	// �R�����̈ʒu���狗���摜�ł̈ʒu�ɕϊ�
	FLOAT depthX1 = 0, depthY1 = 0;
	FLOAT depthX2 = 0, depthY2 = 0;

	NuiTransformSkeletonToDepthImage(pos1, &depthX1, &depthY1, CAMERA_RESOLUTION);
	NuiTransformSkeletonToDepthImage(pos2, &depthX2, &depthY2, CAMERA_RESOLUTION);

	// �����摜�ł̈ʒu����RGB�摜�ł̈ʒu�ɕϊ�
	LONG colorX1 = 0, colorY1 = 0;
	LONG colorX2 = 0, colorY2 = 0;
	kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
		CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX1, (LONG)depthY1, 0, &colorX1, &colorY1 );
	kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
		CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0, (LONG)depthX2, (LONG)depthY2, 0, &colorX2, &colorY2 );
	
	// RGB�摜�ł̈ʒu�ɐ�����`��
	cv::line(image, cv::Point(colorX1,colorY1), cv::Point(colorX2,colorY2), cv::Scalar(50,255,50), 5);
}


void KinectControl::drawPoint( cv::Mat& image, Vector4 position )
{
	// �R�����̈ʒu���狗���摜�ł̈ʒu�ɕϊ�
	FLOAT depthX = 0, depthY = 0;
	NuiTransformSkeletonToDepthImage(position, &depthX, &depthY, CAMERA_RESOLUTION);

	// �����摜�ł̈ʒu����RGB�摜�ł̈ʒu�ɕϊ�
	LONG colorX = 0;
	LONG colorY = 0;
	kinect->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
		CAMERA_RESOLUTION, CAMERA_RESOLUTION, 0,
		(LONG)depthX, (LONG)depthY, 0, &colorX, &colorY );
	
	// RGB�摜�ł̈ʒu�Ɋۂ�`��
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
	//�ۑ�
	cv::FileStorage cfs("skeleton.xml",cv::FileStorage::WRITE);
	cfs << "root" << skeleton1;
	cfs.release();

}