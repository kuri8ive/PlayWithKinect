#include <iostream>
#include <opencv2/opencv.hpp>
#include "KinectControl.h"

int main(int argc, char *argv[])
{
	CKinectControl kinect;
	kinect.initializeSensor();
	kinect.initializeFaceTracker();
	kinect.run();
	
	return 0;
}