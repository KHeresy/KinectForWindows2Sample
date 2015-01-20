// Created by Heresy @ 2015/01/20
// Blog Page: http://kheresy.wordpress.com/2015/01/20/k4w-v2-body-index/
// This sample is used to read body index image from Kinect v2 sensor, and show the image with OpenCV.

// Standard Library
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

int main(int argc, char** argv)
{
	// 1a. Get default Sensor
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// 1b. Open sensor
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	// 2a. Get frame source
	cout << "Try to get body index source" << endl;
	IBodyIndexFrameSource* pFrameSource = nullptr;
	if (pSensor->get_BodyIndexFrameSource(&pFrameSource) != S_OK)
	{
		cerr << "Can't get body index frame source" << endl;
		return -1;
	}

	// 2b. Get frame description
	cout << "get body index frame description" << endl;
	int		iWidth = 0;
	int		iHeight = 0;
	IFrameDescription* pFrameDescription = nullptr;
	if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
	{
		pFrameDescription->get_Width(&iWidth);
		pFrameDescription->get_Height(&iHeight);
	}
	pFrameDescription->Release();
	pFrameDescription = nullptr;

	// 3a. get frame reader
	cout << "Try to get body index frame reader" << endl;
	IBodyIndexFrameReader* pFrameReader = nullptr;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get body index frame reader" << endl;
		return -1;
	}

	// 2c. release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;

	// Prepare OpenCV data
	cv::Mat	mImg(iHeight, iWidth, CV_8UC3);
	cv::namedWindow("Body Index Image");

	// color array
	cv::Vec3b aColorTable[7] = {
		cv::Vec3b(255,0,0),
		cv::Vec3b(0,255,0),
		cv::Vec3b(0,0,255),
		cv::Vec3b(255,255,0),
		cv::Vec3b(255,0,255),
		cv::Vec3b(0,255,255),
		cv::Vec3b(0,0,0),
	};

	// Enter main loop
	while (true)
	{
		// 4a. Get last frame
		IBodyIndexFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			// 4c. Fill OpenCV image
			UINT uSize = 0;
			BYTE* pBuffer = nullptr;
			if (pFrame->AccessUnderlyingBuffer(&uSize,&pBuffer) == S_OK)
			{
				for (int y = 0; y < iHeight; ++y)
				{
					for (int x = 0; x < iWidth; ++x)
					{
						int uBodyIdx = pBuffer[x + y * iWidth];
						if (uBodyIdx < 6)
							mImg.at<cv::Vec3b>(y, x) = aColorTable[uBodyIdx];
						else
							mImg.at<cv::Vec3b>(y, x) = aColorTable[6];
					}
				}
				cv::imshow("Body Index Image", mImg);
			}
			else
			{
				cerr << "Data access error" << endl;
			}

			// 4e. release frame
			pFrame->Release();
		}

		// 4f. check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

	// 3b. release frame reader
	cout << "Release frame reader" << endl;
	pFrameReader->Release();
	pFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
