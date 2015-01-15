// Created by Heresy @ 2015/01/13
// Blog Page: 
// This sample is used to read color and depth image from Kinect v2 sensor, and show the image with OpenCV.

// Standard Library
#include <array>
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

int main(int argc, char** argv)
{
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	IColorFrameReader* pColorFrameReader = nullptr;
	UINT uColorBufferSize = 0;
	array<int, 2> aColorSize;
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&aColorSize[0]);
			pFrameDescription->get_Height(&aColorSize[1]);
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;

		uColorBufferSize = aColorSize[0] * aColorSize[1] * 4 * sizeof(unsigned char);
	}

	IDepthFrameReader* pDepthFrameReader = nullptr;
	array<int, 2> aDepthSize;
	UINT uDepthPointNum = 0;
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&aDepthSize[0]);
			pFrameDescription->get_Height(&aDepthSize[1]);
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;

		uDepthPointNum = aDepthSize[0] * aDepthSize[1];
	}

	ICoordinateMapper* pCoordinateMapper;
	if( pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK )
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}

	// Enter main loop
	UINT				uColorPointNum = aColorSize[0] * aColorSize[1];
	unsigned char*		pColorImage = new unsigned char[uColorPointNum * 4];
	UINT16*				pDrpthImage = new UINT16[aDepthSize[0] * aDepthSize[1]];
	DepthSpacePoint*	pPointArray = new DepthSpacePoint[aColorSize[0] * aColorSize[1]];

	while (true)
	{
		cv::Mat imgColor;

		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			pColorFrame->CopyConvertedFrameDataToArray(uColorBufferSize, pColorImage, ColorImageFormat_Bgra);
			pColorFrame->Release();

			imgColor = cv::Mat(aColorSize[1], aColorSize[0], CV_8UC4, pColorImage);

			IDepthFrame* pDepthFrame = nullptr;
			if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
			{
				pDepthFrame->CopyFrameDataToArray(uDepthPointNum, pDrpthImage);
				pDepthFrame->Release();
			}

			if (pCoordinateMapper->MapColorFrameToDepthSpace(uDepthPointNum, pDrpthImage, uColorPointNum, pPointArray) == S_OK)
			{
				for (size_t y = 0; y < aColorSize[1]; ++y)
					for (size_t x = 0; x < aColorSize[0]; ++x)
					{
						const DepthSpacePoint& rPoint = pPointArray[y * aColorSize[0] + x];
						if (rPoint.X >= 0 && rPoint.X < aDepthSize[0] && rPoint.Y >= 0 && rPoint.Y < aDepthSize[1])
						{
							size_t idx = (int)rPoint.Y *aDepthSize[0] + (int)rPoint.X;
							INT16 vDepth = pDrpthImage[idx];

							imgColor.at<cv::Vec4b>(y, x) = 255 * vDepth / 8000;
						}
					}

				cv::imshow("Image", imgColor);
			}
		}

		// check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}
	delete pColorImage;
	delete pDrpthImage;
	delete pPointArray;

	// 3b. release frame reader
	cout << "Release frame reader" << endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
