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
	cv::Mat	imgColor;
	UINT uColorBufferSize = 0;
	UINT uColorPointNum = 0;
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
		int iWidth = 0,
			iHeight = 0;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidth);
			pFrameDescription->get_Height(&iHeight);
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

		imgColor = cv::Mat(iHeight, iWidth,CV_8UC4);
		uColorPointNum = iWidth * iHeight;
		uColorBufferSize = uColorPointNum * 4 * sizeof(unsigned char);
	}

	IDepthFrameReader* pDepthFrameReader = nullptr;
	cv::Mat imgDepth;
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
		int iWidth = 0,
			iHeight = 0;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iWidth);
			pFrameDescription->get_Height(&iHeight);
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

		imgDepth = cv::Mat(iHeight, iWidth, CV_16UC1);
		uDepthPointNum = iWidth * iHeight;
	}

	ICoordinateMapper* pCoordinateMapper = nullptr;
	if( pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK )
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}

	UINT32 uTableSize = 0;
	PointF* aTable = nullptr;
	auto res = pCoordinateMapper->GetDepthFrameToCameraSpaceTable(&uTableSize, &aTable);
	// Enter main loop
	DepthSpacePoint* pPointArray = new DepthSpacePoint[uColorPointNum];
	while (true)
	{
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			pColorFrame->CopyConvertedFrameDataToArray(uColorBufferSize, imgColor.data, ColorImageFormat_Bgra);
			pColorFrame->Release();
		}

		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			pDepthFrame->CopyFrameDataToArray(uDepthPointNum, reinterpret_cast<UINT16*>(imgDepth.data));
			pDepthFrame->Release();
		}

		if (pCoordinateMapper->MapColorFrameToDepthSpace(uDepthPointNum, reinterpret_cast<UINT16*>(imgDepth.data), uColorPointNum, pPointArray) == S_OK)
		{
			for (size_t y = 0; y < imgColor.rows; ++y)
				for (size_t x = 0; x < imgColor.cols; ++x)
				{
					const DepthSpacePoint& rPoint = pPointArray[y * imgColor.cols + x];
					if (rPoint.X >= 0 && rPoint.X < imgDepth.cols && rPoint.Y >= 0 && rPoint.Y < imgDepth.rows)
					{
						imgColor.at<cv::Vec4b>(y, x) = 255 * imgDepth.at<UINT16>((int)rPoint.Y, (int)rPoint.X) / 8000;
					}
				}

			cv::imshow("Image", imgColor);
		}

		// check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}
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
