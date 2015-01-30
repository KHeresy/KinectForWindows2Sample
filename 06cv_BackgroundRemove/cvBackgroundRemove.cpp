// Created by Heresy @ 2015/01/30
// Blog Page: 
// This sample will use body index frme to filter color frame, show an user only im age with OpenCV.
// ICoordinateMapper is required to map different image coordination.

// Standard Library
#include <array>
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

int main(int argc, char** argv)
{
	// 1. Sensor related code
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

	// 2. Color related code
	IColorFrameReader* pColorFrameReader = nullptr;
	UINT uColorBufferSize = 0;
	UINT uColorPointNum = 0;
	int iColorWidth = 0, iColorHeight = 0;
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
			pFrameDescription->get_Width(&iColorWidth);
			pFrameDescription->get_Height(&iColorHeight);

			uColorPointNum = iColorWidth * iColorHeight;
			uColorBufferSize = uColorPointNum * 4 * sizeof(unsigned char);
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
	}

	// 2. Depth related code
	IDepthFrameReader* pDepthFrameReader = nullptr;
	UINT uDepthPointNum = 0;
	int iDepthWidth = 0, iDepthHeight = 0;
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
			pFrameDescription->get_Width(&iDepthWidth);
			pFrameDescription->get_Height(&iDepthHeight);
			uDepthPointNum = iDepthWidth * iDepthHeight;
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
	}

	// 3. Body Index releated code
	IBodyIndexFrameReader* pBIFrameReader = nullptr;
	cout << "Try to get body index source" << endl;
	{
		// Get frame source
		IBodyIndexFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyIndexFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get body index frame source" << endl;
			return -1;
		}

		// get frame reader
		cout << "Try to get body index frame reader" << endl;
		if (pFrameSource->OpenReader(&pBIFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. Coordinate Mapper
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if( pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK )
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}

	// 5. OpenCV code
	cv::namedWindow("Background Remove");
	cv::Mat	imgColor(iColorHeight, iColorWidth, CV_8UC4);

	// 6. Load background
	cv::Mat imgBG(iColorHeight, iColorWidth, CV_8UC3);
	if (argc > 1)
	{
		cv::Mat imgSrc = cv::imread(argv[1]);
		cv::resize(imgSrc, imgBG, cv::Size(iColorWidth, iColorHeight));
	}
	else
	{
		imgBG.setTo(128);
	}

	// 7. Enter main loop
	UINT16*				pDepthPoints	= new UINT16[uDepthPointNum];
	BYTE*				pBodyIndex		= new BYTE[uDepthPointNum];
	DepthSpacePoint*	pPointArray		= new DepthSpacePoint[uColorPointNum];
	while (true)
	{
		// 7a. Read color frame
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			pColorFrame->CopyConvertedFrameDataToArray(uColorBufferSize, imgColor.data, ColorImageFormat_Bgra);
			pColorFrame->Release();
			pColorFrame = nullptr;
		}

		// 7b. read depth frame
		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			pDepthFrame->CopyFrameDataToArray(uDepthPointNum, pDepthPoints);
			pDepthFrame->Release();
			pDepthFrame = nullptr;
		}

		// 7c. read body index frame
		IBodyIndexFrame* pBIFrame = nullptr;
		if (pBIFrameReader->AcquireLatestFrame(&pBIFrame) == S_OK)
		{
			pBIFrame->CopyFrameDataToArray(uDepthPointNum, pBodyIndex);
			pBIFrame->Release();
			pBIFrame = nullptr;
		}

		// 8a. make a copy of background
		cv::Mat imgTarget = imgBG.clone();

		// 8b. map color to depth
		if (pCoordinateMapper->MapColorFrameToDepthSpace(uDepthPointNum, pDepthPoints, uColorPointNum, pPointArray) == S_OK)
		{
			for (int y = 0; y < imgColor.rows; ++y)
			{
				for (int x = 0; x < imgColor.cols; ++x)
				{
					// ( x, y ) in color frame = rPoint in depth frame
					const DepthSpacePoint& rPoint = pPointArray[y * imgColor.cols + x];

					// check if rPoint is in range
					if (rPoint.X >= 0 && rPoint.X < iDepthWidth && rPoint.Y >= 0 && rPoint.Y < iDepthHeight)
					{
						// fill color from color frame if this pixel is user
						int iIdx = (int)rPoint.X + iDepthWidth * (int)rPoint.Y;
						if (pBodyIndex[iIdx] < 6)
						{
							cv::Vec4b& rPixel = imgColor.at<cv::Vec4b>(y, x);
							imgTarget.at<cv::Vec3b>(y, x) = cv::Vec3b(rPixel[0], rPixel[1], rPixel[2]);
						}
					}
				}
			}

			cv::imshow("Background Remove", imgTarget);
		}

		// check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}
	delete pPointArray;
	delete pBodyIndex;
	delete pDepthPoints;

	// release coordinate mapper
	cout << "Release coordinate mapper" << endl;
	pCoordinateMapper->Release();
	pCoordinateMapper = nullptr;

	// release frame reader
	cout << "Release frame reader" << endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;
	pBIFrameReader->Release();
	pBIFrameReader = nullptr;

	// Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
