// Created by Heresy @ 2015/02/25
// Blog Page: 
// This sample is used to read information of body joint nad draw with OpenCV.

// Standard Library
#include <iostream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

void DrawLine(cv::Mat& rImg, const Joint& rJ1, const Joint& rJ2, ICoordinateMapper* pCMapper )
{
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	ColorSpacePoint ptJ1, ptJ2;
	pCMapper->MapCameraPointToColorSpace(rJ1.Position, &ptJ1);
	pCMapper->MapCameraPointToColorSpace(rJ2.Position, &ptJ2);

	cv::line(rImg, cv::Point(ptJ1.X, ptJ1.Y), cv::Point(ptJ2.X, ptJ2.Y), cv::Vec3b(0, 0, 255), 5);
}

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

	// 2. Color Related code
	IColorFrameReader* pColorFrameReader = nullptr;
	cv::Mat	mColorImg;
	UINT uBufferSize = 0;
	{
		// 2a. Get color frame source
		cout << "Try to get color source" << endl;
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// 2b. Get frame description
		cout << "get color frame description" << endl;
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

		// 2c. get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// 2d. release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;

		// Prepare OpenCV data
		mColorImg = cv::Mat(iHeight, iWidth, CV_8UC4);
		uBufferSize = iHeight * iWidth * 4 * sizeof(BYTE);
	}

	// 3. Body related code
	IBodyFrameReader* pBodyFrameReader = nullptr;
	IBody** aBodyData = nullptr;
	INT32 iBodyCount = 0;
	{
		// 3a. Get frame source
		cout << "Try to get body source" << endl;
		IBodyFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get body frame source" << endl;
			return -1;
		}

		// 3b. Get the number of body
		if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK)
		{
			cerr << "Can't get body count" << endl;
			return -1;
		}
		cout << " > Can trace " << iBodyCount << " bodies" << endl;
		aBodyData = new IBody*[iBodyCount];
		for (int i = 0; i < iBodyCount; ++i)
			aBodyData[i] = nullptr;

		// 3c. get frame reader
		cout << "Try to get body frame reader" << endl;
		if (pFrameSource->OpenReader(&pBodyFrameReader) != S_OK)
		{
			cerr << "Can't get body frame reader" << endl;
			return -1;
		}

		// 3d. release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. get CoordinateMapper
	ICoordinateMapper* pCoordinateMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cout << "Can't get coordinate mapper" << endl;
		return -1;
	}

	// Enter main loop
	cv::namedWindow("Body Image");
	while (true)
	{
		// 4a. Get last frame
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			// 4c. Copy to OpenCV image
			if (pColorFrame->CopyConvertedFrameDataToArray(uBufferSize, mColorImg.data, ColorImageFormat_Bgra) != S_OK)
			{
				cerr << "Data copy error" << endl;
			}

			// 4e. release frame
			pColorFrame->Release();
		}
		cv::Mat mImg = mColorImg.clone();

		// 4b. Get body data
		IBodyFrame* pBodyFrame = nullptr;
		if (pBodyFrameReader->AcquireLatestFrame(&pBodyFrame) == S_OK)
		{
			// 4b. get Body data
			if (pBodyFrame->GetAndRefreshBodyData(iBodyCount, aBodyData) == S_OK)
			{
				// 4c. for each body
				for (int i = 0; i < iBodyCount; ++i)
				{
					IBody* pBody = aBodyData[i];

					// check if is tracked
					BOOLEAN bTracked = false;
					if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
					{
						// get joint position
						Joint aJoints[JointType::JointType_Count];
						if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK)
						{
							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_Neck], aJoints[JointType_Head], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_WristRight], aJoints[JointType_HandRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], pCoordinateMapper);

							DrawLine(mImg, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], pCoordinateMapper);
							DrawLine(mImg, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], pCoordinateMapper);
						}
					}
				}
			}
			else
			{
				cerr << "Can't read body data" << endl;
			}

			// 4e. release frame
			pBodyFrame->Release();
		}

		// show image
		cv::imshow("Body Image",mImg);

		// 4c. check keyboard input
		if (cv::waitKey(30) == VK_ESCAPE){
			break;
		}
	}

	// 3. delete body data array
	delete[] aBodyData;

	// 3. release frame reader
	cout << "Release body frame reader" << endl;
	pBodyFrameReader->Release();
	pBodyFrameReader = nullptr;

	// 2. release color frame reader
	cout << "Release color frame reader" << endl;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;

	// 1c. Close Sensor
	cout << "close sensor" << endl;
	pSensor->Close();

	// 1d. Release Sensor
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;

	return 0;
}
