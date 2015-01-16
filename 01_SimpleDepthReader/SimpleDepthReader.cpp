// Created by Heresy @ 2015/01/08
// Blog Page: http://kheresy.wordpress.com/2015/01/08/k4w-v2-simple-depth-reader-cpp/
// This sample is used to read depth map from Kinect v2 sensor, and output 1 pixel value in console.

// Standard Library
#include <iostream>

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
	}
	else
	{
		// 1b. Open sensor
		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
		}
		else
		{
			// 2a. Get frame source
			cout << "Try to get source" << endl;
			IDepthFrameSource* pFrameSource = nullptr;
			if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
			{
				cerr << "Can't get frame source" << endl;
			}
			else
			{
				// 3a. get frame reader
				cout << "Try to get frame reader" << endl;
				IDepthFrameReader* pFrameReader = nullptr;
				if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
				{
					cerr << "Can't get frame reader" << endl;
				}
				else
				{
					// Enter main loop
					cout << "Enter main loop" << endl;
					size_t uFrameCount = 0;
					while (uFrameCount < 100)
					{
						// 4a. Get last frame
						IDepthFrame* pFrame = nullptr;
						if( pFrameReader->AcquireLatestFrame(&pFrame) == S_OK )
						{
							// 4b. Get frame description
							int		iWidth = 0;
							int		iHeight = 0;
							IFrameDescription* pFrameDescription = nullptr;
							if (pFrame->get_FrameDescription(&pFrameDescription) == S_OK)
							{
								pFrameDescription->get_Width(&iWidth);
								pFrameDescription->get_Height(&iHeight);
								pFrameDescription->Release();
								pFrameDescription = nullptr;
							}

							// 4c. Get image buffer
							UINT	uBufferSize = 0;
							UINT16*	pBuffer = nullptr;
							if (pFrame->AccessUnderlyingBuffer(&uBufferSize, &pBuffer) == S_OK)
							{
								// 4d. Output depth value
								int x = iWidth / 2,
									y = iHeight / 2;
								size_t idx = x + iWidth * y;
								std::cout << pBuffer[idx] << std::endl;
							}
							else
							{
								cerr << "Data access error" << endl;
							}

							// 4e. release frame
							pFrame->Release();
							pFrame = nullptr;

							++uFrameCount;
						}
					}

					// 3b. release frame reader
					cout << "Release frame reader" << endl;
					pFrameReader->Release();
					pFrameReader = nullptr;
				}

				// 2b. release Frame source
				cout << "Release frame source" << endl;
				pFrameSource->Release();
				pFrameSource = nullptr;
			}

			// 1c. Close Sensor
			cout << "close sensor" << endl;
			pSensor->Close();
		}

		// 1d. Release Sensor
		cout << "Release sensor" << endl;
		pSensor->Release();
		pSensor = nullptr;
	}
	return 0;
}
