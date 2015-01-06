// Standard Library
#include <iostream>

// Kinect for Windows SDK Header
#include <Kinect.h>

// Link Kinect for Windows SDK Library
#pragma comment( lib, "kinect20.lib")

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
			cout << "Try to get depth source" << endl;
			IDepthFrameSource* pFrameSource = nullptr;
			if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
			{
				cerr << "Can't get depth frame source" << endl;
			}
			else
			{
				// 3a. get frame reader
				IDepthFrameReader* pFrameReader = nullptr;
				if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
				{
					cerr << "Can't get depth frame reader" << endl;
				}
				else
				{
					// Enter main loop
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
							pFrame->get_FrameDescription(&pFrameDescription);
							pFrameDescription->get_Width(&iWidth);
							pFrameDescription->get_Height(&iHeight);

							// 4c. Get image buffer
							UINT	iBufferSize = 0;
							UINT16*	pBuffer = nullptr;
							pFrame->AccessUnderlyingBuffer(&iBufferSize, &pBuffer);

							// 4d. Output depth value
							int x = iWidth / 2,
								y = iHeight / 2;
							size_t idx = x + iWidth * y;
							std::cout << pBuffer[idx] << std::endl;

							// 4e. release frame
							pFrame->Release();
							++uFrameCount;
						}
					}

					// 3b. release frame reader
					pFrameReader->Release();
					pFrameReader = nullptr;
				}

				// 2b. release Frame source
				pFrameSource->Release();
				pFrameSource = nullptr;
			}

			// 1c. Close Sensor
			pSensor->Close();
		}

		// 1d. Release Sensor
		pSensor->Release();
		pSensor = nullptr;
	}
	return 0;
}
