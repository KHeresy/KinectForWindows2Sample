// Standard Library
#include <iostream>

// Kinect for Windows SDK Header
#include <Kinect.h>

// Link Kinect for Windows SDK Library
#pragma comment( lib, "kinect20.lib")

using namespace std;

int main(int argc, char** argv)
{
	// Get default Sensor
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
	}
	else
	{
		// Open sensor
		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
		}
		else
		{
			// Get frame source
			cout << "Try to get depth source" << endl;
			IDepthFrameSource* pDepthFrameSource = nullptr;
			if (pSensor->get_DepthFrameSource(&pDepthFrameSource) != S_OK)
			{
				cerr << "Can't get depth frame source" << endl;
			}
			else
			{
				// get frame reader
				IDepthFrameReader* pDepthFrameReader = nullptr;
				if (pDepthFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
				{
					cerr << "Can't get depth frame reader" << endl;
				}
				else
				{
					// Enter main loop
					size_t uFrameCount = 0;
					while (uFrameCount < 100)
					{
						IDepthFrame* pFrame = nullptr;
						if( pDepthFrameReader->AcquireLatestFrame(&pFrame) == S_OK )
						{
							// Get frame description
							int		iWidth = 0;
							int		iHeight = 0;
							IFrameDescription* pFrameDescription = nullptr;
							pFrame->get_FrameDescription(&pFrameDescription);
							pFrameDescription->get_Width(&iWidth);
							pFrameDescription->get_Height(&iHeight);

							// Get image buffer
							UINT	iBufferSize = 0;
							UINT16*	pBuffer = nullptr;
							pFrame->AccessUnderlyingBuffer(&iBufferSize, &pBuffer);

							std::cout << pBuffer[(iWidth / 2) + iWidth *(iHeight / 2)] << std::endl;

							pFrame->Release();
							++uFrameCount;
						}
					}

					// release frame reader
					pDepthFrameReader->Release();
					pDepthFrameReader = nullptr;
				}

				// release Frame source
				pDepthFrameSource->Release();
				pDepthFrameSource = nullptr;
			}

			// Close Sensor
			pSensor->Close();
		}

		// Release Sensor
		pSensor->Release();
		pSensor = nullptr;
	}
	return 0;
}
