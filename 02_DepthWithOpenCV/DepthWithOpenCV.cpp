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
					while (true)
					{
						// 4a. Get last frame
						IDepthFrame* pFrame = nullptr;
						if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
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

							// 4d. convert to OpenCV form
							const cv::Mat mRawDepthImg(iHeight, iWidth, CV_16UC1, pBuffer );
							cv::Mat mImg;
							mRawDepthImg.convertTo(mImg, CV_8U, 255.0f / 8000.0f);
							cv::imshow( "Depth Map", mImg);

							// 4e. release frame
							pFrame->Release();
						}

						// 4f. check keyboard input
						if (cv::waitKey(30) == VK_ESCAPE){
							break;
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
