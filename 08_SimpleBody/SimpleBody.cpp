// Created by Heresy @ 2015/02/25
// Blog Page: 
// This sample is used to read information of body joint nad output to console.

// Standard Library
#include <iostream>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

// output operator for CameraSpacePoint
ostream& operator<<(ostream& rOS, const CameraSpacePoint& rPos )
{
	rOS << "(" << rPos.X << "/" << rPos.Y << "/" << rPos.Z << ")";
	return rOS;
}

// output operator for Vector4
ostream& operator<<(ostream& rOS, const Vector4& rVec)
{
	rOS << "(" << rVec.x << "/" << rVec.y << "/" << rVec.z << "/" << rVec.w << ")";
	return rOS;
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

	// 2a. Get frame source
	cout << "Try to get body source" << endl;
	IBodyFrameSource* pFrameSource = nullptr;
	if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK)
	{
		cerr << "Can't get body frame source" << endl;
		return -1;
	}

	// 2b. Get the number of body
	INT32 iBodyCount = 0;
	if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK)
	{
		cerr << "Can't get body count" << endl;
		return -1;
	}
	cout << " > Can trace " << iBodyCount << " bodies" << endl;
	IBody** aBody = new IBody*[iBodyCount];
	for (int i = 0; i < iBodyCount; ++i)
		aBody[i] = nullptr;

	// 3a. get frame reader
	cout << "Try to get body frame reader" << endl;
	IBodyFrameReader* pFrameReader = nullptr;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get body frame reader" << endl;
		return -1;
	}

	// 2b. release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;

	// Enter main loop
	int iStep = 0;
	while (iStep < 1000)
	{
		// 4a. Get last frame
		IBodyFrame* pFrame = nullptr;
		if (pFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
		{
			++iStep;

			// 4b. get Body data
			if (pFrame->GetAndRefreshBodyData(iBodyCount, aBody) == S_OK)
			{
				int iTrackedBodyCount = 0;

				// 4c. for each body
				for (int i = 0; i < iBodyCount; ++i)
				{
					IBody* pBody = aBody[i];

					// check if is tracked
					BOOLEAN bTracked = false;
					if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
					{
						++iTrackedBodyCount;
						cout << "User " << i << " is under tracking" << endl;

						// get joint position
						Joint aJoints[JointType::JointType_Count];
						if (pBody->GetJoints(JointType::JointType_Count, aJoints) != S_OK)
						{
							cerr << "Get joints fail" << endl;
						}

						// get joint orientation
						JointOrientation aOrientations[JointType::JointType_Count];
						if (pBody->GetJointOrientations(JointType::JointType_Count, aOrientations) != S_OK)
						{
							cerr << "Get joints fail" << endl;
						}

						// output information
						JointType eJointType = JointType::JointType_HandRight;
						const Joint& rJointPos = aJoints[JointType::JointType_HandRight];

						cout << " > Right Hand is ";
						if (rJointPos.TrackingState == TrackingState_NotTracked)
						{
							cout << "not tracked" << endl;
						}
						else
						{
							if (rJointPos.TrackingState == TrackingState_Inferred)
							{
								cout << "inferred ";
							}
							else if (rJointPos.TrackingState == TrackingState_Tracked)
							{
								cout << "tracked ";
							}

							const JointOrientation& rJointOri = aOrientations[eJointType];

							cout << "at " << rJointPos.Position << ",\n\t orientation: " << rJointOri.Orientation << endl;
						}
					}
				}

				if (iTrackedBodyCount > 0)
					cout << "Total " << iTrackedBodyCount << " bodies in this time\n" << endl;
			}
			else
			{
				cerr << "Can't read body data" << endl;
			}

			// 4e. release frame
			pFrame->Release();
		}
	}

	// delete body data array
	delete[] aBody;

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
