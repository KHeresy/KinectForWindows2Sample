// Created by Heresy @ 2015/08/11
// Blog Page: 
// This sample is used to read the gesture databases from Visual Gesture Builder and detect gesture.

// Standard Library
#include <string>
#include <iostream>

// Kinect for Windows SDK Header
#include <Kinect.h>
#include <Kinect.VisualGestureBuilder.h>

using namespace std;

int main(int argc, char** argv)
{
	#pragma region Sensor related code
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
	#pragma endregion

	#pragma region Body releated code
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

	// 2c. get frame reader
	cout << "Try to get body frame reader" << endl;
	IBodyFrameReader* pFrameReader = nullptr;
	if (pFrameSource->OpenReader(&pFrameReader) != S_OK)
	{
		cerr << "Can't get body frame reader" << endl;
		return -1;
	}

	// 2d. release Frame source
	cout << "Release frame source" << endl;
	pFrameSource->Release();
	pFrameSource = nullptr;
	#pragma endregion

	#pragma region Visual Gesture Builder Database
	// Load gesture dataase from File
	wstring sDatabaseFile = L"test.gbd";
	IVisualGestureBuilderDatabase* pGestureDatabase = nullptr;
	wcout << L"Try to load gesture database file " << sDatabaseFile << endl;
	if (CreateVisualGestureBuilderDatabaseInstanceFromFile(sDatabaseFile.c_str(), &pGestureDatabase) != S_OK)
	{
		wcerr << L"Can't read database file " << sDatabaseFile << endl;
		return -1;
	}

	// Get how many gestures in database
	UINT iGestureCount = 0;
	cout << "Try to read gesture list" << endl;
	if (pGestureDatabase->get_AvailableGesturesCount(&iGestureCount) != S_OK)
	{
		cerr << "Can't read the gesture count" << endl;
		return -1;
	}
	if (iGestureCount == 0)
	{
		cerr << "There is no gesture in the database" << endl;
		return -1;
	}
	
	// get the list of gestures
	IGesture** pGestureList = new IGesture*[iGestureCount];
	if (pGestureDatabase->get_AvailableGestures(iGestureCount, pGestureList) != S_OK)
	{
		cerr << "Can't read the gesture list" << endl;
		return -1;
	}

	// output the gesture list
	cout << "There are " << iGestureCount << " gestures in the database: " << endl;
	if (pGestureList != nullptr)
	{
		GestureType mType;
		const UINT uTextLength = 260; // magic number, if value smaller than 260, can't get name
		wchar_t sName[uTextLength];

		for (int i = 0; i < iGestureCount; ++i)
		{
			if (pGestureList[i]->get_GestureType(&mType) == S_OK)
			{
				if (mType == GestureType_Discrete)
					cout << "\t[D] ";
				else if (mType == GestureType_Continuous)
					cout << "\t[C] ";

				if (pGestureList[i]->get_Name(uTextLength, sName) == S_OK)
					wcout << sName << endl;
			}
		}
	}
	#pragma endregion

	#pragma region Gesture frame related code
	// create for each possible body
	IVisualGestureBuilderFrameSource** pGestureSources = new IVisualGestureBuilderFrameSource*[iBodyCount];
	IVisualGestureBuilderFrameReader** pGestureReaders = new IVisualGestureBuilderFrameReader*[iBodyCount];
	for (int i = 0; i < iBodyCount; ++i)
	{
		// frame source
		pGestureSources[i] = nullptr;
		if (CreateVisualGestureBuilderFrameSource(pSensor, 0, &pGestureSources[i]) != S_OK)
		{
			cerr << "Can't create IVisualGestureBuilderFrameSource" << endl;
			return -1;
		}

		// set gestures
		if (pGestureSources[i]->AddGestures(iGestureCount, pGestureList) == S_OK)
		{
			for (int iGesture = 0; iGesture < iGestureCount; ++iGesture)
				pGestureSources[i]->SetIsEnabled(pGestureList[iGesture], true);
		}

		// frame reader
		pGestureReaders[i] = nullptr;
		if (pGestureSources[i]->OpenReader(&pGestureReaders[i]) != S_OK)
		{
			cerr << "Can't open IVisualGestureBuilderFrameReader" << endl;
			return -1;
		}
	}
	#pragma endregion

	// Enter main loop
	int iStep = 0;
	while (iStep < 100000)
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
						//cout << "User " << i << " is under tracking" << endl;

						UINT64 uTrackingId = 0;
						if (pBody->get_TrackingId(&uTrackingId) == S_OK)
						{
							UINT64 uGestureId = 0;
							if (pGestureSources[i]->get_TrackingId(&uGestureId) == S_OK)
							{
								if (uGestureId != uTrackingId)
								{
									pGestureSources[i]->put_TrackingId(uTrackingId);
									cout << "Gesture Source " << i << " start to track user " << uTrackingId << endl;
								}
							}
						}

						IVisualGestureBuilderFrame* pGestureFrame = nullptr;
						if (pGestureReaders[i]->CalculateAndAcquireLatestFrame(&pGestureFrame) == S_OK)
						{
							BOOLEAN bGestureTracked = false;
							if (pGestureFrame->get_IsTrackingIdValid(&bGestureTracked) == S_OK && bGestureTracked)
							{
								GestureType mType;
								for (int j = 0; j < iGestureCount; ++j)
								{
									pGestureList[j]->get_GestureType(&mType);
									if (mType == GestureType_Discrete)
									{
										IDiscreteGestureResult* pGestureResult = nullptr;
										if (pGestureFrame->get_DiscreteGestureResult(pGestureList[j], &pGestureResult) == S_OK)
										{
											BOOLEAN bDetected = false;
											if (pGestureResult->get_Detected(&bDetected) == S_OK && bDetected)
											{
												cout << "Detected Gesture" << endl;
											}
											pGestureResult->Release();
										}
									}
									else if (mType == GestureType_Continuous)
									{
										IContinuousGestureResult* pGestureResult = nullptr;
										if (pGestureFrame->get_ContinuousGestureResult(pGestureList[j], &pGestureResult) == S_OK)
										{
											float fProgress = 0.0f;
											if (pGestureResult->get_Progress(&fProgress) == S_OK)
											{
												if (fProgress > 0)
													cout << "Progress: " << fProgress << endl;
											}
										}
										pGestureResult->Release();
									}
								}
							}
							pGestureFrame->Release();
						}
					}
				}

				//if (iTrackedBodyCount > 0)
					cout << "Step " << iStep << " Total " << iTrackedBodyCount << " bodies in this time\n" << endl;
			}
			else
			{
				cerr << "Can't read body data" << endl;
			}

			// 4e. release frame
			pFrame->Release();
		}
	}

	// delete allocated data
	for (int i = 0; i < iBodyCount; ++i)
	{
		pGestureReaders[i]->Release();
		pGestureSources[i]->Release();
	}
	delete [] pGestureReaders;
	delete [] pGestureSources;

	for (int i = 0; i < iGestureCount; ++i)
		pGestureList[i]->Release();
	delete[] pGestureList;

	for (int i = 0; i < iBodyCount; ++i)
		aBody[i]->Release();
	delete[] aBody;

	// 2e. release frame reader
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
