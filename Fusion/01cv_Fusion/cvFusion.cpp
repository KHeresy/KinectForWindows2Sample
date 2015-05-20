// Created by Heresy @ 2015/05/20
// Blog Page: 
// This sample is used to use Kinect Fusion to scan the scene and output a STL file.

// Standard Library
#include <iostream>
#include <fstream>

// OpenCV Header
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Kinect for Windows SDK Header
#include <Kinect.h>
#include <NuiKinectFusionApi.h>

using namespace std;

bool OutputSTL(INuiFusionMesh* pMesh)
{
	UINT	uVertexNum = pMesh->VertexCount(),
			uIndexNum = pMesh->TriangleVertexIndexCount();
	cout << "Build result: " << uVertexNum << " vertices and " << uIndexNum << " indeices" << endl;

	// get vertex
	const Vector3* pVertex = nullptr;
	pMesh->GetVertices(&pVertex);

	// get index
	const int* pIndex = nullptr;
	pMesh->GetTriangleIndices(&pIndex);

	// output to file
	ofstream OutFile("E:\\test.stl");
	OutFile << "solid FusionResult\n";
	for (UINT idx = 0; idx < uIndexNum; idx += 3)
	{
		const Vector3	&rP1 = pVertex[idx],
						&rP2 = pVertex[idx+1],
						&rP3 = pVertex[idx+2];
		OutFile << "  facet normal 0 0 0\n";
		OutFile << "    outer loop\n";
		OutFile << "      vertex " << -rP1.x << " " << rP1.y << " " << rP1.z << "\n";
		OutFile << "      vertex " << -rP2.x << " " << rP2.y << " " << rP2.z << "\n";
		OutFile << "      vertex " << -rP3.x << " " << rP3.y << " " << rP3.z << "\n";
		OutFile << "    endloop" << "\n";
		OutFile << "  endfacet" << "\n";
	}
	OutFile << "endsolid FusionResult\n";
	OutFile.close();

	cout << "Done" << endl;

	return true;
}

int main(int argc, char** argv)
{
	#pragma region Regular Kinect Code
	// Get default Sensor
	cout << "Try to get default sensor" << endl;
	IKinectSensor* pSensor = nullptr;
	if (GetDefaultKinectSensor(&pSensor) != S_OK)
	{
		cerr << "Get Sensor failed" << endl;
		return -1;
	}

	// Open sensor
	cout << "Try to open sensor" << endl;
	if (pSensor->Open() != S_OK)
	{
		cerr << "Can't open sensor" << endl;
		return -1;
	}

	// Depth Related code
	IDepthFrameReader* pDepthFrameReader = nullptr;
	int		iWidth = 0;
	int		iHeight = 0;
	{
		// Get depth frame source
		cout << "Try to get depth source" << endl;
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) == S_OK)
		{
			// Get frame description
			cout << "get depth frame description" << endl;
			IFrameDescription* pFrameDescription = nullptr;
			if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
			{
				pFrameDescription->get_Width(&iWidth);
				pFrameDescription->get_Height(&iHeight);
			}
			pFrameDescription->Release();
			pFrameDescription = nullptr;

			// 2c. get frame reader
			cout << "Try to get depth frame reader" << endl;
			if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
			{
				cerr << "Can't get depth frame reader" << endl;
				return -1;
			}

			// 2d. release Frame source
			cout << "Release frame source" << endl;
			pFrameSource->Release();
			pFrameSource = nullptr;
		}
		else
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}
	}
	#pragma endregion

	#pragma region Kinect Fusion Code
	// Volume size and resolution configuration
	NUI_FUSION_RECONSTRUCTION_PARAMETERS mResolution;
	mResolution.voxelsPerMeter = 64;
	mResolution.voxelCountX = 256;
	mResolution.voxelCountY = 256;
	mResolution.voxelCountZ = 256;

	// create INuiFusionReconstruction
	INuiFusionReconstruction* pReconstruction = nullptr;
	if (NuiFusionCreateReconstruction(&mResolution, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, nullptr, &pReconstruction) != S_OK)
	{
		cerr << "Kinect Fusion object create failed" << endl;
		return -1;
	}

	#pragma region Create depth data image frame
	// Depth
	NUI_FUSION_IMAGE_FRAME	*pFloatDepthFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iWidth, iHeight, nullptr, &pFloatDepthFrame)!= S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << endl;
		return -1;
	}

	// SmoothDepth
	NUI_FUSION_IMAGE_FRAME	*pSmoothDepthFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iWidth, iHeight, nullptr, &pSmoothDepthFrame) != S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << endl;
		return -1;
	}
	#pragma endregion

	#pragma region Create Image Frame for display
	int iImgWidth = iWidth,
		iImgHeight = iHeight;

	// Point Cloud
	NUI_FUSION_IMAGE_FRAME	*pPointCloudFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, iImgWidth, iImgHeight, nullptr, &pPointCloudFrame) != S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( POINT_CLOUD )" << endl;
		return -1;
	}

	// Surface
	NUI_FUSION_IMAGE_FRAME	*pSurfaceFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iImgWidth, iImgHeight, nullptr, &pSurfaceFrame) != S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << endl;
		return -1;
	}

	// Normal
	NUI_FUSION_IMAGE_FRAME	*pNormalFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iImgWidth, iImgHeight, nullptr, &pNormalFrame) != S_OK){
		cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << endl;
		return -1;
	}
	#pragma endregion

	#pragma endregion

	// Enter main loop
	cv::namedWindow("Surface");
	cv::namedWindow("Normal");

	Matrix4 mCameraMatrix;
	Matrix4 mColorTransform = { 0.0f };
	mColorTransform.M11 = mResolution.voxelsPerMeter / mResolution.voxelCountX;
	mColorTransform.M22 = mResolution.voxelsPerMeter / mResolution.voxelCountY;
	mColorTransform.M33 = mResolution.voxelsPerMeter / mResolution.voxelCountZ;
	mColorTransform.M41 = 0.5f;
	mColorTransform.M42 = 0.5f;
	mColorTransform.M43 = 0.0f;
	mColorTransform.M44 = 1.0f;

	while (true)
	{
		// Get last frame
		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			// read data
			UINT	uBufferSize = 0;
			UINT16*	pDepthBuffer = nullptr;
			if (pDepthFrame->AccessUnderlyingBuffer(&uBufferSize, &pDepthBuffer)==S_OK)
			{
				// Convert to Kinect Fusion format
				if (pReconstruction->DepthToDepthFloatFrame(pDepthBuffer, uBufferSize * sizeof(UINT16), pFloatDepthFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH, true) == S_OK)
				{
					// smoothing
					if (pReconstruction->SmoothDepthFloatFrame(pFloatDepthFrame, pSmoothDepthFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD) == S_OK)
					{
						// Reconstruction Process
						pReconstruction->GetCurrentWorldToCameraTransform(&mCameraMatrix);
						if (pReconstruction->ProcessFrame(pSmoothDepthFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, nullptr, &mCameraMatrix) != S_OK)
						{
							cerr << "Can't process this frame" << endl;
						}

						// Calculate Point Cloud
						if (pReconstruction->CalculatePointCloud(pPointCloudFrame, &mCameraMatrix) == S_OK)
						{
							// Shading Point Clouid 
							if (NuiFusionShadePointCloud(pPointCloudFrame, &mCameraMatrix, &mColorTransform, pSurfaceFrame, pNormalFrame) == S_OK)
							{
								cv::Mat surfaceMat(iHeight, iWidth, CV_8UC4, pSurfaceFrame->pFrameBuffer->pBits);
								cv::Mat normalMat(iHeight, iWidth, CV_8UC4, pNormalFrame->pFrameBuffer->pBits);

								cv::imshow("Surface", surfaceMat);
								cv::imshow("Normal", normalMat);
							}
						}
					}
				}
			}

			// release frame
			pDepthFrame->Release();
		}

		int key = cv::waitKey(30);
		if (key == VK_ESCAPE)
		{
			break;
		}
		else if (key == 'r')
		{
			cout << "Reset Reconstruction" << endl;
			pReconstruction->ResetReconstruction(nullptr, nullptr);
		}
		else if (key == 'o')
		{
			INuiFusionMesh* pMesh = nullptr;
			if (pReconstruction->CalculateMesh(1, &pMesh) == S_OK)
			{
				OutputSTL(pMesh);
				pMesh->Release();
			}
		}
	}

	#pragma region Release resource
	NuiFusionReleaseImageFrame(pFloatDepthFrame);
	NuiFusionReleaseImageFrame(pSmoothDepthFrame);
	NuiFusionReleaseImageFrame(pPointCloudFrame);
	NuiFusionReleaseImageFrame(pSurfaceFrame);
	NuiFusionReleaseImageFrame(pNormalFrame);

	// release color frame reader
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	pSensor->Release();
	pSensor = nullptr;
	#pragma endregion

	return 0;
}
