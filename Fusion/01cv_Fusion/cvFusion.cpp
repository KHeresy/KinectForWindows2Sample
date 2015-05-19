// Created by Heresy @ 2015/02/25
// Blog Page: 
// This sample is used to read information of body joint nad draw with OpenCV.

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

void ResetReconstruction(INuiFusionReconstruction* pReconstruction)
{
	std::cout << "Reset Reconstruction" << std::endl;
	pReconstruction->ResetReconstruction(nullptr, nullptr);
}

bool OutputSTL(INuiFusionMesh* pMesh)
{
	UINT	uVertexNum = pMesh->VertexCount(),
			uIndexNum = pMesh->TriangleVertexIndexCount();
	std::cout << "Build result: " << uVertexNum << " vertices and " << uIndexNum << " indeices" << std::endl;

	// get vertex
	const Vector3* pVertex = nullptr;
	pMesh->GetVertices(&pVertex);

	// get index
	const int* pIndex = nullptr;
	pMesh->GetTriangleIndices(&pIndex);

	// output to file
	std::ofstream OutFile("E:\\test.stl");
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

	std::cout << "Done" << std::endl;

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
	UINT	uDepthBufferSize;
	cv::Mat	mDepthImg;
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

			// Prepare OpenCV data
			mDepthImg = cv::Mat(iHeight, iWidth, CV_16UC1);
			uDepthBufferSize = iHeight * iWidth * sizeof(UINT16);
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
		std::cerr << "Kinect Fusion object create failed" << std::endl;
		return -1;
	}

	// Create Image Frame
	NUI_FUSION_IMAGE_FRAME	*pDepthFloatImageFrame = nullptr,
							*pSmoothDepthFloatImageFrame = nullptr,
							*pPointCloudImageFrame = nullptr,
							*pSurfaceImageFrame = nullptr,
							*pNormalImageFrame = nullptr;

	// Depth
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iWidth, iHeight, nullptr, &pDepthFloatImageFrame)!= S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << std::endl;
		return -1;
	}

	// SmoothDepth
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iWidth, iHeight, nullptr, &pSmoothDepthFloatImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << std::endl;
		return -1;
	}

	// Point Cloud
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, iWidth, iHeight, nullptr, &pPointCloudImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( POINT_CLOUD )" << std::endl;
		return -1;
	}

	// Surface
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iWidth, iHeight, nullptr, &pSurfaceImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << std::endl;
		return -1;
	}

	// Normal
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iWidth, iHeight, nullptr, &pNormalImageFrame) != S_OK){
		std::cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << std::endl;
		return -1;
	}

	#pragma endregion

	// Enter main loop
	cv::namedWindow("Surface");
	cv::namedWindow("Normal");

	float	fDepthMin = NUI_FUSION_DEFAULT_MINIMUM_DEPTH,
			fDepthMax = NUI_FUSION_DEFAULT_MAXIMUM_DEPTH;
	USHORT	uIterationCount = NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT,
			uIntegrationWeight = NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT;
	Matrix4 worldToCameraTransform;

	while (true)
	{
		// Get last frame
		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			// read data
			UINT	uBufferSize		= 0;
			UINT16*	pDepthBuffer	= nullptr;
			if (pDepthFrame->AccessUnderlyingBuffer(&uBufferSize, &pDepthBuffer)==S_OK)
			{
				// Convert to Kinect Fusion format
				if (pReconstruction->DepthToDepthFloatFrame(pDepthBuffer, uDepthBufferSize, pDepthFloatImageFrame, fDepthMin, fDepthMax, true) == S_OK)
				{
					// smoothing
					if (pReconstruction->SmoothDepthFloatFrame(pDepthFloatImageFrame, pSmoothDepthFloatImageFrame, 1, 0.04f) == S_OK)
					{
						// Reconstruction Process
						pReconstruction->GetCurrentWorldToCameraTransform(&worldToCameraTransform);
						if (pReconstruction->ProcessFrame(pSmoothDepthFloatImageFrame, uIterationCount, uIntegrationWeight, nullptr, &worldToCameraTransform) != S_OK)
						{
							static int errorCount = 0;
							errorCount++;
							if (errorCount >= 100)
							{
								errorCount = 0;
								ResetReconstruction(pReconstruction);
							}
						}

						// Calculate Point Cloud
						if( pReconstruction->CalculatePointCloud(pPointCloudImageFrame, &worldToCameraTransform) != S_OK )
						{
							std::cerr << "Error : CalculatePointCloud" << std::endl;
							return -1;
						}

						// Shading Point Clouid
						Matrix4 worldToBGRTransform = { 0.0f };
						worldToBGRTransform.M11 = mResolution.voxelsPerMeter / mResolution.voxelCountX;
						worldToBGRTransform.M22 = mResolution.voxelsPerMeter / mResolution.voxelCountY;
						worldToBGRTransform.M33 = mResolution.voxelsPerMeter / mResolution.voxelCountZ;
						worldToBGRTransform.M41 = 0.5f;
						worldToBGRTransform.M42 = 0.5f;
						worldToBGRTransform.M43 = 0.0f;
						worldToBGRTransform.M44 = 1.0f;

						if( NuiFusionShadePointCloud(pPointCloudImageFrame, &worldToCameraTransform, &worldToBGRTransform, pSurfaceImageFrame, pNormalImageFrame) == S_OK )
						{
							cv::Mat surfaceMat(iHeight, iWidth, CV_8UC4, pSurfaceImageFrame->pFrameBuffer->pBits);
							cv::Mat normalMat(iHeight, iWidth, CV_8UC4, pNormalImageFrame->pFrameBuffer->pBits);

							//cv::imshow("Depth", depthMat);
							cv::imshow("Surface", surfaceMat);
							cv::imshow("Normal", normalMat);
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
			ResetReconstruction(pReconstruction);
		}
		else if (key == 'o')
		{
			INuiFusionMesh* pMesh = nullptr;
			if (pReconstruction->CalculateMesh(1.0, &pMesh) == S_OK)
			{
				OutputSTL(pMesh);
				pMesh->Release();
			}
		}
	}

	#pragma region Release resource
	NuiFusionReleaseImageFrame(pDepthFloatImageFrame);
	NuiFusionReleaseImageFrame(pSmoothDepthFloatImageFrame);
	NuiFusionReleaseImageFrame(pPointCloudImageFrame);
	NuiFusionReleaseImageFrame(pSurfaceImageFrame);
	NuiFusionReleaseImageFrame(pNormalImageFrame);

	// release color frame reader
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;
	#pragma endregion

	return 0;
}
