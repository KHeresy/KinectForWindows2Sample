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

void ResetReconstruction(INuiFusionColorReconstruction* pReconstruction)
{
	std::cout << "Reset Reconstruction" << std::endl;
	pReconstruction->ResetReconstruction(nullptr , nullptr);
}

bool OutputOBJ(INuiFusionColorMesh* pMesh)
{
	UINT	uVertexNum = pMesh->VertexCount(),
			uIndexNum = pMesh->TriangleVertexIndexCount();
	std::cout << "Build result: " << uVertexNum << " vertices and " << uIndexNum << " indeices" << std::endl;

	// get vertex
	const Vector3* pVertex = nullptr;
	pMesh->GetVertices(&pVertex);

	// get color
	const int* pColor = nullptr;
	pMesh->GetColors(&pColor);

	// get index
	const int* pIndex = nullptr;
	pMesh->GetTriangleIndices(&pIndex);

	// output to file
	std::ofstream OutFile("E:\\test.obj");
	OutFile << "# Vertex\n";
	for (UINT uIdx = 0; uIdx < uVertexNum; ++uIdx)
	{
		const Vector3	&rVertex = pVertex[uIdx];
		OutFile << "v " << rVertex.x << " " << rVertex.y << " " << rVertex.z << " ";
		
		const int& rColor = pColor[uIdx];
		OutFile << float(rColor & 255) / 255.0f << " " << float((rColor >> 8) & 255) / 255.0f << " " << float(rColor >> 16 & 255) / 255.0f << "\n";
	}

	OutFile << "\n# Face index\n";
	for (UINT idx = 0; idx < uIndexNum; idx += 3)
	{
		OutFile << "f " << pIndex[idx]+1 << " " << pIndex[idx + 1]+1 << " " << pIndex[idx+2]+1 << "\n";
	}
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
	IColorFrameReader* pColorFrameReader = nullptr;
	int		iDepthWidth = 0;
	int		iDepthHeight = 0;
	UINT	uDepthBufferSize = 0;
	UINT	uDepthPointNum = 0;
	int		iColorWidth = 0;
	int		iColorHeight = 0;
	UINT	uColorBufferSize = 0;

	// Get color frame source
	{
		cout << "Try to get color source" << endl;
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) == S_OK)
		{
			// Get frame description
			cout << "get color frame description" << endl;
			IFrameDescription* pFrameDescription = nullptr;
			if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
			{
				pFrameDescription->get_Width(&iColorWidth);
				pFrameDescription->get_Height(&iColorHeight);
				uColorBufferSize = iColorWidth * iColorHeight * 4 * sizeof(BYTE);
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
		}
		else
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}
	}

	// Get depth frame source
	{
		cout << "Try to get depth source" << endl;
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) == S_OK)
		{
			// Get frame description
			cout << "get depth frame description" << endl;
			IFrameDescription* pFrameDescription = nullptr;
			if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
			{
				pFrameDescription->get_Width(&iDepthWidth);
				pFrameDescription->get_Height(&iDepthHeight);
				uDepthPointNum = iDepthHeight * iDepthWidth;
				uDepthBufferSize = uDepthPointNum * sizeof(UINT16);
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

	// Coordinate Mapper
	ICoordinateMapper* pMapper = nullptr;
	if (pSensor->get_CoordinateMapper(&pMapper) != S_OK)
	{
		cerr << "Can't get Coordinate Mapper" << endl;
		return -1;
	}
	#pragma endregion

	#pragma region Kinect Fusion Code
	// Volume size and resolution configuration
	NUI_FUSION_RECONSTRUCTION_PARAMETERS mResolution;
	mResolution.voxelsPerMeter = 128;
	mResolution.voxelCountX = 256;
	mResolution.voxelCountY = 256;
	mResolution.voxelCountZ = 256;

	// create INuiFusionReconstruction
	INuiFusionColorReconstruction* pReconstruction = nullptr;
	if (NuiFusionCreateColorReconstruction(&mResolution, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, nullptr, &pReconstruction) != S_OK)
	{
		std::cerr << "Kinect Fusion object create failed" << std::endl;
		return -1;
	}

	// Create Image Frame
	NUI_FUSION_IMAGE_FRAME	*pColorImageFrame = nullptr,
							*pDepthFloatImageFrame = nullptr,
							*pSmoothDepthFloatImageFrame = nullptr,
							*pPointCloudImageFrame = nullptr,
							*pSurfaceImageFrame = nullptr,
							*pNormalImageFrame = nullptr;

	// Color
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iDepthWidth, iDepthHeight, nullptr, &pColorImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << std::endl;
		return -1;
	}

	// Depth
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iDepthWidth, iDepthHeight, nullptr, &pDepthFloatImageFrame)!= S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << std::endl;
		return -1;
	}

	// SmoothDepth
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iDepthWidth, iDepthHeight, nullptr, &pSmoothDepthFloatImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << std::endl;
		return -1;
	}

	// Point Cloud
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_POINT_CLOUD, iDepthWidth, iDepthHeight, nullptr, &pPointCloudImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( POINT_CLOUD )" << std::endl;
		return -1;
	}

	// Surface
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iDepthWidth, iDepthHeight, nullptr, &pSurfaceImageFrame) != S_OK)
	{
		std::cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << std::endl;
		return -1;
	}

	// Normal
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iDepthWidth, iDepthHeight, nullptr, &pNormalImageFrame) != S_OK){
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
	float	fColorInregrationAngle = NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES;
	Matrix4 worldToCameraTransform;

	ColorSpacePoint* pColorPoint = new ColorSpacePoint[uDepthPointNum];
	BYTE* pColorData = new BYTE[iColorWidth * iColorHeight * 4];

	while (true)
	{
		// Get last depth frame
		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			// read data
			UINT	uBufferSize = 0;
			UINT16*	pDepthBuffer = nullptr;
			if (pDepthFrame->AccessUnderlyingBuffer(&uBufferSize, &pDepthBuffer) == S_OK)
			{
				// build color mapping table
				if (pMapper->MapDepthFrameToColorSpace(uDepthPointNum, pDepthBuffer, uDepthPointNum, pColorPoint) != S_OK)
				{
					cerr << "Can't map depth to color" << endl;
				}

				// Convert to Kinect Fusion format
				if (pReconstruction->DepthToDepthFloatFrame(pDepthBuffer, uDepthBufferSize, pDepthFloatImageFrame, fDepthMin, fDepthMax, true) == S_OK)
				{
					// smoothing
					if (pReconstruction->SmoothDepthFloatFrame(pDepthFloatImageFrame, pSmoothDepthFloatImageFrame, 1, 0.04f) == S_OK)
					{
					}
				}
			}
			pDepthFrame->Release();
		}

		// Get last color frame
		IColorFrame* pColorFrame = nullptr;
		if (pColorFrameReader->AcquireLatestFrame(&pColorFrame) == S_OK)
		{
			// read data
			if (pColorFrame->CopyConvertedFrameDataToArray(uColorBufferSize, pColorData, ColorImageFormat_Rgba) == S_OK)
			{
				for (int i = 0; i < uDepthPointNum; ++i)
				{
					const ColorSpacePoint& rPt = pColorPoint[i];
					if (rPt.X >= 0 && rPt.X < iColorWidth && rPt.Y >= 0 && rPt.Y < iColorHeight)
					{
						int idx = 4 * ( (int)rPt.X + iColorWidth * (int)rPt.Y );
						pColorImageFrame->pFrameBuffer->pBits[4*i] = pColorData[idx];
						pColorImageFrame->pFrameBuffer->pBits[4*i + 1] = pColorData[idx + 1];
						pColorImageFrame->pFrameBuffer->pBits[4*i + 2] = pColorData[idx + 2];
						pColorImageFrame->pFrameBuffer->pBits[4*i + 3] = pColorData[idx + 3];
					}
				}
			}
			pColorFrame->Release();
		}

		// Reconstruction Process
		pReconstruction->GetCurrentWorldToCameraTransform(&worldToCameraTransform);
		if (pReconstruction->ProcessFrame(pSmoothDepthFloatImageFrame, pColorImageFrame, uIterationCount, uIntegrationWeight, fColorInregrationAngle, nullptr, &worldToCameraTransform) != S_OK)
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
		if (pReconstruction->CalculatePointCloud(pPointCloudImageFrame, pColorImageFrame, &worldToCameraTransform) != S_OK)
		{
			std::cerr << "Error : CalculatePointCloud" << std::endl;
			//return -1;
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

		if (NuiFusionShadePointCloud(pPointCloudImageFrame, &worldToCameraTransform, &worldToBGRTransform, pSurfaceImageFrame, pNormalImageFrame) == S_OK)
		{
			cv::Mat surfaceMat(iDepthHeight, iDepthWidth, CV_8UC4, pSurfaceImageFrame->pFrameBuffer->pBits);
			cv::Mat normalMat(iDepthHeight, iDepthWidth, CV_8UC4, pNormalImageFrame->pFrameBuffer->pBits);

			//cv::imshow("Depth", depthMat);
			cv::imshow("Surface", surfaceMat);
			cv::imshow("Normal", normalMat);
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
			INuiFusionColorMesh* pMesh = nullptr;
			if (pReconstruction->CalculateMesh(1.0, &pMesh) == S_OK)
			{
				OutputOBJ(pMesh);
				pMesh->Release();
			}
		}
	}

	#pragma region Release resource
	delete[] pColorPoint;
	delete[] pColorData;

	NuiFusionReleaseImageFrame(pColorImageFrame);
	NuiFusionReleaseImageFrame(pDepthFloatImageFrame);
	NuiFusionReleaseImageFrame(pSmoothDepthFloatImageFrame);
	NuiFusionReleaseImageFrame(pPointCloudImageFrame);
	NuiFusionReleaseImageFrame(pSurfaceImageFrame);
	NuiFusionReleaseImageFrame(pNormalImageFrame);

	// release frame reader
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	cout << "Release sensor" << endl;
	pSensor->Release();
	pSensor = nullptr;
	#pragma endregion

	return 0;
}
