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

bool OutputOBJ(INuiFusionColorMesh* pMesh)
{
	UINT	uVertexNum = pMesh->VertexCount(),
			uIndexNum = pMesh->TriangleVertexIndexCount();
	cout << "Build result: " << uVertexNum << " vertices and " << uIndexNum << " indeices" << endl;

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
	ofstream OutFile("E:\\test.obj");
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

	// Color related data
	IColorFrameReader* pColorFrameReader = nullptr;
	int		iColorWidth = 0;
	int		iColorHeight = 0;
	UINT	uColorBufferSize = 0;
	BYTE*	pColorData = nullptr;

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
				pColorData = new BYTE[iColorWidth * iColorHeight * 4];
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

	// Depth related data
	IDepthFrameReader* pDepthFrameReader = nullptr;
	int		iDepthWidth = 0;
	int		iDepthHeight = 0;
	UINT	uDepthBufferSize = 0;
	UINT	uDepthPointNum = 0;
	ColorSpacePoint* pColorPoint = nullptr;

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
				pColorPoint = new ColorSpacePoint[uDepthPointNum];
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
	mResolution.voxelsPerMeter = 64;
	mResolution.voxelCountX = 256;
	mResolution.voxelCountY = 256;
	mResolution.voxelCountZ = 256;

	// create INuiFusionReconstruction
	INuiFusionColorReconstruction* pReconstruction = nullptr;
	if (NuiFusionCreateColorReconstruction(&mResolution, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, nullptr, &pReconstruction) != S_OK)
	{
		cerr << "Kinect Fusion object create failed" << endl;
		return -1;
	}

	#pragma region Image frame from data source
	// Color
	NUI_FUSION_IMAGE_FRAME	*pColorImageFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_COLOR, iDepthWidth, iDepthHeight, nullptr, &pColorImageFrame) != S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( COLOR )" << endl;
		return -1;
	}

	// Depth
	NUI_FUSION_IMAGE_FRAME	*pFloatDepthFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iDepthWidth, iDepthHeight, nullptr, &pFloatDepthFrame)!= S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << endl;
		return -1;
	}

	// SmoothDepth
	NUI_FUSION_IMAGE_FRAME	*pSmoothDepthFrame = nullptr;
	if (NuiFusionCreateImageFrame(NUI_FUSION_IMAGE_TYPE_FLOAT, iDepthWidth, iDepthHeight, nullptr, &pSmoothDepthFrame) != S_OK)
	{
		cerr << "Error : NuiFusionCreateImageFrame( FLOAT )" << endl;
		return -1;
	}
	#pragma endregion
	
	#pragma region Create Image Frame for display
	int iImgWidth	= iDepthWidth,
		iImgHeight	= iDepthHeight;

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
		#pragma region Read new data and convert to NUI_FUSION_IMAGE_FRAME
		// Get last depth frame
		IDepthFrame* pDepthFrame = nullptr;
		if (pDepthFrameReader->AcquireLatestFrame(&pDepthFrame) == S_OK)
		{
			// read data
			UINT	uBufferSize = 0;
			UINT16*	pDepthBuffer = nullptr;
			if (pDepthFrame->AccessUnderlyingBuffer(&uBufferSize, &pDepthBuffer) == S_OK)
			{
				// build color-depth mapping table
				if (pMapper->MapDepthFrameToColorSpace(uDepthPointNum, pDepthBuffer, uDepthPointNum, pColorPoint) != S_OK)
				{
					cerr << "Can't map depth to color" << endl;
				}

				// Convert to Kinect Fusion format
				if (pReconstruction->DepthToDepthFloatFrame(pDepthBuffer, uDepthBufferSize, pFloatDepthFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH, true) == S_OK)
				{
					// smoothing
					if (pReconstruction->SmoothDepthFloatFrame(pFloatDepthFrame, pSmoothDepthFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD) != S_OK)
					{
						cerr << "Dpeth Frame smooth failed" << endl;
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
				// fill the color image frame with color-depth mapping table
				BYTE* pColorArray = pColorImageFrame->pFrameBuffer->pBits;
				for (unsigned int i = 0; i < uDepthPointNum; ++i)
				{
					const ColorSpacePoint& rPt = pColorPoint[i];
					if (rPt.X >= 0 && rPt.X < iColorWidth && rPt.Y >= 0 && rPt.Y < iColorHeight)
					{
						int idx = 4 * ( (int)rPt.X + iColorWidth * (int)rPt.Y );
						pColorArray[4 * i] = pColorData[idx];
						pColorArray[4 * i + 1] = pColorData[idx + 1];
						pColorArray[4 * i + 2] = pColorData[idx + 2];
						pColorArray[4 * i + 3] = pColorData[idx + 3];
					}
					else
					{
						pColorArray[4 * i] = 0;
						pColorArray[4 * i + 1] = 0;
						pColorArray[4 * i + 2] = 0;
						pColorArray[4 * i + 3] = 0;
					}
				}
			}
			pColorFrame->Release();
		}
		#pragma endregion

		// Reconstruction Process
		pReconstruction->GetCurrentWorldToCameraTransform(&mCameraMatrix);
		if (pReconstruction->ProcessFrame(pSmoothDepthFrame, pColorImageFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES, nullptr, &mCameraMatrix) != S_OK)
		{
			cerr << "Can't process this frame" << endl;
		}

		// Calculate Point Cloud
		if (pReconstruction->CalculatePointCloud(pPointCloudFrame, pColorImageFrame, &mCameraMatrix) == S_OK)
		{
			// Shading Point Clouid
			if (NuiFusionShadePointCloud(pPointCloudFrame, &mCameraMatrix, &mColorTransform, pSurfaceFrame, pNormalFrame) == S_OK)
			{
				cv::Mat surfaceMat(iDepthHeight, iDepthWidth, CV_8UC4, pSurfaceFrame->pFrameBuffer->pBits);
				cv::Mat normalMat(iDepthHeight, iDepthWidth, CV_8UC4, pNormalFrame->pFrameBuffer->pBits);

				cv::imshow("Surface", surfaceMat);
				cv::imshow("Normal", normalMat);
			}
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
			INuiFusionColorMesh* pMesh = nullptr;
			if (pReconstruction->CalculateMesh(1, &pMesh) == S_OK)
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
	NuiFusionReleaseImageFrame(pFloatDepthFrame);
	NuiFusionReleaseImageFrame(pSmoothDepthFrame);
	NuiFusionReleaseImageFrame(pPointCloudFrame);
	NuiFusionReleaseImageFrame(pSurfaceFrame);
	NuiFusionReleaseImageFrame(pNormalFrame);

	// release frame reader
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;

	pMapper->Release();
	pMapper = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	pSensor->Release();
	pSensor = nullptr;
	#pragma endregion

	return 0;
}
