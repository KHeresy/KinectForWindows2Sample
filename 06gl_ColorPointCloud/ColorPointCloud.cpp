// Created by Heresy @ 2015/01/30
// Blog Page: https://kheresy.wordpress.com/2015/02/02/k4w-v2-c-part-5-simple-background-remove/
// This sample will use body index frme to filter color frame, show an user only im age with OpenCV.
// ICoordinateMapper is required to map different image coordination.

// Standard Library
#include <array>
#include <iostream>

// freeglut header
#include <GL/freeglut.h>

// Kinect for Windows SDK Header
#include <Kinect.h>

using namespace std;

// global objects
IKinectSensor*		pSensor				= nullptr;
IColorFrameReader*	pColorFrameReader	= nullptr;
IDepthFrameReader*	pDepthFrameReader	= nullptr;
ICoordinateMapper*	pCoordinateMapper	= nullptr;

int		iColorWidth		= 0,
		iColorHeight	= 0;
UINT	uDepthPointNum		= 0;
UINT	uColorPointNum		= 0;
UINT	uColorBufferSize	= 0;

UINT16*	pDepthBuffer = nullptr;
BYTE*	pColorBuffer = nullptr;
CameraSpacePoint* pCSPoints = nullptr;

// glut display function(draw)
void display()
{
	// clear previous screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw points
	glPointSize(1.0f);
	glBegin(GL_POINTS);

	for (int y = 0; y < iColorHeight; ++y)
	{
		for (int x = 0; x < iColorWidth; ++x)
		{
			int idx = x + y * iColorWidth;
			const CameraSpacePoint& rPt = pCSPoints[idx];
			if (rPt.Z > 0)
			{
				glColor4ubv((const GLubyte*)(&pColorBuffer[4 * idx]));
				glVertex3f(rPt.X * 1000, rPt.Y * 1000, rPt.Z * 1000);
			}
		}
	}
	glEnd();

	// Coordinate
	glLineWidth(5.0f);
	glBegin(GL_LINES);
	glColor3ub(255, 0, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(100, 0, 0);

	glColor3ub(0, 255, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 100, 0);

	glColor3ub(0, 0, 255);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 100);
	glEnd();

	// swap buffer
	glutSwapBuffers();
}

// glut idle function
void idle()
{
	bool bUpdated = false;

	// Read depth data
	IDepthFrame* pDFrame = nullptr;
	if (pDepthFrameReader->AcquireLatestFrame(&pDFrame) == S_OK)
	{
		pDFrame->CopyFrameDataToArray(uDepthPointNum, pDepthBuffer);

		pDFrame->Release();
		pDFrame = nullptr;

		bUpdated = true;
	}

	// Read color data
	IColorFrame* pCFrame = nullptr;
	if (pColorFrameReader->AcquireLatestFrame(&pCFrame) == S_OK)
	{
		pCFrame->CopyConvertedFrameDataToArray(uColorBufferSize, pColorBuffer, ColorImageFormat_Rgba);

		pCFrame->Release();
		pCFrame = nullptr;

		bUpdated = true;
	}

	if (bUpdated)
	{
		pCoordinateMapper->MapColorFrameToCameraSpace(uDepthPointNum, pDepthBuffer, uColorPointNum, pCSPoints);
		glutPostRedisplay();
	}
}

// glut keyboard function
void keyboard(unsigned char key, int x, int y)
{
}

// glut special keyboard function
void specialKey(int key, int x, int y)
{
}

void ExitFunction()
{
	// release buffer
	delete[] pDepthBuffer;
	delete[] pColorBuffer;
	delete[] pCSPoints;

	// release coordinate mapper
	pCoordinateMapper->Release();
	pCoordinateMapper = nullptr;

	// release frame reader
	pColorFrameReader->Release();
	pColorFrameReader = nullptr;
	pDepthFrameReader->Release();
	pDepthFrameReader = nullptr;

	// Close and Release Sensor
	pSensor->Close();
	pSensor->Release();
	pSensor = nullptr;
}

int main(int argc, char** argv)
{
	// 1. Sensor related code
	cout << "Try to get default sensor" << endl;
	{
		if (GetDefaultKinectSensor(&pSensor) != S_OK)
		{
			cerr << "Get Sensor failed" << endl;
			return -1;
		}

		cout << "Try to open sensor" << endl;
		if (pSensor->Open() != S_OK)
		{
			cerr << "Can't open sensor" << endl;
			return -1;
		}
	}

	// 2. Color related code
	cout << "Try to get color source" << endl;
	{
		// Get frame source
		IColorFrameSource* pFrameSource = nullptr;
		if (pSensor->get_ColorFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get color frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get color frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			pFrameDescription->get_Width(&iColorWidth);
			pFrameDescription->get_Height(&iColorHeight);

			uColorPointNum = iColorWidth * iColorHeight;
			uColorBufferSize = uColorPointNum * 4 * sizeof(BYTE);

			pCSPoints = new CameraSpacePoint[uColorPointNum];
			pColorBuffer = new BYTE[4*uColorPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get color frame reader" << endl;
		if (pFrameSource->OpenReader(&pColorFrameReader) != S_OK)
		{
			cerr << "Can't get color frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 3. Depth related code
	cout << "Try to get depth source" << endl;
	{
		// Get frame source
		IDepthFrameSource* pFrameSource = nullptr;
		if (pSensor->get_DepthFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get depth frame source" << endl;
			return -1;
		}

		// Get frame description
		cout << "get depth frame description" << endl;
		IFrameDescription* pFrameDescription = nullptr;
		if (pFrameSource->get_FrameDescription(&pFrameDescription) == S_OK)
		{
			int	iDepthWidth = 0,
				iDepthHeight = 0;
			pFrameDescription->get_Width(&iDepthWidth);
			pFrameDescription->get_Height(&iDepthHeight);
			uDepthPointNum = iDepthWidth * iDepthHeight;
			pDepthBuffer = new UINT16[uDepthPointNum];
		}
		pFrameDescription->Release();
		pFrameDescription = nullptr;

		// get frame reader
		cout << "Try to get depth frame reader" << endl;
		if (pFrameSource->OpenReader(&pDepthFrameReader) != S_OK)
		{
			cerr << "Can't get depth frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 4. Coordinate Mapper
	if (pSensor->get_CoordinateMapper(&pCoordinateMapper) != S_OK)
	{
		cerr << "get_CoordinateMapper failed" << endl;
		return -1;
	}

	// 5. initial glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);

	glutInitWindowSize(640, 480);
	glutCreateWindow("OpenNI 3D OpenGL 3D Point");

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	glMatrixMode(GL_PROJECTION);
	gluPerspective(40.0, 1.0, 300.0, 20000.0);	// FOV, aspect ration, near, far

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 100, -2000,
		0,0,1000,
		0, 1, 0 );

	// register glut callback functions
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKey);

	// STD Exit function
	atexit(ExitFunction);

	// start
	glutMainLoop();

	return 0;
}
