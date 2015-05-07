// Created by Heresy @ 2015/03/03
// Blog Page: https://kheresy.wordpress.com/2015/04/10/k4w-v2-part-7a-draw-user-skeleton/
// This sample is used to show skeleton data with OpenGL.

// Standard Library
#include <array>
#include <iostream>

// freeglut header
#include <GL/freeglut.h>

// Kinect for Windows SDK Header
#include <Kinect.h>

// for OpenGL camera navigation
#include "../common/OpenGLCamera.h"

using namespace std;

// global objects
IKinectSensor*		pSensor				= nullptr;
IBodyFrameReader*	pBodyFrameReader	= nullptr;

INT32				iBodyCount = 0;
IBody**				aBody = nullptr;

SimpleCamera g_Camera;

void DrawLine(const Joint& rJ1, const Joint& rJ2)
{
	if (rJ1.TrackingState == TrackingState_NotTracked || rJ2.TrackingState == TrackingState_NotTracked)
		return;

	glVertex3f(rJ1.Position.X, rJ1.Position.Y, rJ1.Position.Z);
	glVertex3f(rJ2.Position.X, rJ2.Position.Y, rJ2.Position.Z);
}

// glut display function(draw)
void display()
{
	// clear previous screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// draw points
	glPointSize(1.0f);

	// for each body
	for (int i = 0; i < iBodyCount; ++i)
	{
		IBody* pBody = aBody[i];
		if (pBody == nullptr)
			continue;

		// check if is tracked
		BOOLEAN bTracked = false;
		if ((pBody->get_IsTracked(&bTracked) == S_OK) && bTracked)
		{
			// get joint position
			Joint aJoints[JointType::JointType_Count];
			if (pBody->GetJoints(JointType::JointType_Count, aJoints) == S_OK)
			{
				glLineWidth(1.0f);
				glBegin(GL_LINES);
					glColor3f(1.0f, 1.0f, 1.0f);

					DrawLine(aJoints[JointType_SpineBase],		aJoints[JointType_SpineMid]);
					DrawLine(aJoints[JointType_SpineMid],		aJoints[JointType_SpineShoulder]);
					DrawLine(aJoints[JointType_SpineShoulder],	aJoints[JointType_Neck]);
					DrawLine(aJoints[JointType_Neck],			aJoints[JointType_Head]);

					DrawLine(aJoints[JointType_SpineShoulder],	aJoints[JointType_ShoulderLeft]);
					DrawLine(aJoints[JointType_ShoulderLeft],	aJoints[JointType_ElbowLeft]);
					DrawLine(aJoints[JointType_ElbowLeft],		aJoints[JointType_WristLeft]);
					DrawLine(aJoints[JointType_WristLeft],		aJoints[JointType_HandLeft]);
					DrawLine(aJoints[JointType_HandLeft],		aJoints[JointType_HandTipLeft]);
					DrawLine(aJoints[JointType_HandLeft],		aJoints[JointType_ThumbLeft]);

					DrawLine(aJoints[JointType_SpineShoulder],	aJoints[JointType_ShoulderRight]);
					DrawLine(aJoints[JointType_ShoulderRight],	aJoints[JointType_ElbowRight]);
					DrawLine(aJoints[JointType_ElbowRight],		aJoints[JointType_WristRight]);
					DrawLine(aJoints[JointType_WristRight],		aJoints[JointType_HandRight]);
					DrawLine(aJoints[JointType_HandRight],		aJoints[JointType_HandTipRight]);
					DrawLine(aJoints[JointType_HandRight],		aJoints[JointType_ThumbRight]);

					DrawLine(aJoints[JointType_SpineBase],		aJoints[JointType_HipLeft]);
					DrawLine(aJoints[JointType_HipLeft],		aJoints[JointType_KneeLeft]);
					DrawLine(aJoints[JointType_KneeLeft],		aJoints[JointType_AnkleLeft]);
					DrawLine(aJoints[JointType_AnkleLeft],		aJoints[JointType_FootLeft]);

					DrawLine(aJoints[JointType_SpineBase],		aJoints[JointType_HipRight]);
					DrawLine(aJoints[JointType_HipRight],		aJoints[JointType_KneeRight]);
					DrawLine(aJoints[JointType_KneeRight],		aJoints[JointType_AnkleRight]);
					DrawLine(aJoints[JointType_AnkleRight],		aJoints[JointType_FootRight]);

				glEnd();

				glPointSize(5.0f);
				glBegin(GL_POINTS);
				for (int i = 0; i < JointType_Count; ++ i )
				{
					const Joint& rJoint = aJoints[i];
					if (rJoint.TrackingState == TrackingState_NotTracked)
						continue;
					else if(rJoint.TrackingState == TrackingState_Tracked)
						glColor3f(1.0f, 0.0f, 0.0f);
					else
						glColor3f(0.0f, 1.0f, 0.0f);
					glVertex3f(rJoint.Position.X, rJoint.Position.Y, rJoint.Position.Z);
				}
				glEnd();
			}
		}
	}

	// Coordinate
	glLineWidth(5.0f);
	glBegin(GL_LINES);
		glColor3ub(255, 0, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(1, 0, 0);

		glColor3ub(0, 255, 0);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 1, 0);

		glColor3ub(0, 0, 255);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, 1);
	glEnd();

	// swap buffer
	glutSwapBuffers();
}

// glut idle function
void idle()
{
	// Read body data
	IBodyFrame* pFrame = nullptr;
	if (pBodyFrameReader->AcquireLatestFrame(&pFrame) == S_OK)
	{
		// update body data
		pFrame->GetAndRefreshBodyData(iBodyCount, aBody);

		pFrame->Release();
		pFrame = nullptr;

		// redraw
		glutPostRedisplay();
	}
}

// glut keyboard function
void keyboard(unsigned char key, int x, int y)
{
	float fSpeed = 0.1f;
	switch (key)
	{
	case VK_ESCAPE:
		glutExit();

	case 's':
		g_Camera.MoveForward(-fSpeed);
		break;

	case 'w':
		g_Camera.MoveForward(fSpeed);
		break;

	case 'a':
		g_Camera.MoveSide(-fSpeed);
		break;

	case 'd':
		g_Camera.MoveSide(fSpeed);
		break;

	case 'z':
		g_Camera.MoveUp(-fSpeed);
		break;

	case 'x':
		g_Camera.MoveUp(fSpeed);
		break;
	}
}

// glut special keyboard function
void specialKey(int key, int x, int y)
{
	float fRotateScale = 0.01f;
	switch (key)
	{
	case GLUT_KEY_DOWN:
		g_Camera.RotateUp(-fRotateScale);
		break;

	case GLUT_KEY_UP:
		g_Camera.RotateUp(fRotateScale);
		break;

	case GLUT_KEY_RIGHT:
		g_Camera.RotateSide(fRotateScale);
		break;

	case GLUT_KEY_LEFT:
		g_Camera.RotateSide(-fRotateScale);
		break;
	}
}

void reshape(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(40.0, (float)w / h, 0.01, 50.0);

	g_Camera.SetCamera();

	glViewport(0, 0, w, h);
}

void ExitFunction()
{
	delete[] aBody;

	// release frame reader
	pBodyFrameReader->Release();
	pBodyFrameReader = nullptr;

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

	// 2. Body related code
	cout << "Try to get body source" << endl;
	{
		// Get frame source
		IBodyFrameSource* pFrameSource = nullptr;
		if (pSensor->get_BodyFrameSource(&pFrameSource) != S_OK)
		{
			cerr << "Can't get body frame source" << endl;
			return -1;
		}

		// allocate body array
		if (pFrameSource->get_BodyCount(&iBodyCount) != S_OK)
		{
			cerr << "Can't get body count" << endl;
			return -1;
		}
		cout << " > Can trace " << iBodyCount << " bodies" << endl;
		aBody = new IBody*[iBodyCount];
		for (int i = 0; i < iBodyCount; ++i)
			aBody[i] = nullptr;

		// get frame reader
		cout << "Try to get body frame reader" << endl;
		if (pFrameSource->OpenReader(&pBodyFrameReader) != S_OK)
		{
			cerr << "Can't get body frame reader" << endl;
			return -1;
		}

		// release Frame source
		cout << "Release frame source" << endl;
		pFrameSource->Release();
		pFrameSource = nullptr;
	}

	// 5. initial glut
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);

	glutInitWindowSize(640, 480);
	glutCreateWindow("Kinect OpenGL 3D Point");

	glEnable(GL_DEPTH_TEST);
	glDisable(GL_LIGHTING);

	// default camera
	g_Camera.vCenter = Vector3(0.0, 0.0, 1);
	g_Camera.vPosition = Vector3(0.0, 0.0, -2.0);
	g_Camera.vUpper = Vector3(0.0, 1.0, 0.0);

	// register glut callback functions
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKey);

	// STD Exit function
	atexit(ExitFunction);

	// start
	glutMainLoop();

	return 0;
}
