// Basic vector and camera operation for OpenGL 3D
//
// 		by Heresy
// 		http://kheresy.wordpress.com
//
// version 1.02 @2013/03/11

#pragma once

#include <math.h>

// glut Header
#include <GL/glut.h>

class Vector3
{
public:
	float v[3];

public:
	Vector3()
	{
		v[0] = 0;
		v[1] = 0;
		v[2] = 0;
	}

	Vector3( float x, float y, float z )
	{
		v[0] = x;
		v[1] = y;
		v[2] = z;
	}

	Vector3( const Vector3& rV )
	{
		v[0] = rV.v[0];
		v[1] = rV.v[1];
		v[2] = rV.v[2];
	}

	Vector3 operator+( const Vector3& rV2 ) const
	{
		return Vector3( v[0] + rV2.v[0], v[1] + rV2.v[1], v[2] + rV2.v[2] );
	}

	Vector3 operator-( const Vector3& rV2 ) const
	{
		return Vector3( v[0] - rV2.v[0], v[1] - rV2.v[1], v[2] - rV2.v[2] );
	}

	Vector3 operator*( float fScale ) const
	{
		return Vector3( v[0] * fScale, v[1] * fScale, v[2] * fScale );
	}

	float Length() const
	{
		return sqrt( v[0] * v[0] + v[1] * v[1] + v[2] * v[2] );
	}

	Vector3 Normalize() const
	{
		float fLength = Length();
		return Vector3( v[0] / fLength, v[1] / fLength, v[2] / fLength );
	}

	Vector3 Cross( const Vector3& rV ) const
	{
		return Vector3( v[1] * rV.v[2] - v[2] * rV.v[1],
						v[2] * rV.v[0] - v[0] * rV.v[2],
						v[0] * rV.v[1] - v[1] * rV.v[0] );
	}

	float Dot( const Vector3& rV ) const
	{
		return v[0] * rV.v[0] + v[1] * rV.v[1] + v[2] * rV.v[2];
	}
};

class Matrix3
{
public:
	float m[3][3];

public:
	Matrix3()
	{
		m[0][0] = 1;	m[0][1] = 0;	m[0][2] = 0;
		m[1][0] = 0;	m[1][1] = 1;	m[1][2] = 0;
		m[2][0] = 0;	m[2][1] = 0;	m[2][2] = 1;
	}

	Matrix3( float w, float x, float y, float z )
	{
		// normalize
		const float n = 1.0f / sqrt( x * x + y * y + z * z + w * w );
		x *= n;
		y *= n;
		z *= n;
		w *= n;

		m[0][0] = 1 - 2 * y * y - 2 * z * z;
		m[0][1] = 2 * x * y - 2 * z * w;
		m[0][2] = 2 * x * z + 2 * y * w;
		m[1][0] = 2 * x * y + 2 * z * w;
		m[1][1] = 1 - 2 * x * x - 2 * z * z;
		m[1][2] = 2 * y * z - 2 * x * w;
		m[2][0] = 2 * x * z - 2 * y * w;
		m[2][1] = 2 * y * z + 2 * x * w;
		m[2][2] = 1 - 2 * x * x - 2 * y * y;
	}

	Vector3 operator*( const Vector3& rV ) const
	{
		return Vector3( m[0][0] * rV.v[0] + m[0][1] * rV.v[1] + m[0][2] * rV.v[2],
						m[1][0] * rV.v[0] + m[1][1] * rV.v[1] + m[1][2] * rV.v[2],
						m[2][0] * rV.v[0] + m[2][1] * rV.v[1] + m[2][2] * rV.v[2] );
	}

	Matrix3 operator*( const Matrix3& rM ) const
	{
		Matrix3 mR;
		for( int y = 0; y < 3; ++ y )
		{
			for( int x = 0; x < 3; ++ x )
			{
				float v = 0;
				for( int i = 0; i < 3; ++ i )
					v += m[y][i] * rM.m[i][x];

				mR.m[x][y] = v;
			}
		}
		return mR;
	}
};

// A class of simple camera control
class SimpleCamera
{
public:
	Vector3	vPosition;
	Vector3	vCenter;
	Vector3	vLook;
	Vector3	vUpper;
	Vector3	vSide;

	void Update()
	{
		vLook = ( vCenter - vPosition ).Normalize();
		vSide = vLook.Cross( vUpper );
		vUpper = vSide.Cross( vLook );
	}

	void SetCamera()
	{
		Update();
		glMatrixMode( GL_MODELVIEW );
		glLoadIdentity();
		gluLookAt(	vPosition.v[0],	vPosition.v[1],	vPosition.v[2],
					vCenter.v[0],	vCenter.v[1],	vCenter.v[2],
					vUpper.v[0],	vUpper.v[1],	vUpper.v[2] );
	}

	void MoveSide( float fDistance )
	{
		vPosition	= vPosition	+ vSide * fDistance;
		vCenter		= vCenter	+ vSide * fDistance;
		SetCamera();
	}

	void MoveForward( float fDistance )
	{
		vPosition	= vPosition	+ vLook * fDistance;
		vCenter		= vCenter	+ vLook * fDistance;
		SetCamera();
	}

	void MoveUp( float fDistance )
	{
		vPosition	= vPosition	+ vUpper * fDistance;
		vCenter		= vCenter	+ vUpper * fDistance;
		SetCamera();
	}

	void RotateUp( float fR )
	{
		fR *= ( vPosition - vCenter ).Length();
		vCenter = vCenter + vUpper * fR;
		SetCamera();
	}

	void RotateSide( float fR )
	{
		fR *= ( vPosition - vCenter ).Length();
		vCenter = vCenter + vSide * fR;
		SetCamera();
	}
};
