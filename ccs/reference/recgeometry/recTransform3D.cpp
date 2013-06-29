/*****************************************************************************
 *
 * REC Transform Library
 *
 * FILE:	recTransform.cxx
 * VERSION:	$Revision: 1.1 $  $Date: 2005/02/07 17:59:21 $
 * DESCRIPTION: functions for manipulating points, poses, and transforms
 * AUTHOR:	Stephan Roth
 * ORIGINAL C VERSION: Patrick Rowe
 *
 *                           ******************************
 *                           *      Copyright (C) 2002    *   
 *                           * Carnegie Mellon University *
 *                           *      All Rights Reserved   *
 *                           ******************************
 *
 * DISCLAIMER:  This software is made available for academic and research
 *              purposes only. No commercial license is hereby granted.
 *              Copying and other reproduction is authorized only for
 *              research, education, and other non-commercial purposes.  No
 *              warranties, either expressed or implied, are made regarding
 *              the operation, use, or results of the software. 
 *
 *****************************************************************************/

/*****************************************************************************
 * NOTE ON POSES AND TRANSFORMS

    Note that the default Euler Angle Order for a pose is ZYX, as defined in the constructors 
    for this class in recGeometry.h. They use ZYX when an Euler Angle Order isn't specified

    This means that the pose MUST be constructed as such when using the default:

        RecPose3D Pose( X, Y, Z, Yaw, Pitch, Roll);

    This is equivalent to:

        RecPose3D Pose( X, Y, Z, Yaw, Pitch, Roll, RecPose3D::ZYX );

    While this ordering of angles isn't what we would typically use (roll-pitch-yaw
    just rolls off the tongue better), it follows the SAE coordinate system.  This prescribes
    that when transforming points, we rotate about the z axis (yaw), then the new Y axis 
    (body pitch), then about the doubly new X axis (body roll).  X is considered forward, 
    Y is to the right and Z is down.

    A key advantage of this ordering is that the pose angles are simply the yaw, pitch,
    and roll relative to the original coordinate frame and no additional calcs are needed.

    Example:  A sensor faces 0.2 radians to the right, is tilted down 0.1 radians, and is
    tilted 0.4 radians CCW relative to the vehicle body (when viewed from inside the vehicle). 
    In addition, the sensor is 1 meter forward, 0.5 meters to the right and 0.25 
    meters above the vehicle origin. To represent this pose, use the following line:

        RecPose3D sensorPose( 1.0, 0.5, -0.25, 0.2, -0.1, -0.4, RecPose3D::ZYX);

    The Euler Angle Order is specified here just in case the defaults change one day.

    To develop a transform to convert the sensor points to vehicle coord points we use:

        RecTransform sensorToVehicle( sensorPose );

    Applying this to a point from the sensor:
       
        VehiclePoint = sensorToVehicle*SensorPoint;

    Returns that point in the Vehicle Coordinate Frame

 *****************************************************************************/

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <math.h>
#include <iomanip>
#include <iostream>
#include <assert.h>
#include <memory.h>
#include "recGeometry.h"

using namespace std; 
/*****************************************************************************
 * CONSTANTS
 *****************************************************************************/
const double RecTransform3D::tolerance = 1e-6;


/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: assign the transform to the identity transform
 *****************************************************************************/
RecTransform3D::RecTransform3D(void)
{
  identity();

}
/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: assemble a transform matrix from axis vectors 
 *****************************************************************************/
RecTransform3D::RecTransform3D(const RecVector3D& vecX, const RecVector3D& vecY,
                               const RecVector3D& vecZ)
{
  RecVector3D vX(vecX);
  RecVector3D vY(vecY);
  RecVector3D vZ(vecZ);

  vX.normalize();
  vY.normalize();
  vZ.normalize();
  
  matrixPP[0][0] = vX.x;
  matrixPP[1][0] = vX.y;
  matrixPP[2][0] = vX.z;
    
  matrixPP[0][1] = vY.x;
  matrixPP[1][1] = vY.y;
  matrixPP[2][1] = vY.z;
  
  matrixPP[0][2] = vZ.x;
  matrixPP[1][2] = vY.y;
  matrixPP[2][2] = vZ.z;
  
  matrixPP[0][3] = 0.0;
  matrixPP[1][3] = 0.0;
	matrixPP[2][3] = 0.0;
}

/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: assemble a transform matrix from axis vectors and origin vector 
 *****************************************************************************/
RecTransform3D::RecTransform3D(const RecVector3D& vecX, const RecVector3D& vecY,
                               const RecVector3D& vecZ, const RecVector3D& origin)
{
  RecVector3D vX(vecX);
  RecVector3D vY(vecY);
  RecVector3D vZ(vecZ);
  
  vX.normalize();
  vY.normalize();
  vZ.normalize();
  
  matrixPP[0][0] = vX.x;
  matrixPP[1][0] = vX.y;
  matrixPP[2][0] = vX.z;
  
  matrixPP[0][1] = vY.x;
  matrixPP[1][1] = vY.y;
  matrixPP[2][1] = vY.z;
  
  matrixPP[0][2] = vZ.x;
  matrixPP[1][2] = vY.y;
  matrixPP[2][2] = vZ.z;
  
  matrixPP[0][3] = origin.x;
  matrixPP[1][3] = origin.y;
  matrixPP[2][3] = origin.z;
}

/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: copy constructor 
 *****************************************************************************/
RecTransform3D::RecTransform3D(const RecTransform3D& trans3D)
{
  memcpy(matrixPP, trans3D.matrixPP, sizeof(matrixPP));
}

/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: constructor for converting a 2D transform into a 3D transform 
 *****************************************************************************/
RecTransform3D::RecTransform3D(const RecTransform2D& trans2D)
{
  matrixPP[0][0] = trans2D.matrixPP[0][0];
  matrixPP[0][1] = trans2D.matrixPP[0][1];
  matrixPP[0][2] = 0.0;
  matrixPP[0][3] = trans2D.matrixPP[0][2];

  matrixPP[1][0] = trans2D.matrixPP[1][0];
  matrixPP[1][1] = trans2D.matrixPP[1][1];
  matrixPP[1][2] = 0.0;
  matrixPP[1][3] = trans2D.matrixPP[1][2];

  matrixPP[2][0] = 0.0;
  matrixPP[2][1] = 0.0;
  matrixPP[2][2] = 1.0;
  matrixPP[2][3] = 0.0;
}

/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: constructor for converting a 3D pose into a 3D transform 
 *****************************************************************************/
RecTransform3D::RecTransform3D(const RecPose3D& pose)
{
  double cg,cb,ca;
  double sg,sb,sa;

  sa = sin(pose.rot1);
  sb = sin(pose.rot2);
  sg = sin(pose.rot3);

  ca = cos(pose.rot1);
  cb = cos(pose.rot2);
  cg = cos(pose.rot3);

  matrixPP[0][3] = pose.x;
  matrixPP[1][3] = pose.y;
  matrixPP[2][3] = pose.z;

	switch(pose.angleOrder) 
	{
  case  RecPose3D::XYX:  
		matrixPP[0][0] = cb;
		matrixPP[0][1] = sb*sg;
		matrixPP[0][2] = sb*cg;
		matrixPP[1][0] = sa*sb;
		matrixPP[1][1] = ca*cg - sa*cb*sg;
		matrixPP[1][2] = -ca*sg - sa*cb*cg;
		matrixPP[2][0] = -ca*sb;
		matrixPP[2][1] = sa*cg + ca*cb*sg;
		matrixPP[2][2] = -sa*sg + ca*cb*cg;
		break;
	case RecPose3D::XYZ:
		matrixPP[0][0] = cb*cg;
		matrixPP[0][1] = -cb*sg;
		matrixPP[0][2] = sb;
		matrixPP[1][0] = sa*sb*cg + ca*sg;
		matrixPP[1][1] = -sa*sb*sg + ca*cg;
		matrixPP[1][2] = -sa*cb;
		matrixPP[2][0] = -ca*sb*cg + sa*sg;
		matrixPP[2][1] = ca*sb*sg + sa*cg;
		matrixPP[2][2] = ca*cb;
		break;
	case RecPose3D::XZX:
		matrixPP[0][0] = cb;
		matrixPP[0][1] = -sb*cg;
		matrixPP[0][2] = sb*sg;
		matrixPP[1][0] = ca*sb;
		matrixPP[1][1] = -sa*sg + ca*cb*cg;
		matrixPP[1][2] = -ca*cb*sg - sa*cg;
		matrixPP[2][0] = sa*sb;
		matrixPP[2][1] = sa*cb*cg + ca*sg;
		matrixPP[2][2] = ca*cg - sa*cb*sg;
		break;
	case RecPose3D::XZY:
    matrixPP[0][0] = cb*cg;
		matrixPP[0][1] = -sb;
		matrixPP[0][2] = cb*sg;
		matrixPP[1][0] = sa*sg + ca*sb*cg;
		matrixPP[1][1] = ca*cb;
		matrixPP[1][2] = -sa*cg + ca*sb*sg;
		matrixPP[2][0] = -ca*sg + sa*sb*cg;
		matrixPP[2][1] = sa*cb;
		matrixPP[2][2] = ca*cg + sa*sb*sg;
		break;
	case RecPose3D::YXY:
		matrixPP[0][0] = ca*cg - sa*cb*sg;
		matrixPP[0][1] = sa*sb;
		matrixPP[0][2] = sa*cb*cg + ca*sg;
		matrixPP[1][0] = sb*sg;
		matrixPP[1][1] = cb;
		matrixPP[1][2] = -sb*cg;
		matrixPP[2][0] = -ca*cb*sg - sa*cg;
		matrixPP[2][1] = ca*sb;
		matrixPP[2][2] = -sa*sg + ca*cb*cg;
		break;
	case RecPose3D::YXZ:
    matrixPP[0][0] = ca*cg + sa*sb*sg;
		matrixPP[0][1] = -ca*sg + sa*sb*cg;
		matrixPP[0][2] = sa*cb;
		matrixPP[1][0] = cb*sg;
		matrixPP[1][1] = cb*cg;
		matrixPP[1][2] = -sb;
		matrixPP[2][0] = -sa*cg + ca*sb*sg;
		matrixPP[2][1] = sa*sg + ca*sb*cg;
		matrixPP[2][2] = ca*cb;
		break;
	case RecPose3D::YZX:
		matrixPP[0][0] = ca*cb;
		matrixPP[0][1] = -ca*sb*cg + sa*sg;
		matrixPP[0][2] = ca*sb*sg + sa*cg;
		matrixPP[1][0] = sb;
		matrixPP[1][1] = cb*cg;
		matrixPP[1][2] = -cb*sg;
		matrixPP[2][0] = -sa*cb;
		matrixPP[2][1] = sa*sb*cg + ca*sg;
		matrixPP[2][2] = -sa*sb*sg + ca*cg;
		break;
	case RecPose3D::YZY:
		matrixPP[0][0] = -sa*sg + ca*cb*cg;
		matrixPP[0][1] = -ca*sb;
		matrixPP[0][2] = sa*cg + ca*cb*sg;
		matrixPP[1][0] = sb*cg;
		matrixPP[1][1] = cb;
		matrixPP[1][2] = sb*sg;
		matrixPP[2][0] = -ca*sg - sa*cb*cg;
		matrixPP[2][1] = sa*sb;
		matrixPP[2][2] = ca*cg - sa*cb*sg;
		break;
	case RecPose3D::ZXY:
		matrixPP[0][0] = -sa*sb*sg + ca*cg;
		matrixPP[0][1] = -sa*cb;
		matrixPP[0][2] = sa*sb*cg + ca*sg;
		matrixPP[1][0] = ca*sb*sg + sa*cg;
		matrixPP[1][1] = ca*cb; 
		matrixPP[1][2] = -ca*sb*cg + sa*sg;
		matrixPP[2][0] = -cb*sg;
		matrixPP[2][1] = sb;
		matrixPP[2][2] = cb*cg;
		break;
	case RecPose3D::ZXZ:
		matrixPP[0][0] = ca*cg - sa*cb*sg;
		matrixPP[0][1] = -ca*sg - sa*cb*cg;
		matrixPP[0][2] = sa*sb;
		matrixPP[1][0] = sa*cg + ca*cb*sg;
		matrixPP[1][1] = -sa*sg + ca*cb*cg;
		matrixPP[1][2] = -ca*sb;
		matrixPP[2][0] = sb*sg;
		matrixPP[2][1] = sb*cg;
		matrixPP[2][2] = cb;
		break;
	case RecPose3D::ZYX:
		matrixPP[0][0] = ca*cb;
		matrixPP[0][1] = -sa*cg + ca*sb*sg;
		matrixPP[0][2] = sa*sg + ca*sb*cg;
		matrixPP[1][0] = sa*cb;
		matrixPP[1][1] = ca*cg + sa*sb*sg;
		matrixPP[1][2] = -ca*sg + sa*sb*cg;
		matrixPP[2][0] = -sb;
		matrixPP[2][1] = cb*sg;
		matrixPP[2][2] = cb*cg;
		break;
	case RecPose3D::ZYZ:
		matrixPP[0][0] = -sa*sg + ca*cb*cg;
		matrixPP[0][1] = -ca*cb*sg - sa*cg;
		matrixPP[0][2] = ca*sb;
		matrixPP[1][0] = sa*cb*cg + ca*sg;
		matrixPP[1][1] = ca*cg - sa*cb*sg;
		matrixPP[1][2] = sa*sb;
		matrixPP[2][0] = -sb*cg;
		matrixPP[2][1] = sb*sg;
		matrixPP[2][2] = cb;
		break;
	default:
		cout << "Error in transform constructor: invalid angle order for pose\n";
		break;
	}
}

/******************************************************************************
 *      METHOD: RecTransform3D
 * DESCRIPTION: constructor for converting a quaternion into a 3D transform 
 *****************************************************************************/
RecTransform3D::RecTransform3D(const RecQuaternion& q)
{
  matrixPP[0][0] = 1.0 - 2.0 * (q.quat[1] * q.quat[1] + q.quat[2] * q.quat[2]);
  matrixPP[0][1] = 2.0 * (q.quat[0] * q.quat[1] - q.quat[2] * q.quat[3]);
  matrixPP[0][2] = 2.0 * (q.quat[2] * q.quat[0] + q.quat[1] * q.quat[3]);
  matrixPP[0][3] = 0.0;
  
  matrixPP[1][0] = 2.0 * (q.quat[0] * q.quat[1] + q.quat[2] * q.quat[3]);
  matrixPP[1][1]=  1.0 - 2.0 * (q.quat[2] * q.quat[2] + q.quat[0] * q.quat[0]);
  matrixPP[1][2] = 2.0 * (q.quat[1] * q.quat[2] - q.quat[0] * q.quat[3]);
  matrixPP[1][3] = 0.0;
  
  matrixPP[2][0] = 2.0 * (q.quat[2] * q.quat[0] - q.quat[1] * q.quat[3]);
  matrixPP[2][1] = 2.0 * (q.quat[1] * q.quat[2] + q.quat[0] * q.quat[3]);
  matrixPP[2][2] = 1.0 - 2.0 * (q.quat[1] * q.quat[1] + q.quat[0] * q.quat[0]);
  matrixPP[2][3] = 0.0;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this transform with the passed pose
 *****************************************************************************/
void RecTransform3D::operator*=(const RecPose3D& b) 
{
  RecTransform3D transb(b);
  (*this) = (*this) * transb;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this transform with the passed transform
 *****************************************************************************/
void RecTransform3D::operator*=(const RecTransform3D& b) 
{
  (*this) = (*this) * b;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this transform with the passed quaternion
 *****************************************************************************/
void RecTransform3D::operator*=(const RecQuaternion& b) 
{
  RecTransform3D transb(b);
  (*this) = (*this) * transb;
}


/******************************************************************************
 *      METHOD: inverse
 * DESCRIPTION: computes and returns the inverse of the transform 
 *****************************************************************************/
RecTransform3D RecTransform3D::inverse(void)
{
  RecTransform3D inv;
  inv.matrixPP[0][0] = matrixPP[0][0];
  inv.matrixPP[0][1] = matrixPP[1][0];
  inv.matrixPP[0][2] = matrixPP[2][0];
  inv.matrixPP[0][3] = (-matrixPP[0][0]*matrixPP[0][3] -
			matrixPP[1][0]*matrixPP[1][3] - 
			matrixPP[2][0]*matrixPP[2][3]);

  inv.matrixPP[1][0] = matrixPP[0][1];
  inv.matrixPP[1][1] = matrixPP[1][1];
  inv.matrixPP[1][2] = matrixPP[2][1];
  inv.matrixPP[1][3] = (-matrixPP[0][1]*matrixPP[0][3] -
			matrixPP[1][1]*matrixPP[1][3] -
			matrixPP[2][1]*matrixPP[2][3]);

  inv.matrixPP[2][0] = matrixPP[0][2];
  inv.matrixPP[2][1] = matrixPP[1][2];
  inv.matrixPP[2][2] = matrixPP[2][2];
  inv.matrixPP[2][3] = (-matrixPP[0][2]*matrixPP[0][3] -
			matrixPP[1][2]*matrixPP[1][3] -
			matrixPP[2][2]*matrixPP[2][3]);

  return inv;
}

/******************************************************************************
 *      METHOD: identity
 * DESCRIPTION: sets the rotation matrix to the identity matrix and the 
 *              translation vector to zero
 *****************************************************************************/
void RecTransform3D::identity(void)
{
	matrixPP[0][0] = 1.0;
	matrixPP[0][1] = 0.0;
	matrixPP[0][2] = 0.0;
	matrixPP[0][3] = 0.0;

	matrixPP[1][0] = 0.0;
	matrixPP[1][1] = 1.0;
	matrixPP[1][2] = 0.0;
	matrixPP[1][3] = 0.0;

	matrixPP[2][0] = 0.0;
	matrixPP[2][1] = 0.0;
	matrixPP[2][2] = 1.0;
	matrixPP[2][3] = 0.0;

}

/******************************************************************************
 *      METHOD: xAxis
 * DESCRIPTION: returns a vector of unit lenght that is the same direction
 *              as the transformed frame's x axis
 *****************************************************************************/
RecVector3D RecTransform3D::xAxis(void) const
{
	RecVector3D x(matrixPP[0][0], matrixPP[1][0], matrixPP[2][0]);
	return x;
}

/******************************************************************************
 *      METHOD: yAxis
 * DESCRIPTION: returns a vector of unit lenght that is the same direction
 *              as the transformed frame's y axis
 *****************************************************************************/
RecVector3D RecTransform3D::yAxis(void) const
{
	RecVector3D y(matrixPP[0][1], matrixPP[1][1], matrixPP[2][1]);
	return y;
}

/******************************************************************************
 *      METHOD: zAxis
 * DESCRIPTION: returns a vector of unit lenght that is the same direction
 *              as the transformed frame's z axis
 *****************************************************************************/
RecVector3D RecTransform3D::zAxis(void) const
{
	RecVector3D z(matrixPP[0][2], matrixPP[1][2], matrixPP[2][2]);
	return z;
}

/******************************************************************************
 *      METHOD: origin
 * DESCRIPTION: returns the vector that points to the origin of the transformed
 *              frame
 *****************************************************************************/
RecVector3D RecTransform3D::origin(void) const
{
	RecVector3D o(matrixPP[0][3], matrixPP[1][3], matrixPP[2][3]);
	return o;
}

/******************************************************************************
 *      METHOD: rotateAboutXAxis
 * DESCRIPTION: causes the transformed frame to be rotated about it's x axis
 *****************************************************************************/
void RecTransform3D::rotateAboutXAxis(RecRadians radAngle)
{
	double s = sin(double(radAngle));
	double c = cos(double(radAngle));
	RecTransform3D tempTrans(*this);
	
	// modify y axis
	matrixPP[0][1] = tempTrans[0][1] * c + tempTrans[0][2] * s;
	matrixPP[1][1] = tempTrans[1][1] * c + tempTrans[1][2] * s;
	matrixPP[2][1] = tempTrans[2][1] * c + tempTrans[2][2] * s;
	
	// modify z axis
	matrixPP[0][2] = -tempTrans[0][1] * s + tempTrans[0][2] * c;
	matrixPP[1][2] = -tempTrans[1][1] * s + tempTrans[1][2] * c;
	matrixPP[2][2] = -tempTrans[2][1] * s + tempTrans[2][2] * c;

	// do not modify x axis
}

/******************************************************************************
 *      METHOD: rotateAboutYAxis
 * DESCRIPTION: causes the transformed frame to be rotated about it's y axis
 *****************************************************************************/
void RecTransform3D::rotateAboutYAxis(RecRadians radAngle)
{
	double s = sin(double(radAngle));
	double c = cos(double(radAngle));
	RecTransform3D tempTrans(*this);

	// modify x axis
	matrixPP[0][0] = -tempTrans[0][2] * s + tempTrans[0][0] * c;
	matrixPP[1][0] = -tempTrans[1][2] * s + tempTrans[1][0] * c;
	matrixPP[2][0] = -tempTrans[2][2] * s + tempTrans[2][0] * c;

	// modify z axis
	matrixPP[0][2] = tempTrans[0][2] * c + tempTrans[0][0] * s;
	matrixPP[1][2] = tempTrans[1][2] * c + tempTrans[1][0] * s;
	matrixPP[2][2] = tempTrans[2][2] * c + tempTrans[2][0] * s;

	// do not modify y axis
}

/******************************************************************************
 *      METHOD: rotateAboutZAxis
 * DESCRIPTION: causes the transformed frame to be rotated about it's z axis
 *****************************************************************************/
void RecTransform3D::rotateAboutZAxis(RecRadians radAngle)
{
	double s = sin(double(radAngle));
	double c = cos(double(radAngle));
	RecTransform3D tempTrans(*this);
	
	// modify the x axis
	matrixPP[0][0] = tempTrans[0][0] * c + tempTrans[0][1] * s;
	matrixPP[1][0] = tempTrans[1][0] * c + tempTrans[1][1] * s;
	matrixPP[2][0] = tempTrans[2][0] * c + tempTrans[2][1] * s;

	// modify the y axis
	matrixPP[0][1] = -tempTrans[0][0] * s + tempTrans[0][1] * c;
	matrixPP[1][1] = -tempTrans[1][0] * s + tempTrans[1][1] * c;
	matrixPP[2][1] = -tempTrans[2][0] * s + tempTrans[2][1] * c;

	// do not change the z axis
}

/******************************************************************************
 *      METHOD: translate
 * DESCRIPTION: causes the transformed frame to translate by the passed vector
 *****************************************************************************/
void RecTransform3D::translate(const RecVector3D& transVect)
{
	matrixPP[0][3] += transVect.x;
	matrixPP[1][3] += transVect.y;
	matrixPP[2][3] += transVect.z;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: multiplies two transforms together and returns the result in
 *              a new transform (thereby concatenating the transforms)
 *****************************************************************************/
RecTransform3D operator*(const RecTransform3D& b, const RecTransform3D& c)
{
  RecTransform3D a;
  a.matrixPP[0][0] = b.matrixPP[0][0]*c.matrixPP[0][0] +
    b.matrixPP[0][1]*c.matrixPP[1][0] + b.matrixPP[0][2]*c.matrixPP[2][0];
  a.matrixPP[0][1] = b.matrixPP[0][0]*c.matrixPP[0][1] +
    b.matrixPP[0][1]*c.matrixPP[1][1] + b.matrixPP[0][2]*c.matrixPP[2][1];
  a.matrixPP[0][2] = b.matrixPP[0][0]*c.matrixPP[0][2] +
    b.matrixPP[0][1]*c.matrixPP[1][2] + b.matrixPP[0][2]*c.matrixPP[2][2];
  a.matrixPP[0][3] = b.matrixPP[0][0]*c.matrixPP[0][3] +
    b.matrixPP[0][1]*c.matrixPP[1][3] + b.matrixPP[0][2]*c.matrixPP[2][3] +
    b.matrixPP[0][3];
  
  a.matrixPP[1][0] = b.matrixPP[1][0]*c.matrixPP[0][0] +
    b.matrixPP[1][1]*c.matrixPP[1][0] + b.matrixPP[1][2]*c.matrixPP[2][0];
  a.matrixPP[1][1] = b.matrixPP[1][0]*c.matrixPP[0][1] +
    b.matrixPP[1][1]*c.matrixPP[1][1] + b.matrixPP[1][2]*c.matrixPP[2][1];
  a.matrixPP[1][2] = b.matrixPP[1][0]*c.matrixPP[0][2] +
    b.matrixPP[1][1]*c.matrixPP[1][2] + b.matrixPP[1][2]*c.matrixPP[2][2];
  a.matrixPP[1][3] = b.matrixPP[1][0]*c.matrixPP[0][3] +
    b.matrixPP[1][1]*c.matrixPP[1][3] + b.matrixPP[1][2]*c.matrixPP[2][3] +
    b.matrixPP[1][3];
  
  a.matrixPP[2][0] = b.matrixPP[2][0]*c.matrixPP[0][0] +
    b.matrixPP[2][1]*c.matrixPP[1][0] + b.matrixPP[2][2]*c.matrixPP[2][0];
  a.matrixPP[2][1] = b.matrixPP[2][0]*c.matrixPP[0][1] +
    b.matrixPP[2][1]*c.matrixPP[1][1] + b.matrixPP[2][2]*c.matrixPP[2][1];
  a.matrixPP[2][2] = b.matrixPP[2][0]*c.matrixPP[0][2] +
    b.matrixPP[2][1]*c.matrixPP[1][2] + b.matrixPP[2][2]*c.matrixPP[2][2];
  a.matrixPP[2][3] = b.matrixPP[2][0]*c.matrixPP[0][3] +
    b.matrixPP[2][1]*c.matrixPP[1][3] + b.matrixPP[2][2]*c.matrixPP[2][3] +
    b.matrixPP[2][3]; 
  return a;
}


/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: Transforms a 3D point by a transform
 *****************************************************************************/
RecPoint3D operator*(const RecTransform3D& tr, const RecPoint3D& pt)
{
  RecPoint3D newpt;
  newpt.x = tr.matrixPP[0][0]*pt.x + tr.matrixPP[0][1]*pt.y +
    tr.matrixPP[0][2]*pt.z + tr.matrixPP[0][3];
  newpt.y = tr.matrixPP[1][0]*pt.x + tr.matrixPP[1][1]*pt.y +
    tr.matrixPP[1][2]*pt.z + tr.matrixPP[1][3];
  newpt.z = tr.matrixPP[2][0]*pt.x + tr.matrixPP[2][1]*pt.y +
    tr.matrixPP[2][2]*pt.z + tr.matrixPP[2][3];
  return newpt;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: Transforms a 2D point by a 3D transform 
 *              (Z coordinate is ignored)
 *****************************************************************************/
RecPoint2D operator*(const RecTransform3D& tr, const RecPoint2D& pt)
{
  RecPoint2D newpt;
  newpt.x = tr.matrixPP[0][0]*pt.x + tr.matrixPP[0][1]*pt.y + tr.matrixPP[0][3];
  newpt.y = tr.matrixPP[1][0]*pt.x + tr.matrixPP[1][1]*pt.y + tr.matrixPP[1][3];
  return newpt;
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the transform with nice formatting to the passed 
 *              output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecTransform3D& trans3D)
{
  const int fw = 10; // output field width
  out << '[' << ' ' << setw(fw) << trans3D.matrixPP[0][0] << ' '
      << setw(fw) << trans3D.matrixPP[0][1] << ' '
      << setw(fw) << trans3D.matrixPP[0][2] << ' '
      << setw(fw) << trans3D.matrixPP[0][3] << ' '
      << ']' << endl;
  out << '[' << ' ' << setw(fw) << trans3D.matrixPP[1][0] << ' '
      << setw(fw) << trans3D.matrixPP[1][1] << ' '
      << setw(fw) << trans3D.matrixPP[1][2] << ' '
      << setw(fw) << trans3D.matrixPP[1][3] << ' '
      << ']' << endl;
  out << '[' << ' ' << setw(fw) << trans3D.matrixPP[2][0] << ' '
      << setw(fw) << trans3D.matrixPP[2][1] << ' '
      << setw(fw) << trans3D.matrixPP[2][2] << ' '
      << setw(fw) << trans3D.matrixPP[2][3] << ' '
      << ']' << endl;
  out << '[' << ' ' << setw(fw) << 0.0 << ' '
      << setw(fw) << 0.0 << ' '
      << setw(fw) << 0.0 << ' '
      << setw(fw) << 1.0 << ' '
      << ']' << endl;
  return out;
}

/******************************************************************************
 *      METHOD: ==operator
 * DESCRIPTION: compares two transforms and returns true if all values of the
 *              first transform matrix are within the tolerance of the second
 *              transform matrix values 
 *****************************************************************************/
bool operator==(const RecTransform3D& a, const RecTransform3D& b)
{
	return(
		(fabs(a.matrixPP[0][0] - b.matrixPP[0][0]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[0][1] - b.matrixPP[0][1]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[0][2] - b.matrixPP[0][2]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[0][3] - b.matrixPP[0][3]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[1][0] - b.matrixPP[1][0]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[1][1] - b.matrixPP[1][1]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[1][2] - b.matrixPP[1][2]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[1][3] - b.matrixPP[1][3]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[2][0] - b.matrixPP[2][0]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[2][1] - b.matrixPP[2][1]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[2][2] - b.matrixPP[2][2]) < RecTransform3D::tolerance) &&
		(fabs(a.matrixPP[2][3] - b.matrixPP[2][3]) < RecTransform3D::tolerance));
}

/******************************************************************************
 *      METHOD: !=operator
 * DESCRIPTION: compares two transforms and returns false if all values of the
 *              first transform matrix are within the tolerance of the second
 *              transform matrix values 
 *****************************************************************************/
bool operator!=(const RecTransform3D& a, const RecTransform3D& b)
{
	return !(a == b);
}









































