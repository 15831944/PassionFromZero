/*****************************************************************************
 * $Revision: 1.1 $
 *     $Date: 2005/02/07 17:59:21 $
 *   $Author: ekratzer $
 *                         ******************************
 *                         *      Copyright (C) 2003    *   
 *                         * Carnegie Mellon University *
 *                         *      All Rights Reserved   *
 *                         ******************************
 *
 *     PROJECT:	Blitz
 *        FILE:	recQuaternion.cpp
 * DESCRIPTION: 
 *     CREATED: 2003/03/03
 *    
 * HISTORY:
 *
 * $Log: recQuaternion.cpp,v $
 * Revision 1.1  2005/02/07 17:59:21  ekratzer
 * Adding "recGeometry" to repository
 *
 * Revision 1.1  2004/06/14 17:09:34  colmstea
 * Adding recGeometry.
 *
 * Revision 1.2  2003/03/27 16:43:30  bode
 * Fixed compilation warnings
 *
 * Revision 1.1  2003/03/04 21:03:57  bode
 * Added quaternion class and extended support for the pose and transform
 * classes
 *
 *
 *****************************************************************************/

/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <math.h>
#include <iomanip>
#include <iostream>
#include "recGeometry.h"

using namespace std; 


/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecQuaternion
 * DESCRIPTION: copy constructor 
 *****************************************************************************/
RecQuaternion::RecQuaternion(void)
{
  quat[0] = 0.0;
  quat[1] = 0.0;
  quat[2] = 0.0;
  quat[3] = 1.0;
}

/******************************************************************************
 *      METHOD: RecQuaternion
 * DESCRIPTION: copy constructor 
 *****************************************************************************/
RecQuaternion::RecQuaternion(const RecQuaternion& q)
{
  (*this) = q;
}

/******************************************************************************
 *      METHOD: RecQuaternion
 * DESCRIPTION: Constructor for converting a 3D Transform to a Quaternion
 *****************************************************************************/
RecQuaternion::RecQuaternion(const RecTransform3D& trans)
{
  double t,s;
  double maxValue;
  int i,maxIndex;
  
  this->normalize();  

  t = trans.matrixPP[0][0] + trans.matrixPP[1][1] + trans.matrixPP[2][2] + 1;
  
  if(t > 0)
  {  
    s = 0.5/sqrt(t);
    quat[0] = (trans.matrixPP[2][1]-trans.matrixPP[1][2]) * s;
    quat[1] = (trans.matrixPP[0][2]-trans.matrixPP[2][0]) * s;
    quat[2] = (trans.matrixPP[1][0]-trans.matrixPP[0][1]) * s;
    quat[3] = 0.25 / s;  
  }
  else
  {
    maxValue = trans.matrixPP[0][0];
    maxIndex = 0;
    for(i = 1; i < 3; i++)	//MAT-9/28/06 changed to i < 3 since there are only 3 cases in the switch statement...
    {
      if(trans.matrixPP[i][i] > maxValue)
      {
        maxValue = trans.matrixPP[i][i];
        maxIndex = i;
      }
    }
    
    switch(maxIndex)
    {
    case 0:
      s = sqrt(1.0 + trans.matrixPP[0][0] - trans.matrixPP[1][1] - 
        trans.matrixPP[2][2]) * 2;
      quat[0] = 0.5 / s;
      quat[1] = (trans.matrixPP[0][1] + trans.matrixPP[1][0]) / s;
      quat[2] = (trans.matrixPP[0][2] + trans.matrixPP[2][0]) / s;
      quat[3] = (-trans.matrixPP[1][2] + trans.matrixPP[2][1]) / s;  //MAT- if their's a minus, I added it 9/28/06
      break;
    case 1:
      s = sqrt(1.0 + trans.matrixPP[1][1] - trans.matrixPP[0][0] - 
        trans.matrixPP[2][2]) * 2;
      quat[0] = (trans.matrixPP[0][1] + trans.matrixPP[1][0]) / s;
      quat[1] = 0.5 / s;
      quat[2] = (trans.matrixPP[1][2] + trans.matrixPP[2][1]) / s;
      quat[3] = (trans.matrixPP[0][2] - trans.matrixPP[2][0]) / s;  //MAT- if their's a minus, I added it 9/28/06
      break;
    case 2:
      s = sqrt(1.0 + trans.matrixPP[2][2] - trans.matrixPP[0][0] - 
        trans.matrixPP[1][1]) * 2;
      quat[0] = (trans.matrixPP[0][2] + trans.matrixPP[2][0]) / s;
      quat[1] = (trans.matrixPP[1][2] + trans.matrixPP[2][1]) / s;
      quat[2] = 0.5 / s;
      quat[3] = (-trans.matrixPP[0][1] + trans.matrixPP[1][0]) / s;  //MAT- if their's a minus, I added it 9/28/06
      break;
    } 
  }
  this->normalize();
}

/******************************************************************************
 *      METHOD: RecQuaternion
 * DESCRIPTION: Constructor for converting a 3D Pose to a Quaternion
 *****************************************************************************/
RecQuaternion::RecQuaternion(const RecPose3D& pose)
{
  RecTransform3D t(pose);
  RecQuaternion q(t);
  (*this) = q;
}

/******************************************************************************
 *      METHOD: operator=
 * DESCRIPTION: assigns one quaternion to another
 *****************************************************************************/
void RecQuaternion::operator=(const RecQuaternion& q)
{
  quat[0] = q.quat[0];
  quat[1] = q.quat[1];
  quat[2] = q.quat[2];
  quat[3] = q.quat[3];
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this quaternion with the passed pose
 *****************************************************************************/
void RecQuaternion::operator*=(const RecPose3D& b) 
{
  RecTransform3D transThis(*this);
  RecTransform3D transb(b);
  RecTransform3D transa(transThis*transb);
  RecQuaternion a(transa);
  (*this) = a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this quaternion with the passed transform
 *****************************************************************************/
void RecQuaternion::operator*=(const RecTransform3D& b) 
{
  RecTransform3D transThis(*this);
  RecTransform3D transa(transThis*b);
  RecQuaternion a(transa);
  (*this) = a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: concatinates this quaternion with the passed quaternion
 *****************************************************************************/
void RecQuaternion::operator*=(const RecQuaternion& b) 
{
  RecTransform3D transThis(*this);
  RecTransform3D transb(b);
  RecTransform3D transa(transThis*transb);
  RecQuaternion a(transa);
  (*this) = a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: returns the quaternion that is the concatenation of the 
 *              two passed quaternions
 *****************************************************************************/
RecQuaternion operator*(const RecQuaternion& b, const RecQuaternion& c)
{
  RecTransform3D transb(b);
  RecTransform3D transc(c);
  RecTransform3D transa(transb*transc);
  RecQuaternion a(transa);
  return a;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: transforms the passed point by the passed quaternion and 
 *              returns the transformed point
 *****************************************************************************/
RecPoint3D operator*(const RecQuaternion& q, const RecPoint3D& pt)
{
  RecTransform3D t(q);
  return(t*pt);
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the quaternion with nice formatting to the passed 
 *              output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecQuaternion& a)
{
  return out << "< " << a.quat[0] << ", " << a.quat[1] << ", " << a.quat[2]
    << ", " << a.quat[3] << " >";
}



/******************************************************************************
 *      METHOD: getAxisAndAngle
 * DESCRIPTION: converts the quaternion to a vector and returns the vector and
 *              angle phi
 *****************************************************************************/
void RecQuaternion::getAxisAndAngle(RecVector3D* v, RecRadians* phi) const
{
  v->x = quat[0];
  v->y = quat[1];
  v->z = quat[2];

  (*v) *= 1/sin(acos(quat[3]));
  v->normalize();
	*phi = acos(quat[3]) * 2;
}

/******************************************************************************
 *      METHOD: setAxisAndAngle
 * DESCRIPTION: sets the quaternion to a vector and angle phi
 *****************************************************************************/
void RecQuaternion::setAxisAndAngle(const RecVector3D& vec, const RecRadians& phi)
{
  RecVector3D v = vec;
  v.normalize();

  v *= sin(phi/2.0);

  quat[0] = v.x;
  quat[1] = v.y;
  quat[2] = v.z;
  quat[3] = cos(phi/2.0);

}

/******************************************************************************
 *      METHOD: normalize
 * DESCRIPTION: makes this quaternion a unit quaternion
 *****************************************************************************/
void RecQuaternion::normalize()
{
  double m = magnitude();   //Spelling correction:  was 'magintude'    MAT 9/19/06
  quat[0] /= m;
  quat[1] /= m;
  quat[2] /= m;
  quat[3] /= m;

}

/******************************************************************************
 *      METHOD: magnitude
 * DESCRIPTION: returns the magnitude of this quaternion
 *****************************************************************************/
double RecQuaternion::magnitude() const	  //Spelling correction:  was 'magintude'    MAT 9/19/06
{
  return sqrt( (quat[0]*quat[0]) + (quat[1]*quat[1]) +   // MAT 9/28/06- Added SQRT
    (quat[2]*quat[2]) + (quat[3]*quat[3]) );  

}



