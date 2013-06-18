/*****************************************************************************
 * $Revision: 1.1 $
 *     $Date: 2005/02/07 17:59:21 $
 *   $Author: ekratzer $
 *                         ******************************
 *                         *      Copyright (C) 2002    *   
 *                         * Carnegie Mellon University *
 *                         *      All Rights Reserved   *
 *                         ******************************
 *
 *     PROJECT:	Blitz
 *        FILE:	recVector.cpp
 * DESCRIPTION: contains implementation of the RecVector* classes
 *     CREATED: 2003/02/07
 *    
 * HISTORY:
 *
 * $Log: recVector3D.cpp,v $
 * Revision 1.1  2005/02/07 17:59:21  ekratzer
 * Adding "recGeometry" to repository
 *
 * Revision 1.5  2006/10/05 15:31:34  taylor
 * Adding divide by zero protection
 *
 * Revision 1.1  2004/06/14 17:09:34  colmstea
 * Adding recGeometry.
 *
 * Revision 1.3  2003/02/18 17:13:35  bode
 * Added access featurs to recRadians and recVector classes.  Added comments
 * to recPose classes.  Fixed serializable vector3D bug.
 *
 * Revision 1.2  2003/02/14 22:39:48  bode
 * Added additional comments
 *
 * Revision 1.1  2003/02/14 20:30:10  bode
 * Limited Initial Release.  Only classes RecRadians, RecPoint3D,
 * RecPoint3D, RecVector2D, RecVector3D, RecTransform3D and RecPose3D have
 * been well tested.
 *
 *
 *****************************************************************************/
/*****************************************************************************
 * SYSTEM AND EXTERNAL INCLUDES
 *****************************************************************************/
#include <math.h>
/*****************************************************************************
 * PROJECT INCLUDES
 *****************************************************************************/

/*****************************************************************************
 * MODULE INCLUDES
 *****************************************************************************/
#include "recGeometry.h"

/*****************************************************************************
 * RecVector3D Functions and Methods
 *****************************************************************************/
/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecVector3D
 * DESCRIPTION: no argument constructor, sets x,y and z to zero 
 *****************************************************************************/
RecVector3D::RecVector3D(void):RecPoint3D() { }

/******************************************************************************
 *      METHOD: RecVector3D
 * DESCRIPTION: three argument constructor, sets x, y and z to passed 
 *              x, y and z values 
 *****************************************************************************/
RecVector3D::RecVector3D(double inx, double iny, 
												 double inz):RecPoint3D(inx, iny, inz) { }

/******************************************************************************
 *      METHOD: RecVector3D
 * DESCRIPTION: 2D point argument constructor, sets x, y to the x and y of the
 *              passed 2D vector.  Sets z to zero.
 *****************************************************************************/
RecVector3D::RecVector3D(const RecVector2D& v2D):RecPoint3D() {x = v2D.x; y= v2D.y; z =0; }
RecVector3D::RecVector3D(const RecPoint2D& p2D):RecPoint3D(p2D) { }

/******************************************************************************
 *      METHOD: RecVector3D
 * DESCRIPTION: copy constructor, copys the fields of the passed vector 
 *****************************************************************************/
RecVector3D::RecVector3D(const RecVector3D& v3D):RecPoint3D(v3D) { }
RecVector3D::RecVector3D(const RecPoint3D& p3D):RecPoint3D(p3D) { }


/******************************************************************************
 *      METHOD: =operator
 * DESCRIPTION: sets the fields of the vector to those of the passed vector.
 *****************************************************************************/
void RecVector3D::operator=(const RecVector3D& b)
{
	x = b.x;
	y = b.y; 
  z = b.z; 
}


/******************************************************************************
 *      METHOD: +operator
 * DESCRIPTION: adds the fields of the passed vector and returns the result
 *              in a new vector structure.
 *****************************************************************************/
RecVector3D operator+(const RecVector3D& a, const RecVector3D& b)
{
  RecVector3D c(a);
  c += b;
  return c;
}

/******************************************************************************
 *      METHOD: -operator
 * DESCRIPTION: subtracts the fields of the second vector from the fields of
 *              the first vector and returns the result in a new vector.
 *****************************************************************************/
RecVector3D operator-(const RecVector3D& a, const RecVector3D& b)
{
  RecVector3D c(a);
	c -= b;
  return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed vector by the passed double
 *              and returns the result in a new vector structure.
 *****************************************************************************/
RecVector3D operator*(const RecVector3D& a, double b)
{
  RecVector3D c(a);
	c *= b;
  return c;
}

/******************************************************************************
 *      METHOD: *operator
 * DESCRIPTION: multiplies the fields of the passed vector by the passed double
 *              and returns the result in a new vector structure.
 *****************************************************************************/
RecVector3D operator*(double b, const RecVector3D& a)
{
  return a * b;
}

/******************************************************************************
 *      METHOD: /operator
 * DESCRIPTION: divides the fields of the passed vector by the passed double 
 *              and returns the result in a new point structure.  Returns a
 *              copy of the passed vector if the passed double is zero.
 *****************************************************************************/
RecVector3D operator/(const RecVector3D& a, double b)
{
  RecVector3D c(a);
  if (b != 0)		//MAT 10-5-06 - added divide by zero protection
  {
	c /= b;
  }
  return c;
}
/******************************************************************************
 *      METHOD: operator^=
 * DESCRIPTION: returns the cross product of the two passed vectors 
 *****************************************************************************/
RecVector3D operator ^(const RecVector3D& a, const RecVector3D& b)
{
	RecVector3D v;

  v.x = a.y * b.z - a.z * b.y;
	v.y = a.z * b.x - a.x * b.z;
	v.z = a.x * b.y - a.y * b.x;

	return v;
}

/******************************************************************************
 *      METHOD: operator^=
 * DESCRIPTION: makes the vector the cross product of itself and the 
 *              passed vector 
 *****************************************************************************/
void RecVector3D::operator ^=(const RecVector3D& a)
{
	(*this) = (*this) ^ a;
}

/******************************************************************************
 *      METHOD: operator^=
 * DESCRIPTION: returns the cross product of this vector and the passed vector
 *****************************************************************************/
RecVector3D RecVector3D::cross(const RecVector3D& a) const
{
	return((*this) ^ a);
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: computes and returns the dot product of the two passed vectors 
 *****************************************************************************/
double operator *(const RecVector3D& a, const RecVector3D& b) 
{
	return(a.x * b.x + a.y * b.y + a.z * b.z);
}

/******************************************************************************
 *      METHOD: dot
 * DESCRIPTION: computes and returns the dot product of this vector with
 *              the passed vector 
 *****************************************************************************/
double RecVector3D::dot(const RecVector3D& a) const
{
	return((*this)*a);
}

/******************************************************************************
 *      METHOD: lengthSq
 * DESCRIPTION: computes and returns the square of the length of the vector 
 *****************************************************************************/
double RecVector3D::lengthSq(void) const
{
	return(x*x + y*y + z*z);
}

/******************************************************************************
 *      METHOD: length
 * DESCRIPTION: computes and returns the length of the vector 
 *****************************************************************************/
double RecVector3D::length(void) const
{
	return(sqrt(lengthSq()));
}

/******************************************************************************
 *      METHOD: normalize
 * DESCRIPTION: makes the vector a unit vector that has the same direction
 *              as the old vector
 *****************************************************************************/
void RecVector3D::normalize(void)
{
    double newlength = length();   //MAT 10-5-06 - added divide by zero protection
    if (newlength != 0)
    {
        (*this) /= newlength;
    }
}
