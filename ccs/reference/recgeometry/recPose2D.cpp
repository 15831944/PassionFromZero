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
 *        FILE:	recPose.cpp
 * DESCRIPTION: Contains implementation of recPose* classes
 *     CREATED: 2003/02/07
 *    
 * HISTORY:
 *
 * $Log: recPose2D.cpp,v $
 * Revision 1.1  2005/02/07 17:59:21  ekratzer
 * Adding "recGeometry" to repository
 *
 * Revision 1.1  2004/06/14 17:09:34  colmstea
 * Adding recGeometry.
 *
 * Revision 1.4  2003/03/11 19:33:14  bode
 * Updated constructors
 *
 * Revision 1.3  2003/02/18 17:13:34  bode
 * Added access featurs to recRadians and recVector classes.  Added comments
 * to recPose classes.  Fixed serializable vector3D bug.
 *
 * Revision 1.2  2003/02/14 21:12:44  randyw
 * converted to the new iostreams   <iostream> instead of <iostream.h> etc..
 *
 * Revision 1.1  2003/02/14 20:30:08  bode
 * Limited Initial Release.  Only classes RecRadians, RecPoint3D,
 * RecPoint3D, RecVector2D, RecVector3D, RecTransform3D and RecPose3D have
 * been well tested.
 *
 *
 *****************************************************************************/
/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <iostream>
#include <iomanip>
#include <math.h>
#include <assert.h>
#include <memory.h>
#include <stdio.h>

#include "recGeometry.h"

using namespace std;

/*****************************************************************************
 * RecPose2D
 *****************************************************************************/
/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecPose2D
 * DESCRIPTION: no argument constructor, sets x, y and rotZ to zero 
 *****************************************************************************/
RecPose2D::RecPose2D(void):x(0.0), y(0.0), rotZ(0.0) { }

/******************************************************************************
 *      METHOD: RecPose2D
 * DESCRIPTION: three argument constructor, sets x, y and rotZ to passed values 
 *****************************************************************************/
RecPose2D::RecPose2D(double ix, double iy,
                     RecRadians rZ):x(ix), y(iy), rotZ(rZ) { } 

/******************************************************************************
 *      METHOD: RecPose2D
 * DESCRIPTION: copy constructor, copys the fields of the passed pose
 *****************************************************************************/
RecPose2D::RecPose2D(const RecPose2D& b):x(b.x), y(b.y), rotZ(b.rotZ) { }

/******************************************************************************
 *      METHOD: RecPose2D
 * DESCRIPTION: transform constructor, converts a 2D Transform to a 2D pose  
 *****************************************************************************/
RecPose2D::RecPose2D(const RecTransform2D 
                     &trans):x(trans.matrixPP[0][2]), y(trans.matrixPP[1][2]),
                     rotZ(atan2(trans.matrixPP[1][0], trans.matrixPP[1][1])) { }

/******************************************************************************
 *      METHOD: inverse
 * DESCRIPTION: computes and returns the inverse of a pose by converting the
 *              pose to a transform, taking the inverse of this transform and
 *              converting the inverted transform into a 2D pose.
 *****************************************************************************/
RecPose2D RecPose2D::inverse(void) const
{
  RecTransform2D trans(*this);
  RecPose2D poseInv(trans.inverse());
  return poseInv;
}


RecPoint2D RecPose2D::getPoint( ) const
{
  return RecPoint2D(x,y);
}


/******************************************************************************
 *      METHOD: operator+=
 * DESCRIPTION: adds the passed differential pose to the pose  
 *****************************************************************************/
void RecPose2D::operator+=(const RecDifferentialPose2D& b) 
{
	x += b.x; 
	y += b.y; 
	rotZ += b.rotZ;
}

/******************************************************************************
 *      METHOD: operator-=
 * DESCRIPTION: subtracts the passed differential pose from the pose  
 *****************************************************************************/
void RecPose2D::operator-=(const RecDifferentialPose2D& b)
{
	x -= b.x;
	y -= b.y; 
	rotZ -= b.rotZ;
}

/******************************************************************************
 *      METHOD: operator=
 * DESCRIPTION: assigns one pose to another by copying all fields
 *****************************************************************************/
void RecPose2D::operator=(const RecPose2D& b) 
{
	x = b.x; 
	y = b.y; 
	rotZ = b.rotZ;
}

/******************************************************************************
 *      METHOD: operator*
 * DESCRIPTION: returns the pose that is the concatenation of the two passed
 *              poses
 *****************************************************************************/
RecPose2D operator*(const RecPose2D& b, const RecPose2D& c)
{
  RecTransform2D btrans(b);
  RecTransform2D ctrans(c);
  RecTransform2D atrans = btrans * ctrans;
  RecPose2D a(atrans);
  return a;
}

/******************************************************************************
 *      METHOD: operator+
 * DESCRIPTION: adds the passed differential pose to the passed pose and 
 *              returns the result  
 *****************************************************************************/
RecPose2D operator+(const RecPose2D& a, const RecDifferentialPose2D& b)
{
  RecPose2D c(a);
  c += b;
  return c;
}

/******************************************************************************
 *      METHOD: operator+
 * DESCRIPTION: adds the passed differential pose to the passed pose and 
 *              returns the result  
 *****************************************************************************/
RecPose2D operator+(const RecDifferentialPose2D& a, const RecPose2D& b)
{
  return b+a;
}

/******************************************************************************
 *      METHOD: operator-
 * DESCRIPTION: subtracts the passed differential pose from the passed pose and 
 *              returns the result  
 *****************************************************************************/
RecPose2D operator-(const RecPose2D& a, const RecDifferentialPose2D& b)
{
  RecPose2D c(a);
  c -= b;
  return c;
}

/******************************************************************************
 *      METHOD: operator==
 * DESCRIPTION: converts the two poses to transforms and returns the result
 *              of transform comparison
 *****************************************************************************/
bool operator==(const RecPose2D& a, const RecPose2D& b)
{
	RecTransform2D at(a);
	RecTransform2D bt(b);
  return (at == bt);
}

/******************************************************************************
 *      METHOD: operator!=
 * DESCRIPTION: returns the complement of the == operator
 *****************************************************************************/
bool operator!=(const RecPose2D& a, const RecPose2D& b)
{
  return (!(a == b));
}

/******************************************************************************
 *      METHOD: operator<<
 * DESCRIPTION: puts the pose with nice formatting to the passed output stream
 *****************************************************************************/
ostream& operator<<(ostream& out, const RecPose2D& a)
{
  return out << "(x, y, w) = ( " << a.x << ", " << a.y 
    << ", " << a.rotZ << " )";
}


/*****************************************************************************
 * RecDifferentialPose2D Functions
 *****************************************************************************/
/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecDifferentialPose2D
 * DESCRIPTION: no argument constructor, sets x, y and rotZ to zero 
 *****************************************************************************/
RecDifferentialPose2D::RecDifferentialPose2D(void):RecPose2D() { }

/******************************************************************************
 *      METHOD: RecDifferentialPose2D
 * DESCRIPTION: three argument constructor, sets x, y and rotZ to passed values 
 *****************************************************************************/
RecDifferentialPose2D::RecDifferentialPose2D(double inx, double iny, 
                                             RecRadians inr):RecPose2D(inx, 
																						 iny, inr) { } 
/******************************************************************************
 *      METHOD: RecDifferentialPose2D
 * DESCRIPTION: copy constructor, copys the fields of the passed pose 
 *****************************************************************************/
RecDifferentialPose2D::RecDifferentialPose2D(const RecDifferentialPose2D& 
                                             b):RecPose2D(b) { }



