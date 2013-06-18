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
 *     PROJECT: REC Core Technology
 *        FILE: recLineSegment.cpp
 *      AUTHOR: Mike Bode
 *        DESC: Implementation of a 2D line segment class
 *
 * HISTORY:
 *
 * $Log: recLineSegment3D.cpp,v $
 * Revision 1.1  2005/02/07 17:59:21  ekratzer
 * Adding "recGeometry" to repository
 *
 * Revision 1.1  2004/06/14 17:09:34  colmstea
 * Adding recGeometry.
 *
 * Revision 1.1  2003/02/14 20:30:06  bode
 * Limited Initial Release.  Only classes RecRadians, RecPoint3D,
 * RecPoint3D, RecVector2D, RecVector3D, RecTransform3D and RecPose3D have
 * been well tested.
 *
 *****************************************************************************/
/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "recGeometry.h"

/*****************************************************************************
 * MACROS
 *****************************************************************************/
#ifndef MIN
#define MIN(x, y)        (((x) < (y)) ? (x) : (y))
#endif

#ifndef MAX
#define MAX(x, y)        (((x) > (y)) ? (x) : (y))
#endif


/*****************************************************************************
 * PUBLIC FUNCTIONS
 *****************************************************************************/
/*****************************************************************************
 *    FUNCTION: RecLineSegment3D
 * DESCRIPTION: constructor for RecLineSegment3D
 *****************************************************************************/
RecLineSegment3D::RecLineSegment3D()
{
  p1.x = 0;
  p1.y = 0;
	p1.z = 0;
  p2.x = 0;
  p2.y = 0;
	p2.z = 0;
}

/*****************************************************************************
 *    FUNCTION: RecLineSegment3D
 * DESCRIPTION: constructor for CLineSegment3D given 2 3D points
 *****************************************************************************/
RecLineSegment3D::RecLineSegment3D(const RecPoint3D& a, const RecPoint3D& b)
{
  p1 = a;
  p2 = b;
}

/*****************************************************************************
 *    FUNCTION: RecLineSegment3D
 * DESCRIPTION: constructor for CLineSegment2D given a line segment
 *****************************************************************************/
RecLineSegment3D::RecLineSegment3D(const RecLineSegment2D& line)
{
    p1.x = line.p1().x;
    p1.y = line.p1().y;
    p1.z = 0;
    p2.x = line.p2().x;
    p2.y = line.p2().y;
    p2.z = 0;
}

/*****************************************************************************
 *    FUNCTION: RecLineSegment3D
 * DESCRIPTION: constructor for CLineSegment3D given a line segment
 *****************************************************************************/
RecLineSegment3D::RecLineSegment3D(const RecLineSegment3D& line)
{
  p1 = line.p1;
	p2 = line.p2;
}

/*****************************************************************************
 *    FUNCTION: length
 * DESCRIPTION: returns the length of the line segment
 *****************************************************************************/
double RecLineSegment3D::length() const
{
  return(sqrt(lengthSq()));
}

/*****************************************************************************
 *    FUNCTION: lengthSq
 * DESCRIPTION: returns the squared length of the line segment
 *****************************************************************************/
double RecLineSegment3D::lengthSq() const
{
  return(p1.distanceSq(p2));
}






















