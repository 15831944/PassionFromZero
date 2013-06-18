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
 *        FILE:	RecBox3D.cpp
 * DESCRIPTION: Implementation of a 3D Box class
 *     CREATED: 2003/03/06
 *    
 * HISTORY:
 *
 * $Log: recBox3D.cpp,v $
 * Revision 1.1  2005/02/07 17:59:21  ekratzer
 * Adding "recGeometry" to repository
 *
 * Revision 1.1  2004/06/14 17:09:34  colmstea
 * Adding recGeometry.
 *
 * Revision 1.4  2003/10/22 11:39:16  temp
 * Merged tomPNNetBranchTag which includes support for what columns in the
 * density map look like at different ranges from the vehicle into main
 * line code.
 *
 * Revision 1.2.4.1  2003/10/21 12:52:19  temp
 * merged main line Spiral2 code into my branch
 *
 * Revision 1.3  2003/10/13 19:51:49  bode
 * Added access methods for getting the min/max coordinates of the box
 *
 * Revision 1.2  2003/06/18 05:55:57  bode
 * Fixed RecBox3D constructor and removed using statement form recGeometry.h
 *
 * Revision 1.1  2003/03/11 22:02:59  bode
 * Initial Release
 *
 *
 *****************************************************************************/
/*****************************************************************************
 * INCLUDES
 *****************************************************************************/
#include <iostream>
#include <math.h>
#include "recGeometry.h"


/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecAxisAlignedBox3D
 * DESCRIPTION: no argument constructor, sets (x,y) of bounding points to zero 
 *****************************************************************************/
RecAxisAlignedBox3D::RecAxisAlignedBox3D(void):min(), max() { }

/******************************************************************************
 *      METHOD: RecAxisAlignedBox3D
 * DESCRIPTION: constructor from min and max values
 *****************************************************************************/
RecAxisAlignedBox3D::RecAxisAlignedBox3D(double minX, double maxX, double minY, 
                   double maxY, double minZ, double maxZ):min(minX,minY,minZ),
                   max(maxX, maxY, maxZ) {normalize(); }

/******************************************************************************
 *      METHOD: RecAxisAlignedBox3D
 * DESCRIPTION: constructor from min and max points 
 *****************************************************************************/
RecAxisAlignedBox3D::RecAxisAlignedBox3D(const RecPoint3D& minPt, 
                   const RecPoint3D& maxPt):min(minPt), max(maxPt)  {normalize(); }

/******************************************************************************
 *      METHOD: RecAxisAlignedBox3D
 * DESCRIPTION: constructor from line segment
 *****************************************************************************/
RecAxisAlignedBox3D::RecAxisAlignedBox3D(const RecLineSegment3D& seg)
{
  if(seg.p1.x <= seg.p2.x)
  {
    min.x = seg.p1.x;
    max.x = seg.p2.x;
  }
  else
  {
    min.x = seg.p2.x;
    max.x = seg.p1.x;
  }
  if(seg.p1.y <= seg.p2.y)
  {
    min.y = seg.p1.y;
    max.y = seg.p2.y;
  }
  else
  {
    min.y = seg.p2.y;
    max.y = seg.p1.y;
  }
  if(seg.p1.z <= seg.p2.z)
  {
    min.z = seg.p1.z;
    max.z = seg.p2.z;
  }
  else
  {
    min.z = seg.p2.z;
    max.z = seg.p1.z;
  }
}

/******************************************************************************
 *      METHOD: RecAxisAlignedBox3D
 * DESCRIPTION: copy constructor
 *****************************************************************************/
RecAxisAlignedBox3D::RecAxisAlignedBox3D(const 
                                         RecAxisAlignedBox3D& box):min(box.min),
                                         max(box.max) { }

/******************************************************************************
 *      METHOD: area
 * DESCRIPTION: return the area of the 3D box
 *****************************************************************************/
double RecAxisAlignedBox3D::volume() const
{
  RecPoint3D diff;
  diff = max - min;
  return diff.x * diff.y * diff.z;
}

/******************************************************************************
 *      METHOD: isInside
 * DESCRIPTION: returns true if the passed point is inside the box
 *****************************************************************************/
bool RecAxisAlignedBox3D::isInside(const RecPoint3D& a) const
{
  return (a.x > min.x && a.x < max.x && a.y > min.y && a.y < max.y && 
    a.z > min.z && a.z < max.z);
}

/******************************************************************************
 *      METHOD: isOutside
 * DESCRIPTION: returns true if the passed point is outside the box
 *****************************************************************************/
bool RecAxisAlignedBox3D::isOutside(const RecPoint3D& a) const
{
  return (a.x < min.x || a.x > max.x || a.y < min.y || a.y > max.y ||
    a.z < min.z || a.z > max.z);
}

/******************************************************************************
 *      METHOD: isOutside
 * DESCRIPTION: returns true if the passed point is on the surface of the box
 *****************************************************************************/
bool RecAxisAlignedBox3D::isOnBox(const RecPoint3D& a) const
{
  return (!isInside(a) && !isOutside(a));
}

/******************************************************************************
 *      METHOD: getMin
 * DESCRIPTION: returns the minimum x,y,z point of the box
 *****************************************************************************/
RecPoint3D RecAxisAlignedBox3D::getMin() const
{
  return min;
}

/******************************************************************************
 *      METHOD: getMax
 * DESCRIPTION: returns the maximum x,y,z point of the box
 *****************************************************************************/
RecPoint3D RecAxisAlignedBox3D::getMax() const
{
  return max;
}

/******************************************************************************
 *      METHOD: normalize
 * DESCRIPTION: places the proper x,y,z values in the min and max points
 *****************************************************************************/
void RecAxisAlignedBox3D::normalize(void)   //MAT- added 10-6-06
{
    RecPoint3D newMin, newMax;
    newMin.x = std::min(min.x,max.x);
    newMax.x = std::max(min.x,max.x);

    newMin.y = std::min(min.y,max.y);
    newMax.y = std::max(min.y,max.y);

    newMin.z = std::min(min.z,max.z);
    newMax.z = std::max(min.z,max.z);

    min = newMin;
    max = newMax;
}
