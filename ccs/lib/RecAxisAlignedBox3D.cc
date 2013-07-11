
#include "RecGeometry.h"


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
RecAxisAlignedBox3D::RecAxisAlignedBox3D(const RecAxisAlignedBox3D& box):min(box.min), max(box.max) { }

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
    return ( a.x > min.x && a.x < max.x && a.y > min.y && a.y < max.y &&
             a.z > min.z && a.z < max.z );
}

/******************************************************************************
 *      METHOD: isOutside
 * DESCRIPTION: returns true if the passed point is outside the box
 *****************************************************************************/
bool RecAxisAlignedBox3D::isOutside(const RecPoint3D& a) const
{
    return ( a.x < min.x || a.x > max.x || a.y < min.y || a.y > max.y ||
             a.z < min.z || a.z > max.z );
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
