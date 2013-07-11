
#include "RecGeometry.h"

/*****************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 *****************************************************************************/
/******************************************************************************
 *      METHOD: RecAxisAlignedBox2D
 * DESCRIPTION: no argument constructor, sets (x,y) of bounding points to zero 
 *****************************************************************************/
RecAxisAlignedBox2D::RecAxisAlignedBox2D(void):min(), max() { }

/******************************************************************************
 *      METHOD: RecAxisAlignedBox2D
 * DESCRIPTION: constructor from min and max values
 *****************************************************************************/
RecAxisAlignedBox2D::RecAxisAlignedBox2D(double minX, double maxX, double minY, 
                                         double maxY):min(minX,minY), max(maxX, maxY) {normalize(); }

/******************************************************************************
 *      METHOD: RecAxisAlignedBox2D
 * DESCRIPTION: constructor from min and max points 
 *****************************************************************************/
RecAxisAlignedBox2D::RecAxisAlignedBox2D(const RecPoint2D& minPt, 
                   const RecPoint2D& maxPt):min(minPt), max(maxPt) {normalize(); }

/******************************************************************************
 *      METHOD: RecAxisAlignedBox2D
 * DESCRIPTION: constructor from line segment
 *****************************************************************************/
RecAxisAlignedBox2D::RecAxisAlignedBox2D(const RecLineSegment2D& seg)
{
    if(seg.p1().x <= seg.p2().x)	
    {
        min.x = seg.p1().x;
        max.x = seg.p2().x;
    }
    else				//MAT 10-5-06 added ability to take a line with neg slope
    {
        min.x = seg.p2().x;
        max.x = seg.p1().x;
    }

    if(seg.p1().y <= seg.p2().y)
    {
        min.y = seg.p1().y;
        max.y = seg.p2().y;
    }
    else
    {
        min.y = seg.p2().y;
        max.y = seg.p1().y;
    }
}

/******************************************************************************
 *      METHOD: RecAxisAlignedBox2D
 * DESCRIPTION: copy constructor
 *****************************************************************************/
RecAxisAlignedBox2D::RecAxisAlignedBox2D(const RecAxisAlignedBox2D& box):min(box.min), max(box.max) { }

RecAxisAlignedBox2D::~RecAxisAlignedBox2D(){ }
/******************************************************************************
 *      METHOD: area
 * DESCRIPTION: return the area of the 2D box
 *****************************************************************************/
double RecAxisAlignedBox2D::area() const
{
  RecPoint2D diff;
  diff = max - min;
  return diff.x * diff.y;
}

/******************************************************************************
*      METHOD: isInside
* DESCRIPTION: returns true if the passed point is inside the box
*****************************************************************************/
bool RecAxisAlignedBox2D::isInside(const RecPoint2D& a) const
{
  return (a.x > min.x && a.x < max.x && a.y > min.y && a.y < max.y);
}

/******************************************************************************
*      METHOD: isInsideTiled
* DESCRIPTION: returns true if the passed point is inside the box assuming
*              that the box will be tiled and that the lower and left edges
*              are considered inside the box, but the upper and right edges
*              are considered outside the box
*****************************************************************************/
bool RecAxisAlignedBox2D::isInsideTiled(const RecPoint2D& a) const
{
  return (a.x >= min.x && a.x < max.x && a.y >= min.y && a.y < max.y);
}

/******************************************************************************
*      METHOD: isOutside
* DESCRIPTION: returns true if the passed point is outside the box
*****************************************************************************/
bool RecAxisAlignedBox2D::isOutside(const RecPoint2D& a) const
{
  return (a.x < min.x || a.x > max.x || a.y < min.y || a.y > max.y);
}

/******************************************************************************
*      METHOD: isOutsideTiled
* DESCRIPTION: returns true if the passed point is outside the box assuming
*              that the box will be tiled and that the lower and left edges
*              are considered inside the box, but the upper and right edges
*              are considered outside the box
*****************************************************************************/
bool RecAxisAlignedBox2D::isOutsideTiled(const RecPoint2D& a) const
{
  return (a.x < min.x || a.x >= max.x || a.y < min.y || a.y >= max.y);
}

/******************************************************************************
 *      METHOD: isOutside
 * DESCRIPTION: returns true if the passed point is on the surface of the box
 *****************************************************************************/
bool RecAxisAlignedBox2D::isOnBox(const RecPoint2D& a) const
{
  return (!isInside(a) && !isOutside(a));
}

/******************************************************************************
*      METHOD: getMinPoint
* DESCRIPTION: returns true if the bounding boxes overlap or share any edge
*****************************************************************************/
bool RecAxisAlignedBox2D::overlap(const RecAxisAlignedBox2D& box2) const
{
  if( !((min.x <= box2.min.x && max.x >= box2.min.x) ||
        (min.x <= box2.max.x && max.x >= box2.max.x) ||
        (min.x >= box2.min.x && max.x <= box2.max.x)) ) return false;

  if( !((min.y <= box2.min.y && max.y >= box2.min.y) ||
        (min.y <= box2.max.y && max.y >= box2.max.y) ||
        (min.y >= box2.min.y && max.y <= box2.max.y)) ) return false;

  return true;
}


/******************************************************************************
*      METHOD: getMinPoint
* DESCRIPTION: returns the box corner with the minimum x and y values
*****************************************************************************/
RecPoint2D RecAxisAlignedBox2D::getMinPoint() const {return min;}

/******************************************************************************
*      METHOD: setMinPoint
* DESCRIPTION: sets the box corner with the minimum x and y values to the 
*              passed point.  If the passed point has an x or y value
*              that is greater than the corresponding max point x or y 
*              value, this method does nothing              
*****************************************************************************/
void RecAxisAlignedBox2D::setMinPoint(const RecPoint2D& minPt)
{
  if(minPt.x > max.x) return;
  if(minPt.y > max.y) return;
  min = minPt;
}

/******************************************************************************
*      METHOD: getMaxPoint
* DESCRIPTION: returns the box corner with the maximum x and y values
*****************************************************************************/
RecPoint2D RecAxisAlignedBox2D::getMaxPoint() const {return max;}

/******************************************************************************
*      METHOD: setMaxPoint
* DESCRIPTION: sets the box corner with the maximum x and y values to the 
*              passed point.  If the passed point has an x or y value
*              that is less than the corresponding min point x or y 
*              value, this method does nothing              
*****************************************************************************/
void RecAxisAlignedBox2D::setMaxPoint(const RecPoint2D& maxPt)
{
  if(maxPt.x < min.x) return;
  if(maxPt.y < min.y) return;
  max = maxPt;
}

/******************************************************************************
*      METHOD: setMinMaxPoints
* DESCRIPTION: sets the corner points to the passed values and then normalizes
*              this allows users to get around the restrictions in setMin and
*              setMax without harming the object
*****************************************************************************/
void RecAxisAlignedBox2D::setMinMaxPoints(const RecPoint2D& minPt, const RecPoint2D& maxPt)
{
  min = minPt;
  max = maxPt;
  normalize();  // call to ensure the proper order of the points
}

/**
 * @brief this ensures that the min and max stored in the box are truely the min and max for both x and y
 */
void RecAxisAlignedBox2D::normalize(void) 
{
    RecPoint2D newMin, newMax;
    newMin.x = std::min(min.x,max.x);
    newMax.x = std::max(min.x,max.x);

    newMin.y = std::min(min.y,max.y);
    newMax.y = std::max(min.y,max.y);
    min = newMin;
    max = newMax;
}

/**
 * @brief returns a box defining the overlapping area for two boxes
 *
 * @return an empty box if there is no overlap, otherwise the axis aligned box which defines the overlapped area
 *
 * @param the other box that we are comparing against
 */
RecAxisAlignedBox2D RecAxisAlignedBox2D::overlapArea(const RecAxisAlignedBox2D& otherBox) const 
{
    RecAxisAlignedBox2D empty(0,0,0,0);  // See notes on ret.setMinPoint below
    
    if (!overlap(otherBox))  // If there is no overlap, return the zero-area rectangle at the origin
        return empty;
    
    RecPoint2D minPoint, maxPoint;

    minPoint.x = std::max( min.x, otherBox.min.x );  //MAT 10-9-60 - changed to cover more conditions of overlap
    minPoint.y = std::max( min.y, otherBox.min.y );
    maxPoint.x = std::min( max.x, otherBox.max.x );
    maxPoint.y = std::min( max.y, otherBox.max.y );

    //ret.setMinPoint(minPoint);  // These calls don't always work because the setmin and setmax point calls only 
    //ret.setMaxPoint(maxPoint);  // allow setting the min and max to certain quadrants since the box starts with 0's

    RecAxisAlignedBox2D ret(minPoint, maxPoint);
    return ret;

/*
    if (!overlap(otherBox))
        return ret;

    if (isInside(otherBox.max))
    {
        ret.setMaxPoint(otherBox.max);
        if (isInside(otherBox.min))
            ret.setMinPoint(otherBox.min);  //otherbox is fully inside 
        else
            ret.setMinPoint(min);	    //partial overlap
    } 
    else
    {
        // this rectangle is alteast partially inside the other 
        ret.setMaxPoint(max);
        if (isInside(otherBox.min))
            ret.setMinPoint(otherBox.min);
        else
            ret.setMinPoint(min);
    }
    return ret;
*/
}
