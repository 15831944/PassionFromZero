
#include "RecGeometry.h"
//#include <stdlib.h>

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
 *    FUNCTION: RecLineSegment2D
 * DESCRIPTION: constructor for RecLineSegment2D
 *****************************************************************************/
RecLineSegment2D::RecLineSegment2D()
{
  p1_.x = 0;
  p1_.y = 0;
  p2_.x = 0;
  p2_.y = 0;
  vv_.x = 1;
  vv_.y = 0;
}

/*****************************************************************************
 *    FUNCTION: RecLineSegment2D
 * DESCRIPTION: constructor for CLineSegment2D given 2 2D points
 *****************************************************************************/
RecLineSegment2D::RecLineSegment2D(const RecPoint2D& a, const RecPoint2D& b)
{
    setPoints(a,b);
}

 void RecLineSegment2D::setPoints(const RecPoint2D& a, const RecPoint2D& b)
{
    p1_ = a;
    p2_ = b;
    vv_ = b-a;
    vv_.normalize();
}

/*****************************************************************************
 *    FUNCTION: RecLineSegment2D
 * DESCRIPTION: constructor for CLineSegment2D given a line segment
 *****************************************************************************/
RecLineSegment2D::RecLineSegment2D(const RecLineSegment2D& line)
{
  p1_ = line.p1_;
  p2_ = line.p2_;
  vv_ = line.vv_;
}

/**
 * @brief constructs a line segment from a point, vector and length
 *
 * @param base the starting point for the line segment
 * @param direction a vector from p1 to p2_
 * @param length the distance between p1 and p2
 * @param needsNormalize a flag indicating wether the direction vector needs normilizing, defaults to true
 */
RecLineSegment2D::RecLineSegment2D(const RecPoint2D & base, const RecVector2D & direction, 
                                   double length, bool needsNormalize)
{
    p1_ = base;
    vv_ = direction;
    if (needsNormalize)
        vv_.normalize();
    p2_ = base+(direction*length);

}

RecPoint2D RecLineSegment2D::getClosestPointLine(const RecPoint2D & p) const
{
    RecPoint2D p1p = p-p1_;
    return p1_ + vv_ * (vv_.dot(p1p));
}

/**
 * @brief gets the distance between the passed point and the line segment, the line segment is treated as infinitely long
 */
double RecLineSegment2D::getDistanceToLine(const RecPoint2D &p) const
{
    RecPoint2D close;
    close = getClosestPointLine(p);
    return p.distance(close);
}

double RecLineSegment2D::getClosestPointLine(const RecPoint2D &p, RecPoint2D & close) const
{
    close = getClosestPointLine(p);
    return p.distance(close);
}

RecPoint2D RecLineSegment2D::getClosestPointSegment(const RecPoint2D &p) const
{
    RecPoint2D p1p = p - p1_;
    RecPoint2D p2p = p - p2_;

    // if our dot product is negative, the vector to the point in question is off the first end
    if (vv_.dot(p1p) < 0 )
        return p1_;
    // in this case the point is off the other end
    if (vv_.dot(p2p) > 0)
        return p2_;
    // the point projects onto the segment
    return p1_ + vv_*(vv_.dot(p1p));
}

double RecLineSegment2D::getClosestPointSegment(const RecPoint2D &p, RecPoint2D & close) const
{
    close = getClosestPointSegment(p);
    return p.distance(close);
}

/**
 * @brief gets the distance between the passed point and the line segment, the true segment length is used
 */
double RecLineSegment2D::getDistanceToSegment(const RecPoint2D &p) const
{
    RecPoint2D close;
    close = getClosestPointSegment(p);
    return p.distance(close);
}


/*****************************************************************************
 *    FUNCTION: length
 * DESCRIPTION: returns the length of the line segment
 *****************************************************************************/
double RecLineSegment2D::length() const
{
  return(sqrt(lengthSq()));
}

/*****************************************************************************
 *    FUNCTION: lengthSq
 * DESCRIPTION: returns the squared length of the line segment
 *****************************************************************************/
double RecLineSegment2D::lengthSq() const
{
  return(p1_.distanceSq(p2_));
}

/*****************************************************************************
 *    FUNCTION: isPointOnLineSegment
 * DESCRIPTION: support function for intersection finder
 *****************************************************************************/
int isPointOnLineSegment(double X, double Y, double X1, double Y1, double X2, double Y2)
{
    double r;
    double tol=1.0e-25;
  
    if (X1 == X2 && Y1 == Y2)
        return 0;

    if (X1 != X2)
    {
        r = (X - X1) / (X2 - X1);
    
        if (r < -tol || r > 1.0 + tol)
            return 0;
    
        if (fabs(Y1 + r * (Y2 -Y1) - Y) < tol)
            return 1;
    }
    else
    {
        r = (Y - Y1) / (Y2 - Y1);

        if (r < -tol || r > 1.0 + tol)
            return 0;
    
        if (fabs(X1 + r * (X2 - X1) - X) < tol)
            return 1;
    }

    return 0;
}

/*****************************************************************************
 *    FUNCTION: getIntersection
 */
 /*!
 * @brief returns the intersection of 2 line segments in "inter".  if there is more than 1 intersection, the 2nd point is
 *              returned in "intIfLine"
 * @return 0 if lines do not intersect
 *         1 if lines intersect in exactly one place
 *         2 if lines intersect in more than one place
 *****************************************************************************/
int RecLineSegment2D::getIntersection(const RecLineSegment2D& l2, RecPoint2D& inter, RecPoint2D& intIfLine) const
{
    int numInt=0;
    double x1, x2, x3, x4;
    double y1, y2, y3, y4;
    double n1, n2, d;
    double u1, u2;
    double tol = 1.0e-25;

    /*
    * set up some variables
    */
    x1 = p1_.x;
    x2 = p2_.x;
    x3 = l2.p1_.x;
    x4 = l2.p2_.x;

    y1 = p1_.y;
    y2 = p2_.y;
    y3 = l2.p1_.y;
    y4 = l2.p2_.y;
  
    /*
    * bounding box test
    */
    if (MAX(x1, x2) < MIN(x3, x4) || MAX(x3, x4) < MIN(x1, x2))
        return 0;

    if (MAX(y1, y2) < MIN(y3, y4) || MAX(y3, y4) < MIN(y1, y2))
        return 0;
  
    /*
    * set up some variables
    */
    n1 = (x4-x3)*(y1-y3)-(y4-y3)*(x1-x3);
    n2 = (x2-x1)*(y1-y3)-(y2-y1)*(x1-x3);
    d  = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1);

    if (fabs(d) > tol)
    {
        /*
         * parallel or skew case
         */
        u1 = n1 / d;
        u2 = n2 / d;

        if (u1 < -tol || u1 >1+tol || u2 < -tol || u2 > 1+tol)
            return 0;

        if (fabs(u1) <= tol) {
          inter.x = x1;
          inter.y = y1;
        } else if (fabs(u1-1.0) <= tol) {
          inter.x = x2;
          inter.y = y2;
        } else if (fabs(u2) <= tol) {
          inter.x = x3;
          inter.y = y3;
        } else if (fabs(u2-1.0) <= tol) {
          inter.x = x4;
          inter.y = y4;
        } else {
          inter.x = x1 + u1 * (x2 - x1);
          inter.y = y1 + u1 * (y2 - y1);
        }

        return 1;
    }

    /*
    * case when both lines are coincident
    */
    if (!(fabs(n1) < tol && fabs(n2) < tol))
        return 0;

  
    /*
    * Check to see if X1 lies in X3-X4
    */
    if (isPointOnLineSegment(x1, y1, x3, y3, x4, y4))
    {
        inter.x = x1;
        inter.y = y1;

        numInt++;
    }

    /*
    * Check to see if X2 lies in X3-X4
    */
    if (isPointOnLineSegment(x2, y2, x3, y3, x4, y4))
    {
        if (numInt == 0) {
            inter.x = x2;
            inter.y = y2;
        } else {
            intIfLine.x = x2;
            intIfLine.y = y2;

            return 2;
        }

        numInt++;
    }

    /*
    * Check to see if X3 lies in X1-X2
    */
    if (isPointOnLineSegment(x3, y3, x1, y1, x2, y2))
    {
        if (numInt == 0) {
            inter.x = x3;
            inter.y = y3;
        } else {
            intIfLine.x = x3;
            intIfLine.y = y3;

            return 2;
        }

        numInt++;
    }

    /*
    * Check to see if X4 lies in X1-X2
    */
    if (isPointOnLineSegment(x4, y4, x1, y1, x2, y2))
	{
        if (numInt==0)
        {
            inter.x = x4;
            inter.y = y4;
        }
        else
        {
            intIfLine.x = x4;
            intIfLine.y = y4;

            /*
            * Order in increasing distance to (X1,Y1)
            */
            if (sqrt((x4-x1) * (x4-x1) + (y4-y1) * (y4-y1)) < sqrt((inter.x-x1) * (inter.x-x1) + (inter.y-y1) * (inter.y-y1)))
            {
                intIfLine.x = inter.x;
                intIfLine.y = inter.y;
                inter.x = x4;
                inter.y = y4;
            }
        }
    }

    return 2;
}  

/**
 * @brief returns plus or minus 1 indicating which side of the line segment the
 * point is on if the point is on the line it returns 0
 */
int RecLineSegment2D::whichSideIsPointOn(const RecPoint2D & p) const
{
    RecVector2D vec(p-p1_);
    RecVector2D norm = vec.normal();
    double dotprod =vv_.dot(norm);
    if (dotprod==0.0)
        return 0;

    return (dotprod<0)?-1:1;
}
