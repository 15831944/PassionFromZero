/**
 * @file BezierCurve.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 11/16/2006
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _BEZIERCURVE_CC_
#define _BEZIERCURVE_CC_
#include "BezierCurve.h"

/**
 * @brief creates a bezier curve
 * @param start,end the start and end points of the curve
 * @param startDirection,endDirection the direction the curve should have at the start and end points
 */
BezierCurve::BezierCurve(RecPoint2D start, RecPoint2D end,
                         RecVector2D startDirection, RecVector2D endDirection)
{
    x0_ = start;
    RecPoint2D x1,x2,x3;
    x1 = start+startDirection;
    x3 = end;
    x2 = end-endDirection;
    
    c_ = 3*(x1-x0_);
    b_ = 3*(x2-x1)-c_;
    a_ = x3-x0_ - c_ - b_;
}

/**
 * @brief creates a bezier curve
 * @param x0 the start point
 * @param x1 control point1
 * @param x2 control point2
 * @param x3 the end point
 */
BezierCurve::BezierCurve(RecPoint2D x0, RecPoint2D x1,
                         RecPoint2D x2, RecPoint2D x3)
{
    x0_ = x0;
    
    c_ = 3*(x1-x0);
    b_ = 3*(x2-x1)-c_;
    a_ = x3-x0 - c_ - b_;

}

/**
 * @brief gets a point on the curve.
 * @param t ranges between 0 and 1 (0 being the start point, end being the end piont).  
 */
RecPoint2D BezierCurve::getCurvePoint(double t)
{
    if (t<0.0)
        return x0_;
    if (t>1.0)
        t=1.0;
    return ((a_*t+b_)*t+c_)*t+x0_;
}

        



#endif //ifndef _BEZIERCURVE_CC_
