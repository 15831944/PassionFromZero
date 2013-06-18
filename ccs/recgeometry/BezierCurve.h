/**
 * @file BezierCurve.h
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 11/16/2006
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _BEZIERCURVE_H_
#define _BEZIERCURVE_H_
#include "recGeometry.h"
/**
 * @brief a simple class to generate a bezier curve given a start and end point and a pair of vectors
 * @ingroup recGeometryGroup
 *
 * math source from: http://www.moshplant.com/direct-or/bezier/math.html
 */
class BezierCurve 
{
    RecPoint2D x0_;
    RecPoint2D a_,b_,c_;
  public:

    BezierCurve(RecPoint2D start, RecPoint2D end,
                 RecVector2D startDirection, RecVector2D endDirection);
    BezierCurve(RecPoint2D x0, RecPoint2D x1,
                RecPoint2D x2, RecPoint2D x3);

    RecPoint2D getCurvePoint(double t);
};



#endif //ifndef _BEZIERCURVE_H_
