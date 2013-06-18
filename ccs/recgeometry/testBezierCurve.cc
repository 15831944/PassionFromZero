/**
 * @file testBezierCurve.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _TESTBEZIERCURVE_CC_
#define _TESTBEZIERCURVE_CC_
#include "BezierCurve.h"
#include <iostream>
using namespace std;
int main(void)
{
    BezierCurve bc(RecPoint2D(0,0), RecPoint2D(7,7),
                   RecVector2D(0,7), RecVector2D(7,0));

    RecPoint2D p;    
    for (double t=0; t<= 1; t+=0.05)
    {
        p = bc.getCurvePoint(t);
        cout << t << " " << p.x << " " << p.y << endl;

    }


}



#endif //ifndef _TESTBEZIERCURVE_CC_
