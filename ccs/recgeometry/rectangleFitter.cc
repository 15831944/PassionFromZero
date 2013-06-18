/**
 * @file rectangleFitter.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _RECTANGLEFITTER_CC_
#define _RECTANGLEFITTER_CC_
#include "rectangleFitter.h"
using namespace std;
double RectangleFitter::getErrorToRectangle(const std::vector<RecPoint2D> & pts, const std::vector<RecPoint2D> & rect)
{
    double error=0;
    std::vector<RecLineSegment2D> lines;
    lines.resize(4);
    // setup four line segments that we'll use to calculate the rest of this
    for (unsigned int ii=0; ii< rect.size(); ++ii)
    {
        lines[ii].setPoints(rect[ii], rect[(ii+1)%rect.size()]);
    }

    // now find the sum minimum error around the polygon
    for (unsigned int ii=0; ii <pts.size()-1; ++ii)
    {
        error+= findMinEdgeErrorFit(pts[ii], pts[ii+1], lines);
    }
        return error;
}

double RectangleFitter::findMinEdgeErrorFit(const RecPoint2D &a, const RecPoint2D &b, const std::vector<RecLineSegment2D> & rectLines)
{
    if (rectLines.size()==0)
    {
        return 0;
    }
    double error;
    double minError = calculateAreaBetween(a,b,rectLines[0]);
    for (unsigned int ii=1; ii < rectLines.size(); ++ ii)
    {
        error = calculateAreaBetween(a,b,rectLines[ii]);
        if (error < minError)
        {
            minError = error;
        }

    }
    return minError;
}

double RectangleFitter::calculateAreaBetween(const RecPoint2D &a, const RecPoint2D &b, const RecLineSegment2D & line)
{
    double area;
    RecPoint2D closeA, closeB;
    double disA, disB;
    double projDis;
    int sideA, sideB;
    disA = line.getClosestPointLine(a,closeA);
    disB = line.getClosestPointLine(b,closeB);
    sideA = line.whichSideIsPointOn(a);
    sideB = line.whichSideIsPointOn(b);

    projDis = closeA.distance(closeB);

    if (sideA == sideB)
    {
        // in this case the area is described by a rectangle (the closest distance * the distance between the proejected
        // points) plus a triangle that is the diference between the distances from the line * the distance between the
        // projected points

        if (disA < disB)
        {
            area = disA * projDis + (disB-disA)*projDis*0.5;
        }
        else
        {
            area = disB * projDis + (disA-disB)*projDis*0.5;
        }
    }
    else
    {
        // in this case we have points on opposite sides of the line, so we have two triangles, with an intersect at disA/(disA+disB)*projDistance
        area = (disA*disA + disB*disB) * projDis/ ( 2 * disA + disB);
    }


    return area;
}


// total error of fitting weighted points to current rectangle
double RectangleFitter::fitError(std::vector<RecPoint2D> & rect, const std::vector<RecPoint2D> & weightedPts,
								 const std::vector<int> & weights)
{
    double totalError = 0;
    for (unsigned int ii = 0; ii<weightedPts.size(); ++ii)
    {
        RecPoint2D curPt = weightedPts[ii];
	double curError = curPtError(curPt, rect);
	int curWt = weights[ii];
        curError *= curWt;
	totalError += curError;
    }
    return totalError;
}

// distance between point and closest edge of rectangle
double RectangleFitter::curPtError(RecPoint2D & pt, std::vector<RecPoint2D> rect)
{
    double minDist = 9e9;
    rect.push_back(rect[0]);
    for (unsigned int ii = 0; ii<(rect.size()-1); ++ii)
    {
	RecLineSegment2D curLine(rect[ii], rect[ii+1]);
        
	double dist = curLine.getDistanceToLine(pt);
        if (dist<minDist)
	{
	    minDist = dist;
	}
    }
    return minDist;
}



double RectangleFitter::calculateEdgeCost(double edgeStart,
                                          double offsetToAxis,
                                          double increment,
                                          double direction,
                                          int countDirection,
                                          unsigned int startIndex,
                                          unsigned int endIndex,
                                          std::vector<double> & pointsAlongAxis,
                                          std::vector<double> & pointsPerpendicularAxis,
                                          double edgeLength)
{
    //for(unsigned int counter=0;counter<numPts;++counter)
    //{
    //    cout << "("<< pointsAlongAxis[counter] << "," << pointsPerpendicularAxis[counter] << ")" << endl;
    //}
    //cout << endl;
    double cost=0;
    
    //sanity checks
    unsigned int numPts=pointsAlongAxis.size();
    if(numPts!=pointsPerpendicularAxis.size())
    {
        //cannot work
        return 0;
    }
    
    if(   (startIndex>numPts)
        ||(endIndex>numPts))
    {
        //cannot work
        return 0;
    }
    
    
    //walk along the axis...
    unsigned int stepCounter=1;
    unsigned int pointIndex=startIndex;
    unsigned int nextPointIndex=addToIndex(pointIndex,numPts,countDirection);
    
    while(increment*stepCounter<=edgeLength+0.001) //add a small (in comparison to increment) number due to numerical instabilities
    {
        //test coordinate
        double test=edgeStart+direction*increment*stepCounter;
        double diff1=test-pointsAlongAxis[pointIndex];
        double diff2=test-pointsAlongAxis[nextPointIndex];
        
        //update to the correct pair of pts...
        unsigned int loopCounter=0;
        while( (  ((diff1<0)&&(diff2<0))
                ||((diff1>0)&&(diff2>0))
               )
               &&(nextPointIndex!=endIndex) //we do not want to overshoot (again numerical instabilities)
             )
        {
            pointIndex=addToIndex(pointIndex,numPts,countDirection);
            nextPointIndex=addToIndex(nextPointIndex,numPts,countDirection);
            diff1=test-pointsAlongAxis[pointIndex];
            diff2=test-pointsAlongAxis[nextPointIndex];
            
            ++loopCounter;
            if(loopCounter>numPts) //should not need more than that
            {
                //uhh - bad, infinite loop
                return 0;
            }
        }
        
        //ok have the right pair
        //cout << "Found pair" << endl;
        //cout << test <<","<<pointsAlongAxis[pointIndex]<<","<<pointsAlongAxis[nextPointIndex]<<endl;
        
        //calculate the distance...
        
        double dx=pointsAlongAxis[pointIndex]-pointsAlongAxis[nextPointIndex];
        double dy=pointsPerpendicularAxis[pointIndex]-pointsPerpendicularAxis[nextPointIndex];
        double distance=0;
        if(fabs(dx)<0.01)
        {
            distance=pointsPerpendicularAxis[pointIndex];
        }
        else
        {
            distance=pointsPerpendicularAxis[pointIndex]+(dy/dx)*(test-pointsAlongAxis[pointIndex]);
        }
        
        //cout << "Distance: " << distance << endl;
        
        distance=distance-offsetToAxis;
        
        cost+=pow(distance,6);
        ++stepCounter;
    }
    
    return cost;
}



/*

double RectangleFitter::distFromEdge(RecPoint2D Pt1, RecPoint2D Pt2, RecPoint2D Pt3)
{

    RecPoint2D Pt;
    double u, dist;
    u = (((Pt3.x-Pt1.x)*(Pt2.x-Pt1.x)) + ((Pt3.y-Pt1.y)*(Pt2.y-Pt1.y)))
	 / (pow((Pt2.x-Pt1.x),2) + pow((Pt2.y-Pt1.y),2));
    Pt.x = Pt1.x + (u * (Pt2.x-Pt1.x));
    Pt.y = Pt1.y + (u * (Pt2.y-Pt1.y));
    dist = sqrt(pow((Pt.x-Pt3.x),2) + pow((Pt.y-Pt3.y),2));
    return dist;
}

*/

#endif //ifndef _RECTANGLEFITTER_CC_
