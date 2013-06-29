/**
 * @file rectangleFitter.h
 * @author: Nikhil Naikal & Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */


#ifndef _RECTANGLEFITTER_H_
#define _RECTANGLEFITTER_H_

#include <vector>
#include <algorithm>
#include <recgeometry/recGeometry.h>
#include <iostream>
#include <math.h>
class RectangleFitter
{

    public:
    template <class Pt>
    std::vector<RecPoint2D> findBestRectangle(const std::vector<Pt> &inPts, bool junk=false);
    
    template <class Pt>
    bool searchBestRectangle(const RecPoint2D & sensorOrigin, const std::vector<Pt> & inPts, std::vector<RecPoint2D> & rectCoordinates);
                                              

    template <class Pt>
    std::vector<RecPoint2D> findLeastErrorRectangle(const std::vector<Pt> & inPts, 
                                                    const RecPoint2D & ignoreEdgePtA,
                                                    const RecPoint2D & ignoreEdgePtB);

    template <class Pt>
    std::vector<RecPoint2D> findPCARectangle(const std::vector<Pt> &inPts);

    template <class Pt>
    std::vector<RecPoint2D> findWeightedErrorRect(const std::vector<Pt> &allPts,
						  const std::vector<RecPoint2D> &weightedPts, 
						  const std::vector<int> &weights);

    private:
    
    template <class Pt>
    std::vector<RecPoint2D> fitRect(const std::vector<Pt> & cluster ,
				    const double & angle , 
  				    const RecPoint2D &centroid);

    double getErrorToRectangle(const std::vector<RecPoint2D> & pts, const std::vector<RecPoint2D> & rect);

    double findMinEdgeErrorFit(const RecPoint2D &a, const RecPoint2D &b, const std::vector<RecLineSegment2D> & rectLines);
    double calculateAreaBetween(const RecPoint2D &a, const RecPoint2D &b, const RecLineSegment2D & line);


    double fitError(std::vector<RecPoint2D> & rect, const std::vector<RecPoint2D> & weightedPts,
						    const std::vector<int> & weights);


    double curPtError(RecPoint2D & pt, std::vector<RecPoint2D> rect);
    
    double calculateEdgeCost(double edgeStart,
                             double offsetToAxis,
                             double increment,
                             double direction,
                             int countDirection,
                             unsigned int startIndex,
                             unsigned int endIndex,
                             std::vector<double> & pointsAlongAxis,
                             std::vector<double> & pointsPerpendicularAxis,
                             double edgeLength);

    inline unsigned int addToIndex(unsigned int pointIndex, unsigned int numPts, int countDirection)
    {
        if(countDirection>0)
        {
            ++pointIndex;    
            if(pointIndex==numPts)
            {
                return 0;
            }
            else
            {
                return pointIndex;
            }
        }
        else
        {
            if(pointIndex==0)
            {
                return numPts-1;
            }
            else
            {
                --pointIndex;
                return pointIndex;
            }
        }
        
        //should never get here...
        return pointIndex;
    }

//    double distFromEdge(RecPoint2D Pt1, RecPoint2D Pt2, RecPoint2D Pt3);


};

#include "rectangleFitter.def.h"

#endif //ifndef _RECTANGLEFITTER_H_

