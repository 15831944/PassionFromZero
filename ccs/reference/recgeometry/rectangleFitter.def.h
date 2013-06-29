/**
 * @file rectangleFitter.def.h
 * @author: Nikhil Santosh & Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _RECTANGLEFITTER_DEF_H_
#define _RECTANGLEFITTER_DEF_H_

#ifndef _RECTANGLEFITTER_H_
#error "must only be included from rectangleFitter.h"
#endif

#include <iostream>
using namespace std;
#define N_PI 3.141593


/**
 * @brief finds the best bounding rectangle for the 2D projection of the passed points
 * @param inPts must be a vector of a type that has .x and .y members
 * @return a std::vector of the four points defining the corners of the best bounding rectangle found
 */
template <class Pt>
std::vector<RecPoint2D> RectangleFitter::findBestRectangle(const std::vector<Pt> & inPts, bool junk)


{

    std::vector<RecPoint2D> hullPts;
    RecPolygon2D nonConvexPolygon(inPts);
    RecPolygon2D convexPolygon;
    std::vector<RecPoint2D> rectCoordinates;
    rectCoordinates.resize(4);
    double area = 100000, theta;

    // Find the convex hull of the set of points in 2D
    nonConvexPolygon.computeConvexHull(convexPolygon);

    hullPts = convexPolygon.getVertices();

    unsigned int numPts = hullPts.size()-1;


    // Find the 2 points that are furthest from each other in the hull set
    double distance, maxDist = 0;
    int count = 0;
    int id0=0, id1=0;
    for (unsigned int ii=0; ii<numPts; ++ii)
    {
	for (unsigned int jj=0; jj<numPts; ++jj)
	{
	    distance = sqrt(pow((hullPts[ii].x-hullPts[jj].x),2) + pow((hullPts[ii].y-hullPts[jj].y),2));
	    ++count;
	    if (maxDist < distance)
	    {
		maxDist = distance;
                id0 = ii;
                id1 = jj;
	    }
	}
    }
    

    RecPoint2D _point0, _point1;
    _point0 = hullPts[id0];
    _point1 = hullPts[id1];

    // Find the mid point of the 2 which will act as the origin, and the angle to rotate the hull points by
    RecPoint2D midPt;
    midPt.x = (_point0.x+_point1.x)/2;
    midPt.y = (_point0.y+_point1.y)/2;
    theta = atan2((_point1.y-_point0.y), (_point1.x-_point0.x));


    // Translate and Rotate the convex polygon and find the reflection on the polygon about the mid point
    // found previously

    // Store the points in a new vector, as the previous set will be needed later
    std::vector<RecPoint2D> newPoly;
    for (unsigned int ii = 0; ii<numPts; ++ii)
    {
	RecPoint2D tmp, newTmp, itmp, inewTmp;
	tmp.x = (hullPts[ii].x-midPt.x)*cos(theta) + (hullPts[ii].y-midPt.y)*sin(theta);
	tmp.y = -(hullPts[ii].x-midPt.x)*sin(theta) + (hullPts[ii].y-midPt.y)*cos(theta);
	newTmp.x = (-1)*tmp.x;
	newTmp.y = (-1)*tmp.y;

	// Rotate and translated the new set of points
	itmp.x = (tmp.x*cos(theta))-(tmp.y*sin(theta))+midPt.x;
	itmp.y = (tmp.x*sin(theta))+(tmp.y*cos(theta))+midPt.y;
	inewTmp.x = (newTmp.x*cos(theta))-(newTmp.y*sin(theta))+midPt.x;
	inewTmp.y = (newTmp.x*sin(theta))+(newTmp.y*cos(theta))+midPt.y; 

	newPoly.push_back(itmp);
	newPoly.push_back(inewTmp);
    }

    // Find the convex polygon of the new set of points
    std::vector<RecPoint2D> newHullPts;
    RecPolygon2D newNonConvexPolygon(newPoly);
    RecPolygon2D newConvexPolygon;
    newNonConvexPolygon.computeConvexHull(newConvexPolygon);

    newHullPts = newConvexPolygon.getVertices();

    // Use the new convex polygon to find the angle of the oriented rectangle
    unsigned int numNewPts = newHullPts.size() - 1;
    RecPoint2D clusterMean;
    

    clusterMean.x = 0; clusterMean.y = 0;
    for (unsigned int ii=0; ii<numNewPts; ++ii)
    {
        clusterMean += newHullPts[ii];
    }

    clusterMean.x /= numNewPts;
    clusterMean.y /= numNewPts;

    // Translate the Hull Points by their mean
    std::vector<RecPoint2D> txPts;
    for (unsigned int ii=0; ii<numNewPts; ++ii)
    {
        RecPoint2D tmp;
        tmp = newHullPts[ii] - clusterMean;

        txPts.push_back(tmp);
    } 

    // Find the angle between the x-axis and the line joining each point and its successive point
    // The angle that minimizes area of fitting rectangle is chosen as the orientation


    double finalTheta=0;  
    for (unsigned int ii = 0; ii<numNewPts; ++ii)
    {
        theta = atan2((txPts[(ii+1)%numNewPts].y-txPts[ii].y),
                      (txPts[(ii+1)%numNewPts].x-txPts[ii].x));
	
        double minL = 0, maxL = 0, minW = 0, maxW = 0;
        
        for (unsigned int jj = 0; jj<numNewPts; ++jj)
        {
            double tempX = (txPts[jj].x * cos(theta)) + (txPts[jj].y * sin(theta));
            double tempY = (txPts[jj].y * cos(theta)) - (txPts[jj].x * sin(theta));

            minL = std::min(tempX, minL);
            maxL = std::max(tempX, maxL);
            minW = std::min(tempY, minW);
            maxW = std::max(tempY, maxW);

        }
        
	double newArea = (fabs(minL)+fabs(maxL))*(fabs(minW)+fabs(maxW));
        if (newArea<area)
	{
	    area = newArea;
	    finalTheta = theta;
	}
    }

    // Use the angle above to fit the oriented rectangle to the cluster

    clusterMean.x = 0; clusterMean.y = 0;
    for (unsigned int ii=0; ii<numPts; ++ii)
    {
        clusterMean += hullPts[ii];
    }

    clusterMean.x /= numPts;
    clusterMean.y /= numPts;
	
    double minL = 0, maxL = 0, minW = 0, maxW = 0;
    for (unsigned int ii=0; ii<numPts; ++ii)
    {
        RecPoint2D transTmp, rotTmp ;
        transTmp = hullPts[ii] - clusterMean;
	rotTmp.x = (transTmp.x*cos(finalTheta))+(transTmp.y*sin(finalTheta));
	rotTmp.y = -(transTmp.x*sin(finalTheta))+(transTmp.y*cos(finalTheta));
	if (rotTmp.x < minL)
            minL = rotTmp.x;
        if (rotTmp.x > maxL)
            maxL = rotTmp.x;
        if (rotTmp.y < minW)
            minW = rotTmp.y;
        if (rotTmp.y > maxW)
            maxW = rotTmp.y;
    } 
    
    // Apply the right rotation matrix and find the length and width of the rectangle
	
        
    rectCoordinates[0].x = maxL*cos(finalTheta) - maxW*sin(finalTheta) + clusterMean.x;
    rectCoordinates[0].y = maxL*sin(finalTheta) + maxW*cos(finalTheta) + clusterMean.y;
           
    rectCoordinates[1].x = maxL*cos(finalTheta) - minW*sin(finalTheta) + clusterMean.x;
    rectCoordinates[1].y = maxL*sin(finalTheta) + minW*cos(finalTheta) + clusterMean.y;

    rectCoordinates[2].x = minL*cos(finalTheta) - minW*sin(finalTheta) + clusterMean.x;
    rectCoordinates[2].y = minL*sin(finalTheta) + minW*cos(finalTheta) + clusterMean.y;        

    rectCoordinates[3].x = minL*cos(finalTheta) - maxW*sin(finalTheta) + clusterMean.x;
    rectCoordinates[3].y = minL*sin(finalTheta) + maxW*cos(finalTheta) + clusterMean.y; 

    return rectCoordinates;
}

template <class Pt>
std::vector<RecPoint2D> RectangleFitter::findPCARectangle(const std::vector<Pt> &inPts)
{
    
    double a11, a12, a21, a22;
    double eig;
    RecPoint2D evec;
    unsigned int clusterSize = inPts.size();
    double sumX = 0, sumY = 0, sumXY = 0;
    double sumX2 = 0, sumY2 = 0;
    double angle;
    std::vector<RecPoint2D> bestRectCoordinates;

    // Find the mean of the clusters
    RecPoint2D centroid = RecPoint2D(0,0);
    for (unsigned int ii = 0; ii < inPts.size(); ++ii)
    {
	centroid += RecPoint2D(inPts[ii].x, inPts[ii].y);
    }
    centroid.x /= inPts.size();
    centroid.y /= inPts.size();
  

    // Find the elements of the covariance matrix
    // we subtract out the centroid to make this math numerically stable... otherwise bad things happen...
    for (unsigned int ii=0; ii < clusterSize; ++ii)
    {
        sumX += inPts[ii].x-centroid.x;
        sumY += inPts[ii].y-centroid.y;
    	sumXY += (inPts[ii].x-centroid.x) * (inPts[ii].y-centroid.y);         
        sumX2 += (inPts[ii].x-centroid.x) * (inPts[ii].x-centroid.x);
        sumY2 += (inPts[ii].y-centroid.y) * (inPts[ii].y-centroid.y);
    }

    a11 = (sumX2 - ((sumX * sumX)/clusterSize))/clusterSize;
    a12 = (sumXY - ((sumX * sumY)/clusterSize))/clusterSize;
    a21 = a12;
    a22 = (sumY2 - ((sumY * sumY)/clusterSize))/clusterSize;
    
    //The larger Eigen Values
    eig = 0.5 * ((a11+a22) + sqrt((4*a12*a21) + ((a11-a22)*(a11-a22))));// Larger EigenValue


    // The 1st Eigen Vector
    evec.x = sqrt(1/(1 + pow(((eig-a11-a12)/(a12+a22-eig)),2)));
    evec.y = ((eig - a11 - a12)/(a12 + a22 - eig))*evec.x;
  
    // The angle of orientation is found between 0 and pi/2

    angle = atan2(evec.y, evec.x);
    while (angle > (M_PI/2.0))
    {
	angle = angle-(M_PI/2.0);
    }
    while (angle < 0)
    {
	angle = angle+(M_PI/2.0);
    }

    // Find the coordinates of the bounding rectangle with this angle
    
    bestRectCoordinates = fitRect(inPts, angle, centroid);

    return bestRectCoordinates;

}

// Function to fit a rectangle to the points
template <class Pt>
std::vector<RecPoint2D> RectangleFitter::fitRect(const std::vector<Pt> & cluster ,
						 const double & angle, 
						 const RecPoint2D &centroid)
{
    
    std::vector<RecPoint2D> rectCoordinates(4);

    double sinAngle = sin(angle);
    double cosAngle = cos(angle);
    unsigned int numPts = cluster.size();
	
    // Translate and rotate each point, and find the bounds of the points in the x and y axes

    // The minimum rectangle is 1cm by 1cm and is centered at the centroid of the points
    double minL = -0.01, maxL = 0.01, minW = -0.01, maxW = 0.01;
    for (unsigned int ii=0; ii<numPts; ++ii)
    {
        RecPoint2D transTmp, rotTmp ;
        
        transTmp.x = cluster[ ii ].x - centroid.x;
        transTmp.y = cluster[ ii ].y - centroid.y;
        
	rotTmp.x = (transTmp.x*cosAngle)+(transTmp.y*sinAngle);
	rotTmp.y = -(transTmp.x*sinAngle)+(transTmp.y*cosAngle);
        
        minL = std::min(rotTmp.x, minL);
        maxL = std::max(rotTmp.x, maxL);
        minW = std::min(rotTmp.y, minW);
        maxW = std::max(rotTmp.y, maxW);
    } 
    


    // Apply the inverse rotation and translation and find the coordinates of the rectangle
	        
    rectCoordinates[0].x = maxL*cosAngle - maxW*sinAngle + centroid.x;
    rectCoordinates[0].y = maxL*sinAngle + maxW*cosAngle + centroid.y;
           
    rectCoordinates[1].x = maxL*cosAngle - minW*sinAngle + centroid.x;
    rectCoordinates[1].y = maxL*sinAngle + minW*cosAngle + centroid.y;

    rectCoordinates[2].x = minL*cosAngle - minW*sinAngle + centroid.x;
    rectCoordinates[2].y = minL*sinAngle + minW*cosAngle + centroid.y;        

    rectCoordinates[3].x = minL*cosAngle - maxW*sinAngle + centroid.x;
    rectCoordinates[3].y = minL*sinAngle + maxW*cosAngle + centroid.y; 

    return rectCoordinates;

}

template <class Pt>
std::vector<RecPoint2D> RectangleFitter::findWeightedErrorRect(const std::vector<Pt> &allPts,
	    				    		       const std::vector<RecPoint2D> &weightedPts, 
					    		       const std::vector<int> &weights)
{


    // mean of the points
    RecPoint2D centroid;
    centroid = RecPoint2D(0,0);
    for (unsigned int ii = 0; ii<allPts.size(); ++ii)
    {
	centroid.x += allPts[ii].x;
	centroid.y += allPts[ii].y;
    }
    centroid.x /= allPts.size();
    centroid.y /= allPts.size();

      
    std::vector<RecPoint2D> curRect, bestRect;
    
    double minError=9e9;
    double minErrorAngle;

    // find weighted error of fitting rectangle with discretized angles between 0-90 degrees
    for (unsigned int ii = 0; ii<90; ++ii)
    {
	double curAngle = ii * (N_PI/180);
	double curError;
	curRect = fitRect(weightedPts, curAngle, centroid);
	curError = fitError(curRect, weightedPts, weights);
	if (minError > curError)
	{
	    minError = curError;
	    minErrorAngle = curAngle;
	}
    }

    // fit rectangle of minimum error
    bestRect = fitRect(allPts, minErrorAngle, centroid);
    return bestRect;

}




template <class Pt>
bool RectangleFitter::searchBestRectangle(const RecPoint2D & sensorOrigin, 
                                          const std::vector<Pt> & inPts, 
                                          std::vector<RecPoint2D> & resultCoordinates)
{
    //parameters 
    //FIXME: need to move somwhere else, maybe parameters for the method
    const unsigned int NUMBER_OF_ANGLE_STEPS=40; //number of angles for the brute force search - increase to increase precision of angle
    const unsigned int NUMBER_OF_COST_POINTS_LONGE_EDGE=5; //number of points on the longer side of the L to be used in the cost function
    
    //some sanity checks
    
    resultCoordinates.resize(4, RecPoint2D(0,0));
    
    if(inPts.empty())
    {
        return false;
    }
    
    
    // Find the convex hull of the set of points in 2D
    RecPolygon2D nonConvexPolygon(inPts);
    RecPolygon2D convexPolygon;
    nonConvexPolygon.computeConvexHull(convexPolygon);
    std::vector<RecPoint2D> hullPoints = convexPolygon.getVertices();
    unsigned int numPts = hullPoints.size()-1;
    
    //sanity checks
    if(numPts==0)
    {
        return false;
    }
    
    if(numPts==1)
    {
        for(unsigned counter=0;counter<4;++counter)
        {
            resultCoordinates[counter].x=hullPoints[0].x;
            resultCoordinates[counter].y=hullPoints[0].y;
        }
        return false;
    }
    
    //special case
    if(numPts==2)
    {
        
        resultCoordinates[0]=hullPoints[0];
        resultCoordinates[1]=hullPoints[0];
        
        resultCoordinates[2]=hullPoints[1];
        resultCoordinates[3]=hullPoints[1];
        
        return true;
    }
    
    //find the centroid
    RecPoint2D centroid = RecPoint2D(0,0);
    for (unsigned int ii = 0; ii < numPts; ++ii)
    {
        centroid += RecPoint2D(hullPoints[ii].x, hullPoints[ii].y);
    }
    centroid.x /= numPts;
    centroid.y /= numPts;
  
    //we will search for the best L shape with a brute force search
    //FIXME: this should be done by a more intelligent way maybe a Newton search. However I am not sure about local minima
    
    double minCost=9e9;
    double bestAngle=0;
    
    std::vector<RecPoint2D> rectCoordinates(4);
    
    std::vector<double> translatedPointsX(numPts);
    std::vector<double> translatedPointsY(numPts);
    
    RecPoint2D translatedSensorOrigin;

    for(unsigned int angleCounter=0;angleCounter<NUMBER_OF_ANGLE_STEPS;++angleCounter)
    {
        double angle=angleCounter/(double)NUMBER_OF_ANGLE_STEPS*(90.0f*M_PI/180.0f);
        
        //translate the points
        
        double sinAngle = sin(angle);
        double cosAngle = cos(angle);
        
        RecPoint2D transTmp;
        
        double minX = -0.001, maxX = 0.001, minY = -0.001, maxY = 0.001;
        unsigned int indexMaxX=numPts+1, indexMinX=numPts+1, indexMaxY=numPts+1, indexMinY=numPts+1;
        for (unsigned int pointCounter=0; pointCounter<numPts; ++pointCounter)
        {
            
            //could be moved outside the loop
            transTmp.x = hullPoints[pointCounter].x - centroid.x;
            transTmp.y = hullPoints[pointCounter].y - centroid.y;
            
            translatedPointsX[pointCounter]=  (transTmp.x*cosAngle)+(transTmp.y*sinAngle);
            translatedPointsY[pointCounter]= -(transTmp.x*sinAngle)+(transTmp.y*cosAngle);
            
            if(translatedPointsX[pointCounter]<minX)
            {
                minX=translatedPointsX[pointCounter];
                indexMinX=pointCounter;
            }
            
            if(translatedPointsY[pointCounter]<minY)
            {
                minY=translatedPointsY[pointCounter];
                indexMinY=pointCounter;
            }
            
            if(translatedPointsX[pointCounter]>maxX)
            {
                maxX=translatedPointsX[pointCounter];
                indexMaxX=pointCounter;
            }
            
            if(translatedPointsY[pointCounter]>maxY)
            {
                maxY=translatedPointsY[pointCounter];
                indexMaxY=pointCounter;
            }
        }
        
        //check if we found the min/max values
        if(  (indexMaxX>numPts)
           ||(indexMaxY>numPts)
           ||(indexMinX>numPts)
           ||(indexMinY>numPts))
        {
            //this means we did not find the correct values
            //cannot calculate a fit, will keep the current best value and continue
            continue;
        }
        
        rectCoordinates[0].x = minX;
        rectCoordinates[0].y = minY;
           
        rectCoordinates[1].x = maxX;
        rectCoordinates[1].y = minY;

        rectCoordinates[2].x = maxX;
        rectCoordinates[2].y = maxY;        

        rectCoordinates[3].x = minX;
        rectCoordinates[3].y = maxY; 
	
        //translate the sensor origin
        transTmp.x = sensorOrigin.x - centroid.x;
        transTmp.y = sensorOrigin.y - centroid.y;
        
        translatedSensorOrigin.x =  (transTmp.x*cosAngle)+(transTmp.y*sinAngle);
        translatedSensorOrigin.y = -(transTmp.x*sinAngle)+(transTmp.y*cosAngle);
        
        //ok the L is defined by the closest corner and the two adjacent vertices
        //find these points...
        
        
        //could check for quadrant, would be much more efficient
        //find the closest corner
        unsigned int indexMin=0;
        double minDist=rectCoordinates[indexMin].distanceSq(translatedSensorOrigin);
        
        for(unsigned int cornerCounter=1;cornerCounter<4;++cornerCounter)
        {
            double dist=rectCoordinates[cornerCounter].distanceSq(translatedSensorOrigin);
            if(dist<minDist)
            {
                minDist=dist;
                indexMin=cornerCounter;
            }
        }
        
        
        //find the two other verices
        unsigned int indexLeft;
        if(indexMin>0)
        {
            indexLeft=indexMin-1;
        }
        else
        {
            indexLeft=3;
        }
        
        unsigned int indexRight;
        if(indexMin<3)
        {
            indexRight=indexMin+1;
        }
        else
        {
            indexRight=0;
        }
        
        //define the edges
        
        RecVector2D leftEdge(rectCoordinates[indexLeft].x-rectCoordinates[indexMin].x,
                             rectCoordinates[indexLeft].y-rectCoordinates[indexMin].y);
        double leftEdgeLength;
        double leftOffset;
        unsigned int startIndexLeftEdge=0;
        unsigned int endIndexLeftEdge=0;
        RecVector2D leftEdgeUnit;
        
        if(fabs(leftEdge.x)<fabs(leftEdge.y))
        {
            //going along the y axis
            leftEdgeLength=fabs(leftEdge.y);
            
            if(leftEdge.y<0)
            {
                //going along the y-axis (backwards)
                if(indexMin!=2)
                {
                    cout << "WRONG leftEdge" << endl;
                    continue;
                }
                startIndexLeftEdge=indexMaxY;
                endIndexLeftEdge=indexMinY;
                leftOffset=maxX;
                leftEdgeUnit=RecVector2D(0,-1);
            }
            else
            {
                //going along the y-axis (forward)
                if(indexMin!=0)
                {
                    cout << "WRONG leftEdge" << endl;
                    continue;
                }
                startIndexLeftEdge=indexMinY;
                endIndexLeftEdge=indexMaxY;
                leftOffset=minX;
                leftEdgeUnit=RecVector2D(0,+1);
            }
        }
        else
        {
            //going along the x axis
            leftEdgeLength=fabs(leftEdge.x);
            if(leftEdge.x<0)
            {
                //going along the x-axis (backwards)
                if(indexMin!=1)
                {
                    cout << "WRONG leftEdge" << endl;
                    continue;
                }
                startIndexLeftEdge=indexMaxX;
                endIndexLeftEdge=indexMinX;
                leftOffset=minY;
                leftEdgeUnit=RecVector2D(-1,0);                
            }
            else
            {
                //going along the x-axis (forward)
                if(indexMin!=3)
                {
                    cout << "WRONG leftEdge" << endl;
                    continue;
                }
                startIndexLeftEdge=indexMinX;
                endIndexLeftEdge=indexMaxX;
                leftOffset=maxY;
                leftEdgeUnit=RecVector2D(+1,0);
            }
        }
        
        
        RecVector2D rightEdge(rectCoordinates[indexRight].x-rectCoordinates[indexMin].x,
                              rectCoordinates[indexRight].y-rectCoordinates[indexMin].y);
        
        unsigned int startIndexRightEdge=0;
        unsigned int endIndexRightEdge=0;
        double rightEdgeLength;
        double rightOffset;
        RecVector2D rightEdgeUnit;
        if(fabs(rightEdge.x)<fabs(rightEdge.y))
        {
            rightEdgeLength=fabs(rightEdge.y);
            if(rightEdge.y<0)
            {
                //going along the y-axis (backwards)
                if(indexMin!=3)
                {
                    cout << "WRONG rightEdge" << endl;
                    continue;
                }
                startIndexRightEdge=indexMaxY;
                endIndexRightEdge=indexMinY;
                rightOffset=minX;
                rightEdgeUnit=RecVector2D(0,-1);
            }
            else
            {
                //going along the y-axis (forward)
                if(indexMin!=1)
                {
                    cout << "WRONG rightEdge" << endl;
                    continue;
                }
                startIndexRightEdge=indexMinY;
                endIndexRightEdge=indexMaxY;
                rightOffset=maxX;
                rightEdgeUnit=RecVector2D(0,+1);
            }
        }
        else
        {
            rightEdgeLength=fabs(rightEdge.x);
            if(rightEdge.x<0)
            {
                //going along the x-axis (backwards)
                if(indexMin!=2)
                {
                    cout << "WRONG rightEdge" << endl;
                    continue;
                }
                startIndexRightEdge=indexMaxX;
                endIndexRightEdge=indexMinX;
                rightOffset=maxY;
                rightEdgeUnit=RecVector2D(-1,0);
            }
            else
            {
                //going along the x-axis (forward)
                if(indexMin!=0)
                {
                    cout << "WRONG rightEdge" << endl;
                    continue;
                }
                startIndexRightEdge=indexMinX;
                endIndexRightEdge=indexMaxX;
                rightOffset=minY;
                rightEdgeUnit=RecVector2D(+1,0);
            }
        }
        
        
        //get the maximum length
        double maxEdgeLength=0;
        
        if(leftEdgeLength>rightEdgeLength)
        {
            maxEdgeLength=leftEdgeLength;
        }
        else
        {
            maxEdgeLength=rightEdgeLength;
        }
        
        double increment=maxEdgeLength/(double)NUMBER_OF_COST_POINTS_LONGE_EDGE;
        
        //good - now compute the cost
        double cost=0;
        
        
        //cannot use the corner point...
        //no cost calculated for this one...
        
        //first along the left edge...
        {
            //cout << "Along Left edge..." << endl;
            if(fabs(leftEdgeUnit.x)>fabs(leftEdgeUnit.y))
            {
                //cout << "X" << endl;
                cost+=calculateEdgeCost(rectCoordinates[indexMin].x,
                                        leftOffset,
                                        increment,
                                        leftEdgeUnit.x,
                                        -1,
                                        startIndexLeftEdge,
                                        endIndexLeftEdge,
                                        translatedPointsX,
                                        translatedPointsY,
                                        leftEdgeLength);
                
                
            }
            else
            {
                //cout << "Y" << endl;
                cost+=calculateEdgeCost(rectCoordinates[indexMin].y,
                                        leftOffset,
                                        increment,
                                        leftEdgeUnit.y,
                                        -1,
                                        startIndexLeftEdge,
                                        endIndexLeftEdge,
                                        translatedPointsY,
                                        translatedPointsX,
                                        leftEdgeLength);
            }
        }
        
        //then along the right edge...
        {
            //cout << "Along right edge..." << endl;
            if(fabs(rightEdgeUnit.x)>fabs(rightEdgeUnit.y))
            {
                //cout << "X" << endl;
                cost+=calculateEdgeCost(rectCoordinates[indexMin].x,
                                        rightOffset,
                                        increment,
                                        rightEdgeUnit.x,
                                        +1,
                                        startIndexRightEdge,
                                        endIndexRightEdge,
                                        translatedPointsX,
                                        translatedPointsY,
                                        rightEdgeLength);
                
                
            }
            else
            {
                //cout << "Y" << endl;
                cost+=calculateEdgeCost(rectCoordinates[indexMin].y,
                                        rightOffset,
                                        increment,
                                        rightEdgeUnit.y,
                                        +1,
                                        startIndexRightEdge,
                                        endIndexRightEdge,
                                        translatedPointsY,
                                        translatedPointsX,
                                        rightEdgeLength);
            }
        }
        
        //ok - update the minimum cost...
        if(cost<minCost)
        {
            minCost=cost;
            bestAngle=angle;
        }
    }

    
    //ok - we have the best angle now.
    //cout << "bestAngle=" << bestAngle << endl;
    resultCoordinates=fitRect(hullPoints,bestAngle,centroid);
    return true;
}

    

#endif //ifndef _RECTANGLEFITTER_DEF_H_

