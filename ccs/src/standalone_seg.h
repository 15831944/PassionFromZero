#ifndef _STANDALONE_SEG_H_
#define _STANDALONE_SEG_H_

//basic includes
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>  // getline();


//STL includes
#include "STLneed.h"
#include <set>
#include <vector>
/////#include <utility>
/////#include <algorithm>


//Data Structure includes
#include "../lib/header-only/HysteresisScrollingMap.h"

using namespace std;


#define FAILOPENSOURCESETFILE -1
#define FAILOPENSOURCESETCHECKFILE -2
#define FAILOPENVEHICLEPOSEFILE -3
#define FAILOPENSEGLABELCHECKFILE -4
#define FAILOPENDEBUGFILE -5

#define MAXSEGMENTS 5000           // ( (mapSize_m_/cellSize_m_)^2/2 ), should be less than 2^16-1=65535, cuz we use ScrollingMap<short>
#define MAXCELLSINMAP (mapSize_m_/cellSize_m_)*(mapSize_m_/cellSize_m_)


class CCS
{
protected:
    set<RecPoint3D, ltpt3D> sourceSet;

    HysteresisScrollingByteMap segmentationMap_;    // i.e. HysteresisScrollingMap<unsigned char>, Cost map output
    ScrollingShortMap scrollingShortMap_;           // i.e. ScrollingMap<short>, use for recording segments label

    double mapSize_m_;                              // map side length
    double cellSize_m_;                             // cell side length
    int CCSWay_;                                    // connected component search way
//    int CellOccupiedThreshold_;                     // cell occupied threshold ( >(*this)'s points inside one cell means occupied )
    int maxLabel;                                   // number of segments
    int emptySegments;                              // number of empty segments
    int emptySegments_[100];                        // Debug: record the emptySegments' label
    int minLabelSegments_[100];                     // Debug: record the combined minLabel Segments' label
    RecPoint3D infoPT1_[100];                       // Debug: record the point1 when doing combine check
    RecPoint3D infoPT2_[100];                       // Debug: record the point2 when doing combine check
    int labelBefore1_[100];                         // Debug: record the point1's label BEFORE combining
    int labelBefore2_[100];                         // Debug: record the point2's label BEFORE combining
    int labelAfter1_[100];                          // Debug: record the point1's label AFTER combining
    int labelAfter2_[100];                          // Debug: record the point2's label AFTER combining


    RecPoint2D boundaryMIN_;                        // store the boundary of the segmentationMap_/tempScrollingByteMap_

    vector< vector<RecPoint3D> > segLabel;          // a vector array, each one(segLabel[1], segLabel[2]...)
                                                    // contains several cellcenter points
    bool Debug_ScanPoints_;


public:
    CCS();
    virtual ~CCS() {};

    void parametersReset();
    void centerMap(const char* filename);
    void createSourceSet(const char* filename);
    void printSourceSet();
    void setCellValue(ScrollingShortMap &map, const RecPoint3D & pt, const int & value);


    void cellSweepSegment();

    bool neighbourCheck(const RecPoint3D & dstPT);
    int labelGet(const RecPoint3D & dstPT);
    void labelSet(const RecPoint3D & dstPT, const int & value);

    void segmentsCombine(const int & segLabelMin, const int & segLabelMax);
    void setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT);
    void setWithCombineCheck(const RecPoint3D & infoPT1, const RecPoint3D &infoPT2);
    void createNewSegment(const RecPoint3D & dstPT);
    void segLabelAdjust();


    void printSegLabel();

};




#endif // #ifndef _STANDALONE_SEG_H_
