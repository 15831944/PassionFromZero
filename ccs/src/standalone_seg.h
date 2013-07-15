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


#define FAILOPENSOURCEFILE -1
#define FAILOPENSOURCESETCHECKFILE -2
#define POINTBOUNDARYSTATEERROR -3
#define FAILOPENSEGLABELCHECKFILE -4

#define MAXSEGMENTS 5000           // ( (mapSize_m_/cellSize_m_)^2/2 )


class CCS
{
protected:
    set<RecPoint3D, ltpt3D> sourceSet;

    HysteresisScrollingByteMap segmentationMap_;    // i.e. HysteresisScrollingMap<unsigned char>, Cost map output
    ScrollingByteMap tempScrollingByteMap_;         // i.e. ScrollingMap<unsigned char>, use for getCellCenter(), record segments number
//    HysteresisScrollingByteMap recordHSBM_;          // temporarily store the label, for judging whether the cell is occupied
    ScrollingByteMap recordSBM_;                     // temporarily store the label, for judging whether the cell is occupied

    double mapSize_m_;
    double cellSize_m_;
    int CCSWay_;                                    // connected component search way
//    int CellOccupiedThreshold_;                     // cell occupied threshold ( >(*this)'s points inside one cell means occupied )
    int maxLabel;                                   // number of segments

    vector< vector<RecPoint3D> > segLabel;          // a vector array, each one(segLabel[1], segLabel[2]...)
                                                    // contains several cellcenter points
    bool Debug_ScanPoints_;


public:
    CCS();
    virtual ~CCS() {};

    void createSourceSet(const char* filename);
    void printSourceSet();
    void setCellValue(ScrollingByteMap & map, const RecPoint3D & pt, const int & value);
    void parametersReset();

    void generateSegmentationMap();
    void connectedComponentSearch();

    bool neighbourCheck(const RecPoint3D & dstPT, bool & isValid);
    int labelGet(const RecPoint3D & dstPT, bool & isValid);
    void labelSet(const RecPoint3D & dstPT, const int & value, bool & isValid);

    void segmentsCombine(const int & segLabelMin, const int & segLabelMax, bool & isValid);
    void setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT, bool & isValid);
    void setWithCombineCheck(const RecPoint3D & infoPT1, const RecPoint3D &infoPT2, bool & isValid);
    void createNewSegment(const RecPoint3D & dstPT, bool & isValid);
    void segLabelAdjust();


    void printSegLabel();

};




#endif // #ifndef _STANDALONE_SEG_H_
