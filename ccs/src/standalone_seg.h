#ifndef _STANDALONE_SEG_H_
#define _STANDALONE_SEG_H_

//basic includes
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>  // getline();


//STL includes
/////#include <utility>
/////#include <algorithm>
#include <set>
/////#include <vector>
#include "STLneed.h"



using namespace std;


#define FAILOPENSOURCEFILE -1
#define FAILOPENOUTPUTCHECKFILE -2



class CCS
{
protected:
    set<RecPoint3D, ltpt3D> sourceSet;
/*
    HysteresisScrollingByteMap segmentationMap_;    // i.e. HysteresisScrollingMap<unsigned char>, Cost map output
    ScrollingByteMap tempScrollingByteMap_;         // i.e. ScrollingMap<unsigned char>, use for getCellCenter()

    HysteresisScrollingByteMap recordHSBM_;          // temporarily store the label, for judging whether the cell is occupied
    ScrollingByteMap recordSBM_;                     // temporarily store the label, for judging whether the cell is occupied


    unsigned int maxMessages_;
    double mapSize_m_;
    double cellSize_m_;

    bool enablePublishSegmentationMap_;
    int CCSWay_;                                    // connected component search way
    int CellOccupiedThreshold_;                     // cell occupied threshold
    int maxLabel;                                   // number of segments

    set<RecPoint3D,ltpt3D> allPointsSet_Cur;
    ofstream Aout, Bout, Cout, Dout, Eout, Fout;    // create Aout: Segm_allPointsSet_.txt
                                                    //        Bout: Segm_mapBoundary_.txt
                                                    //        Cout: Segm_mapCellValue_.txt
                                                    //        Dout: Segm_TEMPmapCellValue_.txt
                                                    //        Eout: Segm_segLabel_.txt
                                                    //        Fout: Segm_segCombine_.txt

    vector< vector<RecPoint3D> > segLabel;          // a vector array, each one(segLabel[1], segLabel[2]...)
                                                    // contains several cellcenter points

    bool Debug_ScanPoints_;
*/

public:
    CCS() {};
    virtual ~CCS() {};

    void createSourceSet(const char* filename);
    void printSourceSet();






};









#endif // #ifndef _STANDALONE_SEG_H_
