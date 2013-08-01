/**
 * @file Segmentation.h
 * @author: Bruce (Haiyue) Li, (haiyuel@andrew.cmu.edu)
 *
 * @attention Copyright (c) 2013
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */



#ifndef _SEGMENTATION_H_
#define _SEGMENTATION_H_


//STL includes
#include <iostream>
#include <fstream>
#include <utility>
#include <algorithm>
#include <set>
#include <vector>


//Infrastructure includes
#include <task/Task.h>


//Input interfaces
#include <interfaces/TransformSource/Abstract.h>
#include <interfaces/VehicleState/Interpolated/Abstract.h>
#include <interfaces/RoadWorldModel/Input/Abstract.h>
#include <interfaces/IBEOScan/Input/Abstract.h>


//Output Interfaces
#include <interfaces/ScrollingByteMap/Output/Abstract.h>
#include <interfaces/IBEOSegmentation/Output/Abstract.h>


//Data structure
#include <recgeometry/recGeometry.h>
#include <ScrollingMap/HysteresisScrollingMap.h>
#include <TimeStamp/TimeStamp.h>
#include <IBEOSegmentationTypes/IBEOSegmentationTypes.h>


//Macro
#define MAXSEGMENTS 5000           // ( (mapSize_m_/cellSize_m_)^2/2 )
#define MAXCELLSINMAP (mapSize_m_/cellSize_m_)*(mapSize_m_/cellSize_m_)


//Namespace
using namespace std;
using namespace roadWorldModel;
using namespace task;


//Class
/*** This allows us to create a set<RecPoint3D, ltpt3D> ***/
struct ltpt3D
{
    bool operator()(const RecPoint3D & s1, const RecPoint3D & s2) const
    {
        if (s1.x < s2.x)
            return true;

        if ((s1.x == s2.x) && (s1.y < s2.y))
            return true;

        if ((s1.x == s2.x) && (s1.y == s2.y) && (s1.z < s2.z))
            return true;

        return false;
    }
};


class Segmentation: public Task
{
public:
    Segmentation(const string &taskName);
    virtual ~Segmentation(void);

    virtual bool initialize(void);
    virtual bool executive(void);
    virtual void cleanup(void);

private:
    TransformSource* transformSource_;              // Transform source so we can transform IBEO points to global coordinates
    VehicleStateInterpolated* vehicleStateInput_;   // Vehicle State Input

    IBEOScanInput* ibeoInput_;                      // Input of IBEO points.
    RoadWorldModelInput* roadWorldModelInput_;

    ScrollingByteMapOutput* segmentationMapOutput_; // Output for maps.
    IBEOSegmentationOutput* pointsSegmentsOutput_;  // Output for pointsSegments

    RoadWorldModel currentRoadWorldModel_;
    VehicleState curVehicleState_;                  // Current vehicle state.

    HysteresisScrollingByteMap segmentationMap_;    // i.e. HysteresisScrollingMap<unsigned char>, map to output
    ScrollingByteMap tempSBMap_;         // i.e. ScrollingMap<unsigned char>, use for getCellCenter() and recording segments number

    IBEO::Scan scan;                                // (IBEO::)class Scan: public std::vector<ScanPoint>, public csvable
    unsigned int maxMessages_;
    double mapSize_m_;
    double cellSize_m_;

    bool SpuriousNoisefilterState_;
    int frameLastingThreshold_;

    bool enablePublishSegmentationMap_;
    bool enablePublishPointsSegments_;

    int CCSWay_;                                    // connected component search way
    int CellOccupiedThreshold_;                     // cell occupied threshold
    int maxLabel;                                   // number of segments (contained empty segments)
    int emptySegments;                              // number of empty segments
    int emptySegments_[100];                        // Debug: record the emptySegments' label
    int minLabelSegments_[100];                     // Debug: record the combined minLabel Segments' label


    int spaceValidPointsCount;                      // count the spaceValid Points
    RecPoint2D boundaryMIN_;                        // store the boundary of the segmentationMap

    set<RecPoint3D,ltpt3D> cellCentrePointsSet_Cur; // in each cycle, the current cellCentrePointsSet
    set<RecPoint3D,ltpt3D> ibeoSet;                 // Destination set, used for storing the same set among
                                                    // frameLastingThreshold_'s points sets

    ofstream Aout, Bout, Cout, Dout, Eout, Fout;    // create Aout: Segm_cellCentrePointsSet_.txt
                                                    //        Bout: Segm_mapBoundary_.txt
                                                    //        Cout: Segm_mapCellValue_.txt
                                                    //        Dout: Segm_TEMPmapCellValue_.txt
                                                    //        Eout: Segm_segLabel_.txt
                                                    //        Fout: Segm_cellIndex_.txt
    ofstream Gout;                                  //        Gout: Segm_segLabelAdjustCheck_.txt

    vector< vector<RecPoint3D> > segLabel;          //*a vector array, each one(segLabel[1], segLabel[2]...)
                                                    // contains several cellcenter points
    vector< vector<DATA_POINT> > cellIndex;         //*each cell(cellIndex[0], cellIndex[1]...) contains
                                                    // RawPoints corresponding to the cellCentre;
                                                    // It starts from (Row, Column)=(0,0)
    vector<SEGMENT> pointsSegments;                 //*each one(pointsSegments[0], pointsSegments[1]...) is a SEGMENT

    bool Debug_ScanPoints_;


private:
    void checkInput(IBEOScanInput * ibeoInput__);
    bool centerMap(HysteresisScrollingByteMap & map, VehicleStateInterpolated * vehIn, boost::posix_time::ptime operTime);

    template <typename T>
    void setCellValue(T & map, const RecPoint3D & pt, const int & value);
    //void setCellValue(ScrollingByteMap & map, const RecPoint3D & pt, const int & value);
    int getCellIndex(const RecPoint3D & ccPT);

    void parametersReset();
    void generateCellCentrePointsSet();
    void generateSegmentationMap();
    void cellSweepSegment();

    bool neighbourCheck(const RecPoint3D & dstPT);
    int labelGet(const RecPoint3D & dstPT);
    void labelSet(const RecPoint3D & dstPT, const int & value);

    void segmentsCombine(const int & segLabelMin, const int & segLabelMax);
    void setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT);
    void setWithCombineCheck(const RecPoint3D & infoPT1, const RecPoint3D &infoPT2);
    void createNewSegment(const RecPoint3D & dstPT);
    void segLabelAdjust();
    void createPointsSegmentsFromCells();


    void setMapValue();


    void LogCellCentrePointsCheck(const set<RecPoint3D,ltpt3D> & ccp_);
    void LogMapBoundaryCheck(const RecPoint2D & bdMIN_);
    //void LogMapCellValuesCheck();
    void LogSegLabelCheck(const int & nonEmptySegments);
    void LogCellIndexCheck();
    void LogSegLabelAdjustCheck(const int & adjustTimes_, const int & maxLabelBefore);




};





#endif  //#ifndef _SEGMENTATION_H_
