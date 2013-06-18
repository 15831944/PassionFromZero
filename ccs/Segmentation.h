/*
*
*
*
*/

#ifndef _SEGMENTATION_H_
#define _SEGMENTATION_H_


//STL includes
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
#include <interfaces/MOFDiagnosticStatus/Output/Abstract.h>

#include <recgeometry/recGeometry.h>
#include <MovingObstacle/MovingObstacle.h>
#include <ScrollingMap/HysteresisScrollingMap.h>
#include <TimeStamp/TimeStamp.h>
#include <fstream>
#include <iostream>


//Macro
#define MAXSEGMENTS 5000           // ( (mapSize_m_/cellSize_m_)^2/2 )


//Namespace
using namespace std;
using namespace roadWorldModel;
using namespace task;


//Class
/*** This allows us to create a stl::set<RecPoint3D, ltpt3D> ***/
struct ltpt3D
{
    bool operator()(const RecPoint3D & s1, const RecPoint3D & s2) const
    {
        if (s1.x < s2.x)
        {
            return true;
        }
        if ((s1.x == s2.x) && (s1.y < s2.y))
        {
            return true;
        }
        if ((s1.x == s2.x) && (s1.y == s2.y) && (s1.z < s2.z))
        {
            return true;
        }

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

    //std::vector<IBEOScanInput*> ibeoInput_;       // Input of IBEO points.
    IBEOScanInput* ibeoInput_;                      // Input of IBEO points.
    RoadWorldModelInput* roadWorldModelInput_;

    ScrollingByteMapOutput* segmentationMapOutput_; // Output for maps.

    RoadWorldModel currentRoadWorldModel_;
    VehicleState curVehicleState_;                  // Current vehicle state.

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


private:
    void checkInput(IBEOScanInput * ibeoInput__);
    bool centerMap(HysteresisScrollingByteMap & map, VehicleStateInterpolated * vehIn);

    template <typename T>
    void setCellValue(T & map, const RecPoint3D & pt, const int & value);
    //void setCellValue(ScrollingByteMap & map, const RecPoint3D & pt, const int & value);

    void parametersReset();
    void generateAllPointsSet();
    void generateSegmentationMap();
    void connectedComponentSearch();

    bool neighbourCheck(const RecPoint3D & dstPT, bool & isValid);
    int labelGet(const RecPoint3D & dstPT, bool & isValid);
    void labelSet(const RecPoint3D & dstPT, const int & value, bool & isValid);
    void segmentsCombine(const int & segLabelMin, const int & segLabelMax, bool & isValid);

    void setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT, bool & isValid);
    void setWithCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT1, const RecPoint3D &infoPT2, bool & isValid);
    void createNewSegment(const RecPoint3D & dstPT, bool & isValid);

    /**** TO DO ****/
    void segLabelAdjust();

    void setMapValue();



    void LogAllScanPointsCheck(const set<RecPoint3D,ltpt3D> & allSP_);
    void LogMapCellValuesCheck(HysteresisScrollingByteMap & hsbm,
                               ScrollingByteMap & sbm,
                               const set<RecPoint3D,ltpt3D> & allSP_);
    void LogMapBoundaryCheck(const RecPoint2D & bdMIN_);
    void LogSegCombineCheck(ScrollingByteMap & sbm);
    void LogSegLabelCheck(ScrollingByteMap & sbm);




};





#endif  //#ifndef _SEGMENTATION_H_
