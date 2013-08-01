/**
 * @file Segmentation.cc
 * @author: Bruce (Haiyue) Li, (haiyuel@andrew.cmu.edu)
 *
 * @attention Copyright (c) 2013
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */


#ifndef _SEGMENTATION_CC_
#define _SEGMENTATION_CC_


#include "Segmentation.h"

//Namespace
//using namespace std;  //in Segmentation.h
//using namespace task; //in Segmentation.h
using namespace IBEO;
using namespace boost::posix_time;
using namespace boost;


Task* Task::get(void)  // task::Task
{
    static Segmentation task(string("Segmentation"));
    return &task;
}


Segmentation::Segmentation(const string &taskName)
    :Task(taskName),
     transformSource_(NULL),
     vehicleStateInput_(NULL),
     ibeoInput_(NULL),
     roadWorldModelInput_(NULL), 
     segmentationMapOutput_(NULL),
     currentRoadWorldModel_(),
     curVehicleState_(),
     segmentationMap_(),
     tempSBMap_(),
     maxMessages_(10),
     mapSize_m_(100.0),
     cellSize_m_(0.25),
     enablePublishSegmentationMap_(false),
     enablePublishPointsSegments_(false),
     CCSWay_(8),
     CellOccupiedThreshold_(2),
     maxLabel(0),
     emptySegments(0),
     spaceValidPointsCount(0),
     boundaryMIN_(),
     Debug_ScanPoints_(false)
{
}


Segmentation::~Segmentation(void)
{
}


bool Segmentation::initialize(void)
{
    double hysteresis_s_(0.0);
    ConfigSection & configSection(getTaskConfig());

    //*** Get input interfaces
    GET_READONLY_INTERFACE(transformSource_, TransformSource, "TransformSource", task::GetIntfFailOnErrors);
    GET_READONLY_INTERFACE(vehicleStateInput_, VehicleStateInterpolated, "VehicleState", task::GetIntfFailOnErrors);
    GET_READONLY_INTERFACE(roadWorldModelInput_, RoadWorldModelInput, "RoadWorldModelInput", task::GetIntfFailOnErrors);
    GET_READONLY_INTERFACE(ibeoInput_, IBEOScanInput, "IBEOScanInput", task::GetIntfFailOnErrors);
    
    //*** Get output interfaces
    GET_WRITEONLY_INTERFACE(segmentationMapOutput_, ScrollingByteMapOutput, "SegmentationMapOutput", task::GetIntfFailOnErrors);
    GET_WRITEONLY_INTERFACE(pointsSegmentsOutput_,IBEOSegmentationOutput,"IBEOSegmentationOutput",task::GetIntfFailOnErrors);

    //*** Get task parameters we need
    configSection.get("Max Messages", maxMessages_, 10);
    configSection.get("mapSize_m", mapSize_m_, 100);
    configSection.get("cellSize_m", cellSize_m_, 0.25);
    configSection.get("hysteresis_s", hysteresis_s_, 0.1);  // definition of hysteresis_s_ is above, not in header file

    configSection.get("EnablePointsMultiBufferFilter",SpuriousNoisefilterState_,true);
    configSection.get("frameLastingThreshold",frameLastingThreshold_,2);

    configSection.get("EnablePublishSegmentationMap", enablePublishSegmentationMap_, false);
    configSection.get("CCSWay", CCSWay_, 8);                                // default search way: 8-way
    configSection.get("CellOccupiedThreshold", CellOccupiedThreshold_, 2);  // default threshold: 2 points; THIS IS NOT USED NOW

    configSection.get("EnablePublishPointsSegments", enablePublishPointsSegments_, false);

    //*** For Debug
    configSection.get("Debug_ScanPoints",Debug_ScanPoints_,false);

    //*** Double Check
    if (cellSize_m_ <= 0.0)
    {
        // NOTE : if the cellsize is not greater than 0 then the task will crash with a floating point exception
        logger_.log_error("ERROR : cellSize_m_ must be greater than 0!!!  Current value is %f", cellSize_m_);
        return false;
    }
    
    //*** Initialize Map
    /*** bool RC(row-column scrolling)->true, TT(unsigned char) voidedValue->0 ***/
    segmentationMap_ = HysteresisScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    segmentationMap_.setLifeTime(hysteresis_s_);
    segmentationMap_.setUseShallowCopy();

    tempSBMap_ = ScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    tempSBMap_.setUseShallowCopy();


    //*** Preallocate memory for vector/vector array, for efficiency
    segLabel = vector< vector<RecPoint3D> >(MAXSEGMENTS+1);
    segLabel.clear();
    for (int i=1; i<=MAXSEGMENTS; ++i)
        segLabel[i].reserve(MAXCELLSINMAP/10);

    cellIndex = vector< vector<DATA_POINT> >(MAXCELLSINMAP);
    cellIndex.clear();
    for (int i=0; i<MAXCELLSINMAP; ++i)
        cellIndex[i].reserve(20);

    pointsSegments.reserve(MAXSEGMENTS+1);


    //*** Debug: create new txt check files, avoid manually removing them in terminal each time
    if (Debug_ScanPoints_)
    {
        // Segm_allPointsSet_.txt reset to 0 length
        Aout.open("./src/perception/SensorFuser/Segm_cellCentrePointsSet_.txt", ios::out|ios::trunc); // if Segm_cellCentrePointsSet_.txt already exists, delete it first
        Aout << "RESET----------------------------------------------------------------------" << endl;
        Aout.close();

        // Segm_mapBoundary_.txt reset to 0 length
        Bout.open("./src/perception/SensorFuser/Segm_mapBoundary_.txt", ios::out|ios::trunc);         // if Segm_mapBoundary_.txt already exists, delete it first
	    Bout << "RESET----------------------------------------------------------------------" << endl;
	    Bout.close();

        // Segm_mapCellValue_.txt reset to 0 length
        Cout.open("./src/perception/SensorFuser/Segm_mapCellValue_.txt", ios::out|ios::trunc);        // if Segm_mapCellValue_.txt already exists, delete it first
	    Cout << "RESET----------------------------------------------------------------------" << endl;
	    Cout.close();

        // Segm_TEMPmapCellValue_.txt reset to 0 length
        Dout.open("./src/perception/SensorFuser/Segm_TEMPmapCellValue_.txt", ios::out|ios::trunc);    // if Segm_TEMPmapCellCalue_.txt already exists, delete it first
        Dout << "RESET----------------------------------------------------------------------" << endl;
        Dout.close();

        // Segm_segLabel_.txt reset to 0 length
        Eout.open("./src/perception/SensorFuser/Segm_segLabel_.txt", ios::out|ios::trunc);            // if Segm_segLabel_.txt already exists, delete it first
        Eout << "RESET----------------------------------------------------------------------" << endl;
        Eout.close();

        // Segm_cellIndex_.txt reset to 0 length
        Fout.open("./src/perception/SensorFuser/Segm_cellIndex_.txt", ios::out|ios::trunc);           // if Segm_cellIndex_.txt already exists, delete it first
        Fout << "RESET----------------------------------------------------------------------" << endl;
        Fout.close();

        // Segm_segLabelAdjustCheck_.txt reset to 0 length
        Gout.open("./src/perception/SensorFuser/Segm_segLabelAdjustCheck_.txt", ios::out|ios::trunc); // if Segm_segLabelAdjustCheck_.txt already exists, delete it first
        Gout << "RESET----------------------------------------------------------------------" << endl;
        Gout << "adjustTimes_" << "\t" << "emptySegments" << "\t" << "maxLabelBefore" << "\t" << "maxLabelAfter" << endl;
        Gout.close();
    }


    return true;
}


bool Segmentation::executive(void)
{
    logger_.log_debug("Segmentation BEGIN--------------------------------------------------------");

    //*** variables reset to 0
    parametersReset();

/*
    // attain the RoadWorldModel
    while(roadWorldModelInput_->get(currentRoadWorldModel_));
    
    // drain the 'IBEOScanInput' queue properly
    checkInput(ibeoInput_);
*/

    //*** get the latest scan
    while (ibeoInput_->get(scan));

    //*** generate a set that contains all the 'spaceValid' cellCentrePoints
    generateCellCentrePointsSet();
    
    //*** generate the segmentation map from the above set
    generateSegmentationMap();
    
    return true;
}


void Segmentation::cleanup(void)
{
}


/* Reset variables */
void Segmentation::parametersReset()
{
    //*** clear segments label
    if (maxLabel)
    {
        for(int i=0; i<=maxLabel; ++i)  // (segLabel[0] has Nothing)
            segLabel[i].clear();
    }
//    segLabel.clear();  //segLabel.size() is ALWAYS 0, even segLabel[XXX].size() is NOT 0. So no need to use clear() here
    maxLabel=0;
    emptySegments=0;

    //*** clear cellIndex that contained rawPoints
    if (cellCentrePointsSet_Cur.size())
    {
        for ( set<RecPoint3D, ltpt3D>::const_iterator itr = cellCentrePointsSet_Cur.begin();
              itr != cellCentrePointsSet_Cur.end();
              ++itr )
        {
            cellIndex[getCellIndex(*itr)].clear();
        }
    }
//    cellIndex.clear(); // cellIndex.size() is ALWAYS 0, even cellIndex[XXX].size() is NOT 0. So no need to use clear() here

    //*** clear vector<SEGMENT> pointsSegments
    pointsSegments.clear();

    //*** clear cellCentrePoints Set
    cellCentrePointsSet_Cur.clear();
    ibeoSet.clear();

    //*** reset spaceValidPointsCount
    spaceValidPointsCount=0;

    //*** reset maps' cell value
    tempSBMap_.clear();
}


/* Drain The Message Queue */
/*
void Segmentation::checkInput(IBEOScanInput* ibeoInput__)
{
    long unsigned int queueRemaining(0), queueMax(0), messagesToDrain(0);
    Scan tempScan;  // IBEO::Scan
    if(ibeoInput__->hasChannel())
    {
        ibeoInput__->channel().getQueueProperties(queueRemaining, queueMax);

        logger_.log_debug("Queue has %d messages", queueRemaining);
        if(queueRemaining > maxMessages_)
        {
            messagesToDrain = queueRemaining - maxMessages_;
            logger_.log_info("Input has %d messages, we only read %d in one pass. Draining %d messages to keep up.",
                             queueRemaining, queueMax, messagesToDrain);

            for(long unsigned int ii(0); ii < messagesToDrain; ++ii)
            {
                ibeoInput__->get(tempScan);
            }
        }
    }
}
*/


void Segmentation::generateCellCentrePointsSet()
{
    RecPose3D totalPose;      // x,y,z,rot1,rot2,rot3,angleOrder(default:ZYX); indirectly store the transformation matrix
    RecPoint3D localpt;       // x,y,z; distance(another point); store the local coordinate of a point

    RecPoint3D cellCenterPoint;

    DATA_POINT dataPoint;     // reusable
    int pointsCount=0;

    ptime pointTime = scan.header.scanStart; // + ii->offsetIntoScan; -->timestamp

    // Touch corners to get the output map centered on the vehicle.
    if ( !centerMap(segmentationMap_, vehicleStateInput_, pointTime) )
    {
        logger_.log_warn("Center Segmentation Map Failed!");
        return;
    }

    // Find boundary of the map
    boundaryMIN_ = segmentationMap_.getDataMap().getBounds().getMinPoint();

/////////////////
/*
    //--- This is not accurate, the boundary doesn't locate at a corner of a cell ---//
    // find boundary of the map
    if( vehicleStateInput_->getAtIsAcceptable(ptime(pos_infin), curVehicleState_) )
    {
        RecPoint2D tempPoint(curVehicleState_.pose.x, curVehicleState_.pose.y);
        boundaryMIN_ = tempPoint - RecPoint2D(mapSize_m_/2.0, mapSize_m_/2.0);
    }
    else
    {
        logger_.log_warn("Getting Latest VehicleState Failed! Unable To Attain Map Boundary!");
        return;
    }
*/
/////////////////

    // Get totalPose: the transformation matrix
    transformSource_->getPoseAt("IBEO", curVehicleState_.pose, totalPose);

    // Read scan points
    for(IBEO::Scan::iterator ii = scan.begin(); ii != scan.end(); ++ii)
    {
        if(ii->flags != IBEO::Ok) // flag shows whether the point is valid (not ground, clustered)
            continue;

        pointsCount++;

        localpt.x = ii->x;
        localpt.y = ii->y;
        localpt.z = ii->z;
        //logger_.log_info("Raw scan point-->pt.x: %f, pt.y: %f, pt.z: %f",pt.x,pt.y,pt.z);

        //*** Set DATA_POINT's fields      
        if (localpt.x > 0)
            dataPoint.bearing = atan(localpt.y / localpt.x); // compute dataPoint's bearing (local coordinates)
        else if (localpt.x == 0)
            dataPoint.bearing = localpt.y/fabs(localpt.y)*PI/2;
        else
        {
            if (localpt.y > 0)
                dataPoint.bearing = atan(localpt.y / localpt.x) + PI;
            else if (localpt.y == 0)
                dataPoint.bearing = PI;
            else
                dataPoint.bearing = atan(localpt.y / localpt.x) - PI;
        }

        dataPoint.position = totalPose*localpt;          //
        dataPoint.segmentID = -1;                        // Only thing that needs to be decided LATER!
        dataPoint.timeStamp = pointTime;                 //
        dataPoint.rawID = pointsCount;                   // If it starts from 0, then dataPoint.rawID = pointsCount -1;
        dataPoint.deviceID = ii->deviceId;               //
        dataPoint.layerID =  ii->channel;                //

        if(segmentationMap_.isSpaceValid(dataPoint.position)) // judge whether the pt is within the map
        {
            spaceValidPointsCount++;
            tempSBMap_.getCellCenter(dataPoint.position, cellCenterPoint);

            cellCentrePointsSet_Cur.insert(cellCenterPoint);  // cellCenterPoint like: (XXXXXXX.X, XXXXXX.X, 0.0)
            cellIndex[getCellIndex(cellCenterPoint)].push_back(dataPoint);
        }
    } // for IBEO::Scan::iterator

}


/*
 * Paint 0 cost into the map at the locations which define the corner of the map given the current vehicle state.
 * Corners are defined as VehicleState +/- mapSize/2.0.
 * @return true if we succeeded, false if we never managed to get a most recent vehicle state.
 */
bool Segmentation::centerMap(HysteresisScrollingByteMap & map, VehicleStateInterpolated * vehIn, ptime operTime)
{
    bool status(false);

    // get latest vehicle state
    status = vehIn->getAtIsAcceptable(operTime, curVehicleState_);

    if(status)
    {
        RecPoint2D point(curVehicleState_.pose.x, curVehicleState_.pose.y);
        map.centerAt(point);
    }
    else
    {
        logger_.log_warn("No valid vehicle state at time %Lf (current is %Lf)",
                                     convertToSeconds(operTime),
                                     convertToSeconds(getSystemTime())       );
    }

    return status;
}


int Segmentation::getCellIndex(const RecPoint3D & ccPT)
{
    // the row and col corresponding to the cellCenterPoint
    int row = (int)floor((ccPT.x-boundaryMIN_.x) / cellSize_m_);
    int col = (int)floor((ccPT.y-boundaryMIN_.y) / cellSize_m_);

    return (int)(row*(mapSize_m_/cellSize_m_)+col);
}


void Segmentation::generateSegmentationMap()
{
    //*** Spurious noise filter
    if (SpuriousNoisefilterState_)
    {
        static vector< set<RecPoint3D,ltpt3D> > allPointsSet_Buffer(frameLastingThreshold_); // vector for buffer of each frame's points
        allPointsSet_Buffer[frameLastingThreshold_-1]=cellCentrePointsSet_Cur; // everytime, the last one is the current points

        set<RecPoint3D,ltpt3D> sameSet, tempSet;
        sameSet=allPointsSet_Buffer[0];  //everytime, the sameSet is the first one

        for (int ii=0; ii<frameLastingThreshold_-1; ++ii)
        {
            set_intersection( sameSet.begin(), sameSet.end(),
                              allPointsSet_Buffer[ii+1].begin(), allPointsSet_Buffer[ii+1].end(),
                              inserter(tempSet, tempSet.begin()),
                              ltpt3D() );
            sameSet.clear();  // clear before assignment
            sameSet=tempSet;
            tempSet.clear();
            allPointsSet_Buffer[ii].clear(); // clear before assignment
            allPointsSet_Buffer[ii] = allPointsSet_Buffer[ii+1];
        }

//        ibeoSet.clear();  // we already do it in parameterReset()
        ibeoSet=sameSet;
    }
    else
        ibeoSet=cellCentrePointsSet_Cur;

    if (Debug_ScanPoints_)
        LogCellIndexCheck();


    //*** for the following neighbourCheck();
    for (set<RecPoint3D,ltpt3D>::const_iterator ii = ibeoSet.begin(); ii != ibeoSet.end(); ++ii)
        setCellValue(tempSBMap_, *ii, 255);

    if (Debug_ScanPoints_)
        LogCellCentrePointsCheck(ibeoSet);


    //*** Segmentation
    cellSweepSegment();

//    if (Debug_ScanPoints_)
//        LogSegLabelCheck();  // already do it in createPointsSegmentsFromCells()


    //*** Set cell value according to the info from Segmentation
    setMapValue();


    //*** Publish
    if (enablePublishSegmentationMap_)
    {
        segmentationMap_.clearByTime();
        segmentationMapOutput_->publish(segmentationMap_.getDataMap());
        logger_.log_debug("segmentationMap_ Publish.");
    }

    if (enablePublishPointsSegments_)
    {
        IBEOSegmentation outputMessage;
        outputMessage.timeStamp = scan.header.scanStart;
        outputMessage.segments = pointsSegments;
        //outputMessage.lines = lines_;

        pointsSegmentsOutput_->publish(outputMessage);
        logger_.log_debug("pointsSegments Publish.");
    }
}


template <typename T>
void Segmentation::setCellValue(T & map, const RecPoint3D & pt, const int & value)
{
    map.set(pt, (unsigned char)value);
}


/*
void Segmentation::setCellValue(ScrollingByteMap & map, const RecPoint3D & pt, const int & value)
{
    map.set(pt, (unsigned char)value);
}
*/


void Segmentation::cellSweepSegment()
{
    if (Debug_ScanPoints_)
        LogMapBoundaryCheck(boundaryMIN_);

    //*** variable setup
    RecPoint3D leftPT, downPT, leftdownPT, rightdownPT;
    bool leftState(false), downState(false), leftdownState(false), rightdownState(false);

    //*** check whether the source set is empty
    if (!ibeoSet.size())
    {
        if (Debug_ScanPoints_)
            LogSegLabelCheck(0); // Even ibeoSet is empty, we still want to log info into txt files, for timeStamp synchronization

        return;
    }

    //*** algorithm
    for( set<RecPoint3D, ltpt3D>::const_iterator ii = ibeoSet.begin(); ii != ibeoSet.end(); ++ii )
    {
        // generate leftPT, downPT, leftdownPT and rightdownPT
        leftPT.x = (*ii).x;                    leftPT.y = (*ii).y - cellSize_m_;
        downPT.x = (*ii).x - cellSize_m_;      downPT.y = (*ii).y;
        leftdownPT.x  = (*ii).x - cellSize_m_; leftdownPT.y  = (*ii).y - cellSize_m_;
        rightdownPT.x = (*ii).x - cellSize_m_; rightdownPT.y = (*ii).y + cellSize_m_;

        // check left
        leftState = neighbourCheck(leftPT);
        if (leftState)
            setWithoutCombineCheck(*ii, leftPT);

        // check down
        downState = neighbourCheck(downPT);
        if (downState)
        {
            if (!leftState)
                setWithoutCombineCheck(*ii, downPT);
            else
                setWithCombineCheck(leftPT, downPT);
        }

        // whether ckeck leftdown corner and rightdown corner
        if (CCSWay_ == 8)
        {
            // check leftdown
            leftdownState = neighbourCheck(leftdownPT);
            if (leftdownState)
            {
                if (!leftState && !downState)  // left cell and down cell are all empty
                    setWithoutCombineCheck(*ii, leftdownPT);
            }

            // check rightdown
            rightdownState = neighbourCheck(rightdownPT);
            if (rightdownState)
            {
                if (!downState)
                {
                    // left and leftdown, at least one cell is occupied, we should segmentsCombine()
                    if (leftState)
                        setWithCombineCheck(leftPT, rightdownPT);
                    else if (leftdownState)
                        setWithCombineCheck(leftdownPT, rightdownPT);
                    else
                        setWithoutCombineCheck(*ii, rightdownPT);
                }
            }

            if (!leftState && !downState && !leftdownState && !rightdownState)
                createNewSegment(*ii);

            continue;
        }  // if (CCSWay_ == 8)


        // if it goes here, indicate: CCSWay_ != 8
        if (!leftState && !downState)
            createNewSegment(*ii);

    } // for(set<RecPoint3D, ltpt3D>::const_iterator ...)



    //*** eliminate 0 size segments
    /*** We must run EITHER one of these two (running them both is WRONG!) ***/
//    segLabelAdjust();
    createPointsSegmentsFromCells();
}


/* According to the info of segmentation, set the cells' value of the segmentationMap_ */
void Segmentation::setMapValue()
{
    bool isValid(false);
    int tempValue(0);

    /*
    //Linear mapping between: Cost --> Red Hue
    double a = 50;
    double b = (255.0 - a)/255.0;
    unsigned char red = a + b * (*ll).first;
    */

    for (int i=1; i<=maxLabel; ++i)
    {
        for (vector<RecPoint3D>::const_iterator itr=segLabel[i].begin(); itr!=segLabel[i].end(); ++itr)
        {
            /**** Find a good way to use color (gradually change/various color) to show the segments ****/
            tempValue = floor((int)tempSBMap_.get(*itr, isValid) * 255.0/maxLabel);
            setCellValue(segmentationMap_, *itr, tempValue);
        }
    }

}


/* Check whether the neighbour cell is occupied   */
bool Segmentation::neighbourCheck(const RecPoint3D & dstPT)
{   
    bool isValid = false;   // hsbm.get(..., isValid) -> both isSpaceValid and isTimeValid (include checkTimeStamp(time, lifetime_) )
                            // dataMap_.get(..., isValid)->isSpaceValid
                            // timeMap_.get(..., isValid)->isTimeValid (not include checkTimeStamp(time, lifetime_) )
    return ( (int)tempSBMap_.get(dstPT, isValid) && isValid );  // occupied && spaceValid
}


/* Get the cell's label  */
int Segmentation::labelGet(const RecPoint3D & dstPT)
{
    bool isValid=false;
    return (int)tempSBMap_.get(dstPT, isValid);
}


/* Set the cell with the label the same as neighbour's  */
void Segmentation::labelSet(const RecPoint3D & dstPT, const int & value)
{
    setCellValue(tempSBMap_, dstPT, value);
}


/* Comebine 2 segments  */
void Segmentation::segmentsCombine(const int & segLabelMin, const int & segLabelMax)
{
    for(vector<RecPoint3D>::const_iterator itr = segLabel[segLabelMax].begin();
        itr != segLabel[segLabelMax].end();
        ++itr)
    {
        labelSet(*itr, segLabelMin);
        segLabel[segLabelMin].push_back(*itr);

//        if ( labelGet(*itr) != segLabelMin )
//            cout << "labelSet() FAILED in segmentsCombine()! point's label is: " << labelGet(*itr) << endl;
    }



    segLabel[segLabelMax].clear();  // delete the big-label segment
    //cout << "The clear one is segLabel: " <<segLabelMax << "&&&&&&&&&&&&&"<<endl;
    emptySegments++;
    emptySegments_[emptySegments] = segLabelMax;
    minLabelSegments_[emptySegments] = segLabelMin;
}


/* Simply set the cell label  */
void Segmentation::setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT)
{
    int tempLabel = labelGet(infoPT);
    labelSet(operPT, tempLabel);
    segLabel[tempLabel].push_back(operPT);
}


/* Set the cell label and Check combine  */
void Segmentation::setWithCombineCheck(const RecPoint3D & infoPT1, const RecPoint3D & infoPT2)
{
    int LabelPT1 = labelGet(infoPT1);
    int LabelPT2 = labelGet(infoPT2);
    //if (!LabelPT1 || !LabelPT2) cout << 'fing 0 Label from tempScrollingByteMap_' << endl;

    int Labelmin = min(LabelPT1, LabelPT2);
    int Labelmax = max(LabelPT1, LabelPT2);
    if ( Labelmin != Labelmax )  // two segments should combine!
        segmentsCombine(Labelmin, Labelmax);
}


/* Create a New Segment  */
void Segmentation::createNewSegment(const RecPoint3D & dstPT)
{
    maxLabel+=1;
    labelSet(dstPT, maxLabel);
    segLabel[maxLabel].push_back(dstPT);
}




/* Adjust the segLabel vector array --> delete those EMPTY vectors(segments)  */
/*void Segmentation::segLabelAdjust()
{
    int adjustTimes=0, firstZeroLabel=0;

    ////////////////////////////////////////////////////////////////////////////
    int temp=maxLabel;
    ////////////////////////////////////////////////////////////////////////////

    // adjust the label
    for (int i=1; i<=maxLabel; ++i)
    {
        if (segLabel[i].size() == 0)
        {
            //cout << "segLabel[" << i << "] is zero (after each adjustion)." << endl;
            adjustTimes++;
            if (adjustTimes == 1)
                firstZeroLabel=i;

            for (int j=i; j<maxLabel; ++j)
            {
                //segLabel[j].clear();
                segLabel[j]=segLabel[j+1];
            }
            segLabel[maxLabel].clear();
            maxLabel--;

            --i;  //in case there are 2 Continuous 0 size segments
        }
    }

    //cout << adjustTimes << " adjustments have been done!" << endl;

    ////////////////////////////////////////////////////////////////////////////
    if (Debug_ScanPoints_)
        LogSegLabelAdjustCheck(adjustTimes, temp);
    ////////////////////////////////////////////////////////////////////////////

    // adjust the corresponding cell values
    for (int i=maxLabel; i>=firstZeroLabel; --i)
    {
        for (vector<RecPoint3D>::iterator jj=segLabel[i].begin();
             jj != segLabel[i].end();
             ++jj)
        {
            labelSet(*jj, i);
        }
    }

}*/


/* In this function we do TWO things: */
/* 1. Use another vertor< vector<> > to do Seg Label Adjust (eliminate empty segments)*/
/* 2. Substitute Cells with RawPoints to create PointsSegments */
void Segmentation::createPointsSegmentsFromCells()
{
    //*** check maxLabel
    if (!maxLabel)
    {
        logger_.log_warn("ERROR: maxLabel is 0 before createPointsSegmentsFromCells()");
        return;
    }

    SEGMENT segment;
    int tempCellIndex, pointsCount=0;

    //*** We know that (maxLabel-emptySegments) should equal to the real number of segments ***//
    //*** but, we ought to leave some margin, that's why we use maxLabel                    ***//
    int i=0;
    for (int j=1; j<=maxLabel; ++j)
    {
        if (!segLabel[j].size()) continue;

        i++;
        segment.segmentID=i; // If the SegmentID starts from 0, then segment.segmentID = i-1;
        segment.points.clear();

        for ( vector<RecPoint3D>::const_iterator itr_RecP = segLabel[j].begin();
              itr_RecP != segLabel[j].end();
              ++itr_RecP )
        {
            tempCellIndex = getCellIndex(*itr_RecP);
            for ( vector<DATA_POINT>::iterator itr_DataP = cellIndex[tempCellIndex].begin();
                  itr_DataP != cellIndex[tempCellIndex].end();
                  ++itr_DataP )
            {
                itr_DataP->segmentID = i; // If the SegmentID starts from 0, then itr_DataP->segmentID = i-1;
                segment.points.push_back(*itr_DataP);
                pointsCount++;
            }
        }

        pointsSegments.push_back(segment);
    }

    if (Debug_ScanPoints_)
        LogSegLabelCheck(i);


    if ( i!=(maxLabel-emptySegments) ) // When doing this check, SegmentID should start from 1
        logger_.log_warn("WARN: i:%d, maxLabel:%d, emptySegments:%d. But nothing is exceptional!",i,maxLabel,emptySegments);




    if (Debug_ScanPoints_)
    {
        int pointsCountinibeoSet=0;
        for (set<RecPoint3D, ltpt3D>::const_iterator itr = ibeoSet.begin(); itr != ibeoSet.end(); ++itr)
            pointsCountinibeoSet+=cellIndex[getCellIndex(*itr)].size();

        if ( pointsCount != pointsCountinibeoSet )
            logger_.log_warn("ERROR: pointsCount != pointsCountinibeoSet; pointsCount:%d, pointsCountinibeoSet:%d", pointsCount, pointsCountinibeoSet);
    }

}







/////////////////////////////////////////////////////////
/////// The following are all Log_Check functions ///////
/////////////////////////////////////////////////////////
void Segmentation::LogCellCentrePointsCheck(const set<RecPoint3D,ltpt3D> & ccp_)
{
    static int Times_LogcellCentrePointsCheck=1;

    Aout.open("./src/perception/SensorFuser/Segm_cellCentrePointsSet_.txt", ios::out|ios::app);   // if Segm_cellCentrePointsSet_.txt already exists, add behind existing content
    if ( !Aout.is_open() )
    {
        logger_.log_warn("Open Segm_cellCentrePointsSet_.txt failed!");
        return;
    }

    Aout << "[Segm_cellCentrePointsSet_]:" << Times_LogcellCentrePointsCheck << "    Size: " << ccp_.size() << " ----------------------------------------" <<endl;
    for (std::set<RecPoint3D,ltpt3D>::const_iterator itr = ccp_.begin();
         itr != ccp_.end();
         ++itr)
    {
        Aout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << endl;   ///////////////////////////////////
    }
    Aout << "------------------------------------------------------------------------------------------" <<endl;

    Aout.close();

    Times_LogcellCentrePointsCheck++;
}


void Segmentation::LogMapBoundaryCheck(const RecPoint2D & bdMIN_)
{
    static int Times_LogMapBoundaryCheck=1;

    Bout.open("./src/perception/SensorFuser/Segm_mapBoundary_.txt", ios::out|ios::app);   // if Segm_mapBoundary_.txt already exists, add behind existing content
    if ( !Bout.is_open() )
    {
        logger_.log_warn("Open Segm_mapBoundary_.txt failed!");
        return;
    }

    Bout << "[Segm_mapBoundary_]: " << Times_LogMapBoundaryCheck << "\t" << "MIN point: " << endl;
    Bout << fixed << setprecision(4) << bdMIN_.x << "   " << bdMIN_.y << endl;   ///////////////////////////////////
    Bout << "--------------------------------------------------------------------------------------------" <<endl;

    Bout.close();

    Times_LogMapBoundaryCheck++;
}

/*
void Segmentation::LogMapCellValuesCheck()
{
    bool isTimeValid = false;
    boost::posix_time::ptime cellTimeStamp;

    Cout.open("./src/perception/SensorFuser/Segm_segmentationMapCellValue_.txt", ios::out|ios::app);
    Dout.open("./src/perception/SensorFuser/Segm_tempScrollingByteMapCellValue_.txt", ios::out|ios::app);

    if (!Cout.is_open())
    {
        logger_.log_warn("Open Segm_segmentationMapCellValue_.txt failed!");
        return;
    }

    if (!Dout.is_open())
    {
        logger_.log_warn("Open Segm_tempScrollingByteMapCellValue_.txt failed!");
        return;
    }

    Cout << "[Segm_segmentationMapCellValue_]" << "\t" << " ----------------------------------------" <<endl;
    Dout << "[Segm_tempScrollingByteMapCellValue_]" << "\t" << " ----------------------------------------" <<endl;
    for (std::set<RecPoint3D,ltpt3D>::const_iterator itr = cellCentrePointsSet_Cur.begin(); itr != cellCentrePointsSet_Cur.end(); ++itr)
    {
        Cout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << " "
             << "cellValue: " << (int)segmentationMap_.get(*itr, cellTimeStamp, isTimeValid) << "   " << cellTimeStamp
             << endl;   ///////////////////////////////////
        Dout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << " "
             << "cellValue: " << (int)tempSBMap_.get(*itr, isTimeValid)
             << endl;   ///////////////////////////////////
    }
    Cout << "------------------------------------------------------------------------------------------" <<endl;
    Dout << "------------------------------------------------------------------------------------------" <<endl;

    Cout.close();
    Dout.close();
}
*/


void Segmentation::LogSegLabelCheck(const int & nonEmptySegments)
{
    static int Times_LogSegLabelCheck=1;
    bool isTimeValid = false;
    int cellCounts=0, temp=0, emptySegmentsCount=0;

    Eout.open("./src/perception/SensorFuser/Segm_segLabel_.txt", ios::out|ios::app);   // if Segm_segLabel_.txt already exists, add behind existing content
    if ( !Eout.is_open() )
    {
        logger_.log_warn("Open Segm_segLabel_.txt failed!");
        return;
    }

    Eout << "[Segm_segLabel_]: " << Times_LogSegLabelCheck << "   Number of Segments: " << maxLabel << " --------------------------------" <<endl;
    for (int i=0; i<=maxLabel; ++i)  // segLabel[0] is not a segment, but we may still want to output it
    {
        if ( (temp = segLabel[i].size()) == 0 )
        {
            if (i)  // not count segLabel[0]
                emptySegmentsCount++;

            Eout << "segLabel[" << i << "]: " <<  temp << " cells" << endl;
        }
/*
        Eout << "segLabel[" << i << "]: " <<  temp << " cells" << " =====================================" << endl;
        if (!temp)
        {
            Eout << "XXXXXXXXXX" << endl;
        }
        else
        {
            for (vector<RecPoint3D>::const_iterator itr=segLabel[i].begin(); itr != segLabel[i].end(); ++itr)
            {
                Eout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << " "
                     << "cellValue: " << (int)sbm.get(*itr, isTimeValid)
                     << endl;   ///////////////////////////////////
            }
        }
        Eout << "==========================================================" << endl << endl;
*/
        cellCounts += temp;
    }
    Eout << ":::::::::::::::::::::::::::::::::::::::::::::::::: The above total cells: " << cellCounts
         << ", maxLabel: " << maxLabel
         << ", emptySegmentsCount: " << emptySegmentsCount
         << endl;

    Eout << "::::::::::::::::::::::::::::::: nonEmptySegments( createPointsSegmentsFromCells() ): " << nonEmptySegments
         << ", emptySegments( segmentsCombine() ): " << emptySegments
         << endl;

    for (int i=1; i<=emptySegments; ++i)
    {
        Eout << "segLabel[" << emptySegments_[i] << "] should be 0 cells, but it has:" << segLabel[emptySegments_[i]].size() << "cells"
             << ", corresponding to segLabel[" << minLabelSegments_[i] //<< "]: " << segLabel[minLabelSegments_[i]].size() << "cells"
             << endl;
    }

    if ( emptySegmentsCount != emptySegments || nonEmptySegments != (maxLabel - emptySegmentsCount) )
    {
        Eout << "!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!" << endl
             << "!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!" << endl
             << "!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!" << endl;
    }

    Eout.close();

    Times_LogSegLabelCheck++;
}


void Segmentation::LogCellIndexCheck()
{
    static int Times_LogCellIndexCheck=1;
    int cellCounts=0, rawPointCounts=0, temp=0;

    Fout.open("./src/perception/SensorFuser/Segm_cellIndex_.txt", ios::out|ios::app);   // if Segm_cellIndex_.txt already exists, add behind existing content
    if ( !Fout.is_open() )
    {
        logger_.log_warn("Open Segm_cellIndex_.txt failed!");
        return;
    }

    Fout << "[Segm_cellIndex_]: " << Times_LogCellIndexCheck << " --------------------------------" <<endl;
    for (int i=0; i<MAXCELLSINMAP; ++i)
    {
        if ( (temp=cellIndex[i].size()) == 0) continue;

        cellCounts++;
        rawPointCounts+=temp;

/*        Fout << "cellIndex[" << i << "]: " << temp << " points" << "====================================="  << endl;
        for ( vector<DATA_POINT>::const_iterator itr=cellIndex[i].begin();
              itr != cellIndex[i].end();
              ++itr )
        {
            Fout << fixed << setprecision(4)
                 << itr->position.x << "\t"
                 << itr->position.y << "\t"
                 << itr->position.z << endl;
        }
*/
    }
    Fout << ":::::::::::::::::::::::::::::::::::::::::: Total occupied cells(cellIndex): " << cellCounts
         << ", Total rawPoints within those cells: " << rawPointCounts
         << ", Total spaceValidPoints: " << spaceValidPointsCount
         << endl;
    Fout << "::::::::::::::::::::::::::::::::::::::::::::::Total cellCentrePoints(ibeoSet): " << ibeoSet.size() << endl;

    Fout.close();

    Times_LogCellIndexCheck++;
}


void Segmentation::LogSegLabelAdjustCheck(const int & adjustTimes_, const int & maxLabelBefore)
{
    Gout.open("./src/perception/SensorFuser/Segm_segLabelAdjustCheck_.txt", ios::out|ios::app);
    if (!Gout.is_open())
    {
        logger_.log_warn("Open Segm_segLabelAdjustCheck_.txt failed!");
        return;
    }

    Gout << adjustTimes_ << "\t\t\t\t" << emptySegments << "\t\t\t\t" << maxLabelBefore << "\t\t\t\t" << maxLabel << endl;


    Gout.close();
}







#endif  //#ifndef _SEGMENTATION_CC_




