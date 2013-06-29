/*
*
*
*
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
     tempScrollingByteMap_(),
     recordHSBM_(),
     recordSBM_(),
     maxMessages_(10),
     mapSize_m_(100.0),
     cellSize_m_(0.25),
     enablePublishSegmentationMap_(false),
     CCSWay_(8),
     CellOccupiedThreshold_(2),
     maxLabel(0),
     segLabel(NULL),
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

    // Get input interfaces
    GET_READONLY_INTERFACE(transformSource_, TransformSource, "TransformSource", task::GetIntfFailOnErrors);
    GET_READONLY_INTERFACE(vehicleStateInput_, VehicleStateInterpolated, "VehicleState", task::GetIntfFailOnErrors);
    GET_READONLY_INTERFACE(roadWorldModelInput_, RoadWorldModelInput, "RoadWorldModelInput", task::GetIntfFailOnErrors);
    GET_READONLY_INTERFACE(ibeoInput_, IBEOScanInput, "IBEOScanInput", task::GetIntfFailOnErrors);
    
    //Get output interfaces
    GET_WRITEONLY_INTERFACE(segmentationMapOutput_, ScrollingByteMapOutput, "SegmentationMapOutput", task::GetIntfFailOnErrors);

    //Get task parameters we need
    configSection.get("Max Messages", maxMessages_, 10);
    configSection.get("mapSize_m", mapSize_m_, 100);
    configSection.get("cellSize_m", cellSize_m_, 0.25);
    configSection.get("hysteresis_s", hysteresis_s_, 0.1);  // definition of hysteresis_s_ is above, not in header file

    configSection.get("EnablePublishSegmentationMap", enablePublishSegmentationMap_, false);
    configSection.get("CCSWay", CCSWay_, 8);                                // default search way: 8-way
    configSection.get("CellOccupiedThreshold", CellOccupiedThreshold_, 2);  // default threshold: 2 points


    //For Debug
    configSection.get("Debug_ScanPoints",Debug_ScanPoints_,false);

    //Double Check
    if (cellSize_m_ <= 0.0)
    {
        // NOTE : if the cellsize is not greater than 0 then the task will crash with a floating point exception
        logger_.log_error("ERROR : cellSize_m_ must be greater than 0!!!  Current value is %f", cellSize_m_);
        return false;
    }
    
    //Initialize Map
    /*** bool RC(row-column scrolling)->true, TT(unsigned char) voidedValue->0 ***/
    segmentationMap_ = HysteresisScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    segmentationMap_.setLifeTime(hysteresis_s_);
    segmentationMap_.setUseShallowCopy();

    tempScrollingByteMap_ = ScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    tempScrollingByteMap_.setUseShallowCopy();

    recordHSBM_ = HysteresisScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    recordHSBM_.setLifeTime(hysteresis_s_);
    recordHSBM_.setUseShallowCopy();

    recordSBM_ = ScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    //recordSBM_.setUseShallowCopy();


    //Initialize vector array
    segLabel = vector< vector<RecPoint3D> >(MAXSEGMENTS);


    //Debug
    if (Debug_ScanPoints_)
    {
        // Segm_allPointsSet_.txt reset to 0 length
        Aout.open("./src/perception/SensorFuser/Segm_allPointsSet_.txt", ios::out|ios::trunc);     // if Segm_allPointsSet_.txt already exists, delete it first
        Aout << "RESET----------------------------------------------------------------------" << endl;
        Aout.close();

        // Segm_mapBoundary_.txt reset to 0 length
        Bout.open("./src/perception/SensorFuser/Segm_mapBoundary_.txt", ios::out|ios::trunc);      // if Segm_mapBoundary_.txt already exists, delete it first
	    Bout << "RESET----------------------------------------------------------------------" << endl;
	    Bout.close();

        // Segm_mapCellValue_.txt reset to 0 length
        Cout.open("./src/perception/SensorFuser/Segm_mapCellValue_.txt", ios::out|ios::trunc);     // if Segm_mapCellValue_.txt already exists, delete it first
	    Cout << "RESET----------------------------------------------------------------------" << endl;
	    Cout.close();

        // Segm_TEMPmapCellValue_.txt reset to 0 length
        Dout.open("./src/perception/SensorFuser/Segm_TEMPmapCellValue_.txt", ios::out|ios::trunc); // if Segm_TEMPmapCellCalue_.txt already exists, delete it first
        Dout << "RESET----------------------------------------------------------------------" << endl;
        Dout.close();

        // Segm_segLabel_.txt reset to 0 length
        Eout.open("./src/perception/SensorFuser/Segm_segLabel_.txt", ios::out|ios::trunc);         // if Segm_segLabel_.txt already exists, delete it first
        Eout << "RESET----------------------------------------------------------------------" << endl;
        Eout.close();

        // Segm_segCombine_.txt reset to 0 length
        Fout.open("./src/perception/SensorFuser/Segm_segCombine_.txt", ios::out|ios::trunc);         // if Segm_segCombine_.txt already exists, delete it first
        Fout << "RESET----------------------------------------------------------------------" << endl;
        Fout.close();

    }


    return true;
}


bool Segmentation::executive(void)
{
    logger_.log_debug("Segmentation BEGIN--------------------------------------------------------");

    // variables reset to 0
    parametersReset();

    // attain the RoadWorldModel
    while(roadWorldModelInput_->get(currentRoadWorldModel_));
    
    // drain the 'IBEOScanInput' queue properly
    checkInput(ibeoInput_);

    // generate a set that contains all the 'spaceValid' points (actually it is the cellCenter!)
    generateAllPointsSet();    
    
    // generate the segmentation map from the above set
    generateSegmentationMap();
    
    return true;
}


void Segmentation::cleanup(void)
{


}


/* Reset variables */
void Segmentation::parametersReset()
{
    allPointsSet_Cur.clear();

    for(int i=0; i<=maxLabel; ++i)  // segLabel[0] has Nothing!
    {
        segLabel[i].clear();
    }

    segLabel.clear();
    maxLabel=0;

    // reset maps' cell value!!!

    tempScrollingByteMap_.clear();
    recordHSBM_.getDataMap().clear();
    recordSBM_.clear();

}


/* Drain The Message Queue */
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


void Segmentation::generateAllPointsSet()
{
    unsigned int messages(0); // number of messages in queue
    bool isValid(false);      // store the return value of segmentationMap_.isSpaceValid()
    RecPose3D totalPose;      // x,y,z,rot1,rot2,rot3,angleOrder(default:ZYX); indirectly store the transformation matrix
    RecPoint3D pt;            // x,y,z; distance(another point); store the global coordinate of a point after transformation

    RecPoint3D cellCenterPoint;

    IBEO::Scan scan; // IBEO::Scan
    ptime pointTime; // timestamp

    for (messages=0; messages < maxMessages_ && ibeoInput_->get(scan); ++messages)
    {
        // Touch corners to get the output map centered on the vehicle.
        if (!centerMap(segmentationMap_, vehicleStateInput_))
        {
            logger_.log_warn("Center Segmentation Map Failed!");
        }

        // read scan points
        for(IBEO::Scan::iterator ii(scan.begin()); ii != scan.end() ; ++ii)
        {
            if(ii->flags != IBEO::Ok  ) // flag shows whether the point is valid (not ground, clustered)
            {
                continue;
            }

            pointTime = scan.header.scanStart; // + ii->offsetIntoScan;

            if(!vehicleStateInput_->getAtIsAcceptable(pointTime, curVehicleState_))
            {
                logger_.log_warn("No valid vehicle state at time %Lf (current is %Lf)",
                                 convertToSeconds(pointTime),
                                 convertToSeconds(getSystemTime())
                                 );
                continue;
            }

            /*
            if (false)
            {
                cout<< curVehicleState_.pose.rot1 << ";   " << curVehicleState_.pose.rot2 << ";   " << curVehicleState_.pose.rot3 <<endl;
                cout<< curVehicleState_.velocity.rot1 << ":  " << curVehicleState_.velocity.rot2 << ":  " << curVehicleState_.velocity.rot3 <<endl;
                cout<< curVehicleState_.velocity.x << ",  " << curVehicleState_.velocity.y << ",  " << curVehicleState_.velocity.z <<endl;
            }
            */

            transformSource_->getPoseAt("IBEO", curVehicleState_.pose, totalPose);

            //Transform point to Global Coordinates
            pt.x = ii->x;
            pt.y = ii->y;
            pt.z = ii->z;
            //logger_.log_info("Raw scan point-->pt.x: %f, pt.y: %f, pt.z: %f",pt.x,pt.y,pt.z);
            pt = totalPose*pt;

            isValid = segmentationMap_.isSpaceValid(pt);  // judge whether the pt is within the map

            if(isValid)
            {
                tempScrollingByteMap_.getCellCenter(pt, cellCenterPoint);
                allPointsSet_Cur.insert(cellCenterPoint);  // cellCenterPoint like: (XXXXXXX.X,XXXXXX.X,0.0)

                /*******************  TO DO  *******************/
                /**** Find which map should be set as the record map ****/
                /**** Because the hysteresis may cause problem       ****/
                //setCellValue(recordHSBM_, cellCenterPoint, 255);
                setCellValue(recordSBM_, cellCenterPoint, 255);



                /*
                recordHSBM_.set(cellCenterPoint, (unsigned char)255);
                */


            }
        } // for IBEO::Scan::iterator

        if (Debug_ScanPoints_)
        {
            LogAllScanPointsCheck(allPointsSet_Cur);
            //LogMapCellValuesCheck(segmentationMap_, tempScrollingByteMap_, allPointsSet_Cur);
        }
    } // for message


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


void Segmentation::generateSegmentationMap()
{
    //Segmentation
    connectedComponentSearch();

    if (Debug_ScanPoints_)
    {
        LogSegLabelCheck(tempScrollingByteMap_);
    }

    //Set cell value according to the info from Segmentation
    setMapValue();



    //Publish:
    segmentationMap_.clearByTime();
    segmentationMapOutput_->publish(segmentationMap_.getDataMap());
    logger_.log_debug("segmentationMap_ Publish.");

}


void Segmentation::connectedComponentSearch()
{
    // find boundary of the map
    tempScrollingByteMap_ = segmentationMap_.getDataMap();
    RecAxisAlignedBox2D MapBoundary_ = tempScrollingByteMap_.getBounds();
    RecPoint2D boundaryMIN_ = MapBoundary_.getMinPoint();

    if (Debug_ScanPoints_)
    {
        LogMapBoundaryCheck(boundaryMIN_);
    }

    // algorithm
    RecPoint3D leftPT, downPT, leftdownPT, rightdownPT;
    bool leftState(false), downState(false), leftdownState(false), rightdownState(false);
    short boundaryState=0; // 0: not boundary; 1: only bottom; 2: only leftside 3: only rightside
    int loopCounts=0;      // record looptimes
    bool isValid(false);   // hsbm.get(..., isValid) -> both isSpaceValid and isTimeValid (include checkTimeStamp(time, lifetime_) )
                           // dataMap_.get(..., isValid)->isSpaceValid
                           // timeMap_.get(..., isValid)->isTimeValid (not include checkTimeStamp(time, lifetime_) )

    for(set<RecPoint3D, ltpt3D>::const_iterator ii=allPointsSet_Cur.begin();
        ii != allPointsSet_Cur.end();
        ++ii)
    {
        loopCounts++;     // record point number
        boundaryState=0;  // reset for every point

        downPT.x = (*ii).x - cellSize_m_;      downPT.y = (*ii).y;
        leftPT.x = (*ii).x;                    leftPT.y = (*ii).y - cellSize_m_;
        leftdownPT.x  = (*ii).x - cellSize_m_; leftdownPT.y  = (*ii).y - cellSize_m_;
        rightdownPT.x = (*ii).x - cellSize_m_; rightdownPT.y = (*ii).y + cellSize_m_;

        if (loopCounts == 1)  // look at the first point in the set
        {
            if ( downPT.x < boundaryMIN_.x && leftPT.y < boundaryMIN_.y )  // this is in the leftbottom corner!
            {
                createNewSegment(*ii, isValid);
                continue;
            }
        }

        if ( downPT.x < boundaryMIN_.x )       // (*ii) is at the bottom
            boundaryState=1;
        else if ( leftPT.y < boundaryMIN_.y )  // (*ii) is at the leftside
            boundaryState=2;
        else if ( rightdownPT.y > (boundaryMIN_.y + mapSize_m_)) // // (*ii) is at the rightside
            boundaryState=3;


        if (boundaryState==0)
        {
            // check left
            leftState = neighbourCheck(leftPT, isValid);
            if (leftState)
                setWithoutCombineCheck(*ii, leftPT, isValid);

            // check down
            downState = neighbourCheck(downPT, isValid);
            if (downState)
            {
                if (!leftState)
                    setWithoutCombineCheck(*ii, downPT, isValid);
                else
                    setWithCombineCheck(*ii, leftPT, downPT, isValid);
            }

            // whether ckeck leftdown corner and rightdown corner
            if (CCSWay_ == 8)
            {    
                // check leftdown
                leftdownState = neighbourCheck(leftdownPT, isValid);
                if (leftdownState)
                {
                    if (!leftState && !downState)  // left cell and down cell are all empty
                        setWithoutCombineCheck(*ii, leftdownPT, isValid);
                }

                // check rightdown
                rightdownState = neighbourCheck(rightdownPT, isValid);
                if ( (boundaryState != 3) && rightdownState ) // not at the rightside
                {
                    if (!downState)
                    {
                        // left and leftdown, at least one cell is occupied, we should segmentsCombine()
                        if (leftState)
                            setWithCombineCheck(*ii, leftPT, rightdownPT, isValid);
                        else if (leftdownState)
                            setWithCombineCheck(*ii, leftdownPT, rightdownPT, isValid);
                    }
                }

                if (!leftState && !downState && !leftdownState && !rightdownState)
                    createNewSegment(*ii, isValid);

                continue;
            }  // if (CCSWay_ == 8)

            // if it goes here, indicate: CCSWay_ != 8
            if (!leftState && !downState)
                createNewSegment(*ii, isValid);
        }
        else if (boundaryState==1)  //only bottom//
        {
            // only check left
            leftState = neighbourCheck(leftPT, isValid);
            if (leftState)
                setWithoutCombineCheck(*ii, leftPT, isValid);
            else
                createNewSegment(*ii, isValid);
        }
        else if (boundaryState==2)  //only leftside//
        {
            // only check down
            downState = neighbourCheck(downPT, isValid);
            if (downState)
                setWithoutCombineCheck(*ii, downPT, isValid);

            // whether ckeck rightdown corner
            if (CCSWay_ ==8)
            {
                // check rightdown
                rightdownState = neighbourCheck(rightdownPT, isValid);
                if (rightdownState)
                {
                    if (!downState)
                        setWithoutCombineCheck(*ii, rightdownPT, isValid);
                }
                else
                {
                    if (!downState)
                        createNewSegment(*ii, isValid);
                }

                continue;
            }

            // if it goes here, indicate: CCSWay_ != 8
            if (!downState)
                createNewSegment(*ii, isValid);
        }
        else  // shouldn't go here
        {
            logger_.log_warn("Exception! point boundaryState: %f, %f; loopCounts:%d, point.x:%f, point.y:%f",
                             boundaryMIN_.x, boundaryMIN_.y, loopCounts, (*ii).x, (*ii).y);
        }

    } // for(set<RecPoint3D, ltpt3D>::const_iterator ...)


    /************** TO DO ****************/
    // segLabelAdjust();

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
            /**** TO DO ****/
            /**** 1. Fix the hysteresis bug ****/
            /**** 2. Find a good way to use color (gradually change/various color) to show the segments ****/
            //tempValue = floor((int)recordHSBM_.get(*itr, isValid) * 255.0/maxLabel);
            tempValue = floor((int)recordSBM_.get(*itr, isValid) * 255.0/maxLabel);
            setCellValue(segmentationMap_, *itr, tempValue);
        }
    }

}




/* Check whether the neighbour cell is occupied   */
bool Segmentation::neighbourCheck(const RecPoint3D & dstPT, bool & isValid)
{
    //return ( (int)recordHSBM_.get(dstPT, isValid) == 255 && isValid );  // occupied && valid
    return ( (int)recordSBM_.get(dstPT, isValid) == 255 && isValid );  // occupied && valid
}


/* Get the cell's label  */
int Segmentation::labelGet(const RecPoint3D & dstPT, bool & isValid)
{
    return (int)tempScrollingByteMap_.get(dstPT, isValid);
}


/* Set the cell with the label the same as neighbour's  */
void Segmentation::labelSet(const RecPoint3D & dstPT, const int & value, bool & isValid)
{
    setCellValue(tempScrollingByteMap_, dstPT, value);
}


/* Comebine 2 segments  */
void Segmentation::segmentsCombine(const int & segLabelMin, const int & segLabelMax, bool & isValid)
{

    if (!segLabel[segLabelMax].size())
 //       cout << "find size()==0 !!!segLabelMax: "<< segLabelMax <<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" <<endl;

    for(vector<RecPoint3D>::const_iterator itr = segLabel[segLabelMax].begin();
        itr != segLabel[segLabelMax].end();
        ++itr)
    {
        labelSet(*itr, segLabelMin, isValid);
        segLabel[segLabelMin].push_back(*itr);
    }

    /*
    if (1)
    {
        for(vector<RecPoint3D>::const_iterator itr = segLabel[segLabelMax].begin();
            itr != segLabel[segLabelMax].end();
            ++itr)
        {
            if ((int)labelGet(*itr, isValid) == segLabelMax)
                cout << itr->x<< ","<<itr->y << " -************************************************8"<<endl;

        }

    }
    */

    segLabel[segLabelMax].clear();  // delete the big-label segment
    //cout << "The clear one is segLabelMax: " <<segLabelMax << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<endl;

}


/* Simply set the cell label  */
void Segmentation::setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT, bool & isValid)
{
    int tempLabel = labelGet(infoPT, isValid);
    labelSet(operPT, tempLabel, isValid);
    segLabel[tempLabel].push_back(operPT);
}


/* Set the cell label and Check combine  */
void Segmentation::setWithCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT1, const RecPoint3D & infoPT2, bool & isValid)
{
    int tempLabel2 = labelGet(infoPT1, isValid);
    int tempLabel3 = labelGet(infoPT2, isValid);
    int tempLabel = min(tempLabel2, tempLabel3);
    labelSet(operPT, tempLabel, isValid);
    segLabel[tempLabel].push_back(operPT);

    int tempLabel4 = max(tempLabel2, tempLabel3);
    if ( tempLabel != tempLabel4 )  // two segments should combine!
    {
        //LogSegCombineCheck(operPT, );

        segmentsCombine(tempLabel, tempLabel4, isValid);
        //LogSegCombineCheck(operPT);
    }
}


/* Create a New Segment  */
void Segmentation::createNewSegment(const RecPoint3D & dstPT, bool & isValid)
{
    maxLabel++;
    labelSet(dstPT, maxLabel, isValid);
    segLabel[maxLabel].push_back(dstPT);
}



/************** TO DO **********************/
/* Adjust the segLabel vector array --> delete those EMPTY vectors(segments)  */
void Segmentation::segLabelAdjust()
{

}


/*
 * Paint 0 cost into the map at the locations which define the corner of the map given the current vehicle state.
 * Corners are defined as VehicleState +/- mapSize/2.0.
 * @return true if we succeeded, false if we never managed to get a most recent vehicle state.
 */
bool Segmentation::centerMap(HysteresisScrollingByteMap & map, VehicleStateInterpolated * vehIn)
{
    bool status(false);
    VehicleState currentState;

    // get latest vehicle state
    status = vehIn->getAtIsAcceptable(ptime(pos_infin), currentState);

    if(status)
    {
        RecPoint2D point(currentState.pose.x, currentState.pose.y);
        map.centerAt(point);
    }

    return status;
}



//////////////////////////////////////////////////////////
void Segmentation::LogAllScanPointsCheck(const set<RecPoint3D,ltpt3D> & allSP_)
{
    static int Times_LogAllScanPointsCheck=1;

    Aout.open("./src/perception/SensorFuser/Segm_allPointsSet_.txt", ios::out|ios::app);   // if Segm_allPointsSet_.txt already exists, add behind existing content
    if ( Aout.is_open() )
    {
        //logger_.log_warn("Open Segm_allPointsSet_.txt successfully!");
	
        Aout << "[Segm_allPointsSet_]:" << Times_LogAllScanPointsCheck << "    Size: " << allSP_.size() << " ----------------------------------------" <<endl;
        for (std::set<RecPoint3D,ltpt3D>::const_iterator itr = allSP_.begin();
             itr != allSP_.end();
             ++itr)
        {
            Aout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << endl;   ///////////////////////////////////
        }
        Aout << "------------------------------------------------------------------------------------------" <<endl;
	
        Aout.close();
    }
    else
    {
        logger_.log_warn("Open Segm_allPointsSet_.txt failed!");
    }

    Times_LogAllScanPointsCheck++;

}


void Segmentation::LogMapBoundaryCheck(const RecPoint2D & bdMIN_)
{
    Bout.open("./src/perception/SensorFuser/Segm_mapBoundary_.txt", ios::out|ios::app);   // if Segm_mapBoundary_.txt already exists, add behind existing content
    if ( Bout.is_open() )
    {
        //logger_.log_warn("Open Segm_mapBoundary_.txt successfully!");

        Bout << "[Segm_mapBoundary_]" << "\t" << "MIN point: " << endl;
        Bout << fixed << setprecision(4) << bdMIN_.x << "   " << bdMIN_.y << endl;   ///////////////////////////////////
        Bout << "--------------------------------------------------------------------------------------------" <<endl;

        Bout.close();
    }
    else
    {
        logger_.log_warn("Open Segm_mapBoundary_.txt failed!");
    }
}


void Segmentation::LogMapCellValuesCheck(HysteresisScrollingByteMap & hsbm,
                                         ScrollingByteMap &sbm,
                                         const set<RecPoint3D,ltpt3D> & allSP_)
{
    bool isTimeValid = false;
    boost::posix_time::ptime cellTimeStamp;

    Cout.open("./src/perception/SensorFuser/Segm_mapCellValue_.txt", ios::out|ios::app);   // if Segm_mapCellValue_.txt already exists, add behind existing content
    if ( Cout.is_open() )
    {
        //logger_.log_warn("Open Segm_mapCellValue_.txt successfully!");

        Cout << "[Segm_mapCellValue_]" << "\t" << " ----------------------------------------" <<endl;
        for (std::set<RecPoint3D,ltpt3D>::const_iterator itr = allSP_.begin();
             itr != allSP_.end();
             ++itr)
        {
            Cout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << " "
                 << "cellValue: " << (int)hsbm.get(*itr, cellTimeStamp, isTimeValid) << "   " << cellTimeStamp
                 << endl;   ///////////////////////////////////
        }
        Cout << "------------------------------------------------------------------------------------------" <<endl;

        Cout.close();
    }
    else
    {
        logger_.log_warn("Open Segm_mapCellValue_.txt failed!");
    }


    Dout.open("./src/perception/SensorFuser/Segm_TEMPmapCellValue_.txt", ios::out|ios::app);   // if Segm_TEMPmapCellValue_.txt already exists, add behind existing content
    if ( Dout.is_open() )
    {
        //logger_.log_warn("Open Segm_TEMPmapCellValue_.txt successfully!");

        Dout << "[Segm_TEMPmapCellValue_]" << "\t" << " ----------------------------------------" <<endl;
        for (std::set<RecPoint3D,ltpt3D>::const_iterator itr = allSP_.begin();
             itr != allSP_.end();
             ++itr)
        {
            Dout << fixed << setprecision(4) << itr->x << "	" << itr->y << "	" << itr->z << " "
                 << "cellValue: " << (int)sbm.get(*itr, isTimeValid)
                 << endl;   ///////////////////////////////////
        }
        Dout << "------------------------------------------------------------------------------------------" <<endl;

        Dout.close();
    }
    else
    {
        logger_.log_warn("Open Segm_TEMPmapCellValue_.txt failed!");
    }


}


/************ TO DO *****************/
/* Check whether the two segLabel[] is correct */
void Segmentation::LogSegCombineCheck(ScrollingByteMap & sbm)
{







}



void Segmentation::LogSegLabelCheck(ScrollingByteMap & sbm)
{
    bool isTimeValid = false;
    int cellCounts=0, temp=0;
    static int Times_LogSegLabelCheck=1;

    Eout.open("./src/perception/SensorFuser/Segm_segLabel_.txt", ios::out|ios::app);   // if Segm_segLabel_.txt already exists, add behind existing content
    if ( Eout.is_open() )
    {
        //logger_.log_warn("Open Segm_segLabel_.txt successfully!");

        Eout << "[Segm_segLabel_]: " << Times_LogSegLabelCheck << "   Number of Segments: " << maxLabel << " --------------------------------" <<endl;
        for (int i=1; i<=maxLabel; ++i)  // segLabel[0] is not a segment, but we still want to output it
        {
            temp = segLabel[i].size();

            /*
            //////////////////////////////////////////////////////////////////////////////////////////////////
            for (vector<RecPoint3D>::const_iterator itr=segLabel[i].begin(); itr != segLabel[i].end(); ++itr)
            {
                Eout << fixed << setprecision(4) << itr->x << " " << itr->y << "    " << itr->z << "    " << (int)sbm.get(*itr, isTimeValid)
                     << endl;   ///////////////////////////////////
            }
            //////////////////////////////////////////////////////////////////////////////////////////////////
            */


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
            Eout << "==========================================================" << endl;


            cellCounts += temp;
        }   
        Eout << "---------------------------------------------------- The above total cells: " << cellCounts <<endl;

        Eout.close();
    }
    else
    {
        logger_.log_warn("Open Segm_segLabel_.txt failed!");
    }

    Times_LogSegLabelCheck++;

}













#endif  //#ifndef _SEGMENTATION_CC_




