
#include "standalone_seg.h"
#include <iomanip>


CCS::CCS()
{
    //***initialization variables
    mapSize_m_=100;                                // side length of the map (m)
    cellSize_m_ =0.25;                             // side length of the cell (m)
    CCSWay_ =8;                                    // 4: 4-way; 8: 8-way
    maxLabel =0;                                   // number of segments

    Debug_ScanPoints_ =true;

    //***Initialize Map
    int hysteresis_s_ =0.1;
    /*** bool RC(row-column scrolling)->true, TT(unsigned char) voidedValue->0 ***/
    segmentationMap_ = HysteresisScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    segmentationMap_.setLifeTime(hysteresis_s_);
    segmentationMap_.setUseShallowCopy();

    tempScrollingByteMap_ = ScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    tempScrollingByteMap_.setUseShallowCopy();

    recordSBM_ = ScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    recordSBM_.setUseShallowCopy();

    //***Initialize vector array
    segLabel = vector< vector<RecPoint3D> >(MAXSEGMENTS);
}


void CCS::createSourceSet(const char* filename)
{
    ifstream Ain(filename, ios::in);
    if (!Ain.is_open()) throw FAILOPENSOURCEFILE;

    int i;
    string str;
    RecPoint3D p;

    // read 2 lines first
    for (i=0; i<2; ++i)  // 2, 682, 1368
        getline(Ain, str);

    // then, begin reading points
    while (1)
    {
        getline(Ain, str, '\t');
        if ( !(p.x=atof(str.c_str())) ) break;
        getline(Ain, str, '\t');
        p.y=atof(str.c_str());
        getline(Ain, str, '\n');
        p.z=atof(str.c_str());

        sourceSet.insert(p);
        setCellValue(recordSBM_, p, 255);
        setCellValue(tempScrollingByteMap_, p, 255);
    }

    Ain.close();
}


void CCS::printSourceSet()
{
    ofstream Aout;
    Aout.open("SourceSet_Check.txt", ios::out|ios::trunc);
    if ( !Aout.is_open() ) throw FAILOPENSOURCESETCHECKFILE;

    Aout << "PointsNum: " << sourceSet.size() << " ----------------------------------------" << endl;
    for (set<RecPoint3D,ltpt3D>::const_iterator itr = sourceSet.begin(); itr != sourceSet.end(); ++itr)
        Aout << fixed << setprecision(4) << itr->x << " " << itr->y << "    " << itr->z << endl;

    Aout << "------------------------------------------------------------------------------" << endl;
    Aout.close();
}


void CCS::setCellValue(ScrollingByteMap & map, const RecPoint3D & pt, const int & value)
{
    map.set(pt, (unsigned char)value);
}


void CCS::parametersReset()
{
    sourceSet.clear();

    for(int i=0; i<=maxLabel; ++i)  // segLabel[0] has Nothing!
        segLabel[i].clear();

    segLabel.clear();
    maxLabel=0;

    // reset maps' cell value!!!
    tempScrollingByteMap_.clear();
    recordSBM_.clear();
}


void CCS::generateSegmentationMap()
{
    //Segmentation
    connectedComponentSearch();

/*
    if (Debug_ScanPoints_)
    {
        LogSegLabelCheck(tempScrollingByteMap_);
    }
*/

    //Set cell value according to the info from Segmentation
//    setMapValue();


/*
    //Publish:
    segmentationMap_.clearByTime();
    segmentationMapOutput_->publish(segmentationMap_.getDataMap());
    logger_.log_debug("segmentationMap_ Publish.");
*/
}


void CCS::connectedComponentSearch()
{
    //*** find boundary of the map
//    tempScrollingByteMap_ = segmentationMap_.getDataMap();
//    RecAxisAlignedBox2D MapBoundary_ = tempScrollingByteMap_.getBounds();
//    RecPoint2D boundaryMIN_ = MapBoundary_.getMinPoint();
    RecPoint2D boundaryMIN_(4473885.0000, 589388.2500);

//    if (Debug_ScanPoints_)
//    {
//        LogMapBoundaryCheck(boundaryMIN_);
//    }

    //*** algorithm
    RecPoint3D leftPT, downPT, leftdownPT, rightdownPT;
    bool leftState(false), downState(false), leftdownState(false), rightdownState(false);
    short boundaryState=0; // 0: not boundary; 1: only bottom; 2: only leftside 3: only rightside
    int loopCounts=0;      // record looptimes
    bool isValid(false);   // hsbm.get(..., isValid) -> both isSpaceValid and isTimeValid (include checkTimeStamp(time, lifetime_) )
                           // dataMap_.get(..., isValid)->isSpaceValid
                           // timeMap_.get(..., isValid)->isTimeValid (not include checkTimeStamp(time, lifetime_) )

    for(set<RecPoint3D, ltpt3D>::const_iterator ii=sourceSet.begin();
        ii != sourceSet.end();
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
        else if ( rightdownPT.y > (boundaryMIN_.y + mapSize_m_)) // (*ii) is at the rightside
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
                        else
                            setWithoutCombineCheck(*ii, rightdownPT, isValid);
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
//            logger_.log_warn("Exception! point boundaryState: %f, %f; loopCounts:%d, point.x:%f, point.y:%f",
//                             boundaryMIN_.x, boundaryMIN_.y, loopCounts, (*ii).x, (*ii).y);
            throw POINTBOUNDARYSTATEERROR;
        }

    } //for(set<RecPoint3D, ltpt3D>::const_iterator ...)


    /************** TO DO ****************/
    // segLabelAdjust();

}


/* Check whether the neighbour cell is occupied   */
bool CCS::neighbourCheck(const RecPoint3D & dstPT, bool & isValid)
{
    //return ( (int)recordHSBM_.get(dstPT, isValid) == 255 && isValid );  // occupied && valid
    return ( (int)recordSBM_.get(dstPT, isValid) == 255 && isValid );  // occupied && valid
}


/* Get the cell's label  */
int CCS::labelGet(const RecPoint3D & dstPT, bool & isValid)
{
    return (int)tempScrollingByteMap_.get(dstPT, isValid);
}


/* Set the cell with the label the same as neighbour's  */
void CCS::labelSet(const RecPoint3D & dstPT, const int & value, bool & isValid)
{
    setCellValue(tempScrollingByteMap_, dstPT, value);
}


/* Comebine 2 segments  */
void CCS::segmentsCombine(const int & segLabelMin, const int & segLabelMax, bool & isValid)
{

//    if (!segLabel[segLabelMax].size())
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
void CCS::setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT, bool & isValid)
{
    int tempLabel = labelGet(infoPT, isValid);
    labelSet(operPT, tempLabel, isValid);
    segLabel[tempLabel].push_back(operPT);
}


/* Set the cell label and Check combine  */
void CCS::setWithCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT1, const RecPoint3D & infoPT2, bool & isValid)
{
    int tempLabel2 = labelGet(infoPT1, isValid);
    int tempLabel3 = labelGet(infoPT2, isValid);
    int tempLabel = min(tempLabel2, tempLabel3);
//    labelSet(operPT, tempLabel, isValid);
//    segLabel[tempLabel].push_back(operPT);

    int tempLabel4 = max(tempLabel2, tempLabel3);
    if ( tempLabel != tempLabel4 )  // two segments should combine!
    {
        //LogSegCombineCheck(operPT, );

        segmentsCombine(tempLabel, tempLabel4, isValid);
        //LogSegCombineCheck(operPT);
    }
}


/* Create a New Segment  */
void CCS::createNewSegment(const RecPoint3D & dstPT, bool & isValid)
{
    maxLabel++;
    labelSet(dstPT, maxLabel, isValid);
    segLabel[maxLabel].push_back(dstPT);
}


void CCS::printSegLabel()
{
    bool isTimeValid = false;
    int cellCounts=0, temp=0;
    static int Times_printSegLabel=1;

    ofstream Eout;
    Eout.open("SegLabel_Check.txt", ios::out|ios::trunc);
    if ( Eout.is_open() )
    {
        //logger_.log_warn("Open Segm_segLabel_.txt successfully!");

        Eout << "[Segm_segLabel_]: " << Times_printSegLabel << "   Number of Segments: " << maxLabel << " --------------------------------" <<endl;
        for (int i=0; i<=maxLabel; ++i)  // segLabel[0] is not a segment, but we still want to output it
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
                         << "cellValue: " << (int)tempScrollingByteMap_.get(*itr, isTimeValid)
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
        throw FAILOPENSEGLABELCHECKFILE;
    }

    Times_printSegLabel++;
}


