
#include "standalone_seg.h"
#include <iomanip>


CCS::CCS()//:boundaryMIN_()
{
    //***initialization variables
    mapSize_m_=100;                                // side length of the map (m)
    cellSize_m_ =0.25;                             // side length of the cell (m)
    CCSWay_ =8;                                    // 4: 4-way; 8: 8-way
    maxLabel =0;                                   // number of segments
    emptySegments=0;

    Debug_ScanPoints_ =true;

    //***Initialize Map
    int hysteresis_s_ =0.1;
    /*** bool RC(row-column scrolling)->true, TT(unsigned char) voidedValue->0 ***/
    segmentationMap_ = HysteresisScrollingByteMap(mapSize_m_, cellSize_m_, true, 0);
    segmentationMap_.setLifeTime(hysteresis_s_);
    segmentationMap_.setUseShallowCopy();

    scrollingShortMap_ = ScrollingShortMap(mapSize_m_, cellSize_m_, true, 0);
    scrollingShortMap_.setUseShallowCopy();

    //***Initialize vector array
    segLabel = vector< vector<RecPoint3D> >(MAXSEGMENTS+1);
    segLabel.clear();
    for (int i=1; i<=MAXSEGMENTS; ++i)
        segLabel[i].reserve(1000);//MAXCELLSINMAP/10);

}


void CCS::parametersReset()
{
    //*** clear segments label
    for(int i=0; i<=maxLabel; ++i)  // segLabel[0] has Nothing!
        segLabel[i].clear();

//    segLabel.clear();
    maxLabel=0;
    emptySegments=0;

    //*** clear cellCentrePoints Set
    sourceSet.clear();

    //*** reset maps' cell value!!!
    scrollingShortMap_.clear();
}


void CCS::centerMap(const char* filename)
{
    ifstream Ain(filename, ios::in);
    if (!Ain.is_open()) throw FAILOPENVEHICLEPOSEFILE;

    int i;
    string str;
    RecPoint2D p;

    //*** Read 5 lines first
    for (i=0; i<5; ++i)  //
        getline(Ain, str);

    //*** Then, begin reading points
    getline(Ain, str, '\t');
    p.x=atof(str.c_str());
    getline(Ain, str, '\n');
    p.y=atof(str.c_str());
//    cout << fixed << setprecision(4) << p.x << "\t"<< p.y<<endl;

    //*** Touch corners to get the segmentationMap_/scrollingShortMap_ centered on the vehiclePose.
    segmentationMap_.centerAt(p);
    scrollingShortMap_.centerAt(p);
//    boundaryMIN_=scrollingShortMap_.getBounds().getMinPoint();
//    cout << fixed << setprecision(4) << boundaryMIN_.x << "\t"<< boundaryMIN_.y << endl; //Check with Segm_mapBoundary_.txt

    Ain.close();
}


void CCS::createSourceSet(const char* filename)
{
    ifstream Ain(filename, ios::in);
    if (!Ain.is_open()) throw FAILOPENSOURCESETFILE;

    int i;
    string str;
    RecPoint3D p;

    //*** Read 4 lines first
    for (i=0; i<4; ++i)  //
        getline(Ain, str);

    //*** Then, begin reading points
    while (1)
    {
        getline(Ain, str, '\t');
        if ( !(p.x=atof(str.c_str())) ) break;
        getline(Ain, str, '\t');
        p.y=atof(str.c_str());
        getline(Ain, str, '\n');
        p.z=atof(str.c_str());

        sourceSet.insert(p);
        setCellValue(scrollingShortMap_, p, 255);
    }

    Ain.close();
}


void CCS::printSourceSet()
{
    ofstream Aout;
    Aout.open("ccs_SourceSetCheck.txt", ios::out|ios::trunc);
    if ( !Aout.is_open() ) throw FAILOPENSOURCESETCHECKFILE;

    Aout << "PointsNum: " << sourceSet.size() << " ----------------------------------------" << endl;
    for (set<RecPoint3D,ltpt3D>::const_iterator itr = sourceSet.begin(); itr != sourceSet.end(); ++itr)
        Aout << fixed << setprecision(4) << itr->x << " " << itr->y << "    " << itr->z << endl;

    Aout << "------------------------------------------------------------------------------" << endl;
    Aout.close();
}


void CCS::setCellValue(ScrollingShortMap & map, const RecPoint3D & pt, const int & value)
{
    //map.set(pt, (unsigned char)value);
    map.set(pt, (short)value);
}


void CCS::cellSweepSegment()
{
    //*** variable setup
    RecPoint3D leftPT, downPT, leftdownPT, rightdownPT;
    bool leftState(false), downState(false), leftdownState(false), rightdownState(false);

    //*** check whether the source set is empty
    if (!sourceSet.size())
    {
        //if (Debug_ScanPoints_)
        //    LogSegLabelCheck(0); // Even ibeoSet is empty, we still want to log info into txt files, for timeStamp synchronization

        return;
    }

    //*** algorithm
    for(set<RecPoint3D, ltpt3D>::const_iterator ii=sourceSet.begin(); ii != sourceSet.end(); ++ii)
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

    } //for(set<RecPoint3D, ltpt3D>::const_iterator ...)


    //*** eliminate 0 size segments
    /*** We must run EITHER one of these two (running them both is WRONG!) ***/
//    segLabelAdjust();
//    createPointsSegmentsFromCells();
}


/* Check whether the neighbour cell is occupied   */
bool CCS::neighbourCheck(const RecPoint3D & dstPT)
{
    bool isValid = false;   // hsbm.get(..., isValid) -> both isSpaceValid and isTimeValid (include checkTimeStamp(time, lifetime_) )
                            // dataMap_.get(..., isValid)->isSpaceValid
                            // timeMap_.get(..., isValid)->isTimeValid (not include checkTimeStamp(time, lifetime_) )
    return ( (int)scrollingShortMap_.get(dstPT, isValid) && isValid );  // occupied && spaceValid
}


/* Get the cell's label  */
int CCS::labelGet(const RecPoint3D & dstPT)
{
    bool isValid = false;
    return (int)scrollingShortMap_.get(dstPT, isValid);
}


/* Set the cell with the label the same as neighbour's  */
void CCS::labelSet(const RecPoint3D & dstPT, const int & value)
{
    setCellValue(scrollingShortMap_, dstPT, value);
}


/* Comebine 2 segments  */
void CCS::segmentsCombine(const int & segLabelMin, const int & segLabelMax)
{
//    cout << "segLabelMax: " << segLabelMax << "-------------------------" << endl;

    for(vector<RecPoint3D>::const_iterator itr = segLabel[segLabelMax].begin();
        itr != segLabel[segLabelMax].end();
        ++itr)
    {
//        cout << fixed << setprecision(4) << itr->x << "," << itr->y << "," << itr->z << endl;


        labelSet(*itr, segLabelMin);
        segLabel[segLabelMin].push_back(*itr);

        if ( labelGet(*itr) != segLabelMin )
            cout << "labelSet() FAILED in segmentsCombine()! point's label is: " << labelGet(*itr) << endl;
    }

//    cout << "==============================================" << endl;

    segLabel[segLabelMax].clear();  // delete the big-label segment
    //cout << "The clear one is segLabelMax: " <<segLabelMax << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<endl;
//    emptySegments++;
//    emptySegments_[emptySegments] = segLabelMax;
//    minLabelSegments_[emptySegments] = segLabelMin;
}


/* Simply set the cell label  */
void CCS::setWithoutCombineCheck(const RecPoint3D & operPT, const RecPoint3D & infoPT)
{
    int tempLabel = labelGet(infoPT);
    labelSet(operPT, tempLabel);
    segLabel[tempLabel].push_back(operPT);
}


/* Set the cell label and Check combine  */
void CCS::setWithCombineCheck(const RecPoint3D & infoPT1, const RecPoint3D & infoPT2)
{
    int LabelPT1 = labelGet(infoPT1);
    int LabelPT2 = labelGet(infoPT2);
    //if (!LabelPT1 || !LabelPT2) cout << 'find 0 Label from scrollingShortMap_' << endl;

    int Labelmin = min(LabelPT1, LabelPT2);
    int Labelmax = max(LabelPT1, LabelPT2);
    if ( Labelmin != Labelmax )  // two segments should combine!
    {
        emptySegments++;

        //*** record emptySegments' label(maxLabel), minLabel
        emptySegments_[emptySegments] = Labelmax;
        minLabelSegments_[emptySegments] = Labelmin;

        //*** record infoPT1, infoPT2
        infoPT1_[emptySegments]= infoPT1;
        infoPT2_[emptySegments]= infoPT2;

        //*** record label BEFORE combining
        labelBefore1_[emptySegments]= LabelPT1;
        labelBefore2_[emptySegments]= LabelPT2;

        ////////////////////////////////
        segmentsCombine(Labelmin, Labelmax);
        ////////////////////////////////

        //*** record label AFTER combining
        labelAfter1_[emptySegments]= labelGet(infoPT1);
        labelAfter2_[emptySegments]= labelGet(infoPT2);
    }
}


/* Create a New Segment  */
void CCS::createNewSegment(const RecPoint3D & dstPT)
{
    maxLabel++;
    labelSet(dstPT, maxLabel);
    segLabel[maxLabel].push_back(dstPT);

#if 0
    /////////////////////////////////////////////////////////////////////////////////
    ofstream Aout;
    Aout.open("ccs_debug.txt", ios::out|ios::app);
    if ( !Aout.is_open() )
        throw FAILOPENDEBUGFILE;

    Aout << "*************************************************" << endl
         << "********** current maxLabel = " << maxLabel << " **********" << endl
         << "*************************************************" << endl;

    for (int i=1; i<=maxLabel; ++i)
    {
        Aout << "segLabel["<< i <<"]:" <<endl;
        for (vector<RecPoint3D>::const_iterator itr = segLabel[i].begin(); itr != segLabel[i].end(); ++itr)
            Aout << fixed << setprecision(4) << itr->x << "," << itr->y << "\t" << labelGet(*itr) << endl;

        Aout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"<<endl;
    }

    Aout << "-----------------------------------------------------------------------------------------" << endl;

    Aout.close();
    /////////////////////////////////////////////////////////////////////////////////
#endif
}


/* Adjust the segLabel, eliminate 0 size segments */
/*void CCS::segLabelAdjust()
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




void CCS::printSegLabel()
{
    bool isTimeValid = false;
    int cellCounts=0, temp=0, emptySegmentsCount=0;
    static int Times_printSegLabel=1;

    ofstream Eout;
    Eout.open("ccs_SegLabelCheck.txt", ios::out|ios::trunc);
    if ( !Eout.is_open() )
        throw FAILOPENSEGLABELCHECKFILE;

    Eout << "[Segm_segLabel_]: " << Times_printSegLabel << "   Number of Segments: " << maxLabel << " --------------------------------" <<endl;
    for (int i=0; i<=maxLabel; ++i)  // segLabel[0] is not a segment, but we still want to output it
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
                     << "cellValue: " << (int)scrollingShortMap_.get(*itr, isTimeValid)
                     << endl;   ///////////////////////////////////
            }
        }
        Eout << "==========================================================" << endl;
*/
        cellCounts += temp;
    }
    Eout << ":::::::::::::::::::::::::::::::::::::::::::::::::: The above total cells: " << cellCounts
         << ", maxLabel: " << maxLabel
         << ", emptySegmentsCount: " << emptySegmentsCount
         << endl;

    Eout << "::::::::::::::::::::::::::::::: nonEmptySegments( createPointsSegmentsFromCells() ): " << "---"
         << ", emptySegments( segmentsCombine() ): " << emptySegments
         << endl;

    for (int i=1; i<=emptySegments; ++i)
    {
        Eout << fixed << setprecision(4)
             << "segLabel[" << emptySegments_[i] << "] should 0 cells, but:" << segLabel[emptySegments_[i]].size() << "cells"
             << ", mapping segLabel[" << minLabelSegments_[i] << "]: " //<< segLabel[minLabelSegments_[i]].size() << "cells"
             << "infoPT1->(" <<infoPT1_[i].x <<","<<infoPT1_[i].y <<") Before: "<<labelBefore1_[i]<<" After:"<<labelAfter1_[i]
             << "infoPT2->(" <<infoPT2_[i].x <<","<<infoPT2_[i].y <<") Before: "<<labelBefore2_[i]<<" After:"<<labelAfter2_[i]
             << endl;
    }

    if ( emptySegmentsCount != emptySegments )
    {
        Eout << "!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!" << endl
             << "!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!" << endl
             << "!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!*!" << endl;
    }

    Eout.close();


    Times_printSegLabel++;
}


