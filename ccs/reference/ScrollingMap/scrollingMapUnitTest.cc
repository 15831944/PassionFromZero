/**
 * @file scrollingMapUnitTest.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 9/25/2006
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _SCROLLINGMAPUNITTEST_CC_
#define _SCROLLINGMAPUNITTEST_CC_

// so we get our main function
#define BOOST_TEST_DYN_LINK 1
#define BOOST_TEST_MAIN 1
#define BOOST_TEST_ALTERNATIVE_INIT_API 1

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <fstream>
#include "ScrollingMap.h"

using boost::unit_test::test_suite;
using boost::unit_test::test_case;
using boost::archive::text_oarchive;
using boost::archive::text_iarchive;
using namespace std;

template <class T>
void save(const string & name, const T & to_save)
{
    ofstream ofs(name.c_str());
    text_oarchive oa(ofs);
    oa << to_save;
}

template <class T>
void load(const string & name, T & to_load)
{
    ifstream ifs(name.c_str());
    text_iarchive ia(ifs);
    ia >> to_load;
}

void changeMapTest(void)
{
    ScrollingDoubleMap map;
    bool isValid;
    double c;
    RecPoint2D index;
    for (index.x = -50; index.x<100;index.x+=0.25)
    {
        for (index.y = -50; index.y<100;index.y+=0.25)
            {
                c = map.get(index,isValid);
                BOOST_CHECK(!isValid);
            }
    }

    double counter=0;
    for (index.x = -50; index.x<100;index.x+=0.25)
    {
        for (index.y = -50; index.y<100;index.y+=0.25, counter+=1)
        {
            map.set(index,counter);
        }
    }

    counter =0;
    for (index.x = -50; index.x<100;index.x+=0.25)
    {
        for (index.y = -50; index.y<100;index.y+=0.25, counter+=1)
        {
            c = map.get(index,isValid);
            if (index.x>=50 && index.y>=50)
                BOOST_CHECK(isValid);
            else
                BOOST_CHECK(!isValid);
        }
    }
    
    ScrollingDoubleMap::iterator smit;
    for (smit= map.begin(RecPoint2D(50,50),
                         RecPoint2D(100,100));
         smit!=map.end(); 
         smit++)
    {
        BOOST_CHECK(smit.isValid());
    }
}

void mapSerializeTest(const bool RC, const bool shallow, const bool compressed)
{
    ScrollingDoubleMap testMap(50,0.25,RC,0.13579);

    cout << "testing ScrollingMap serialization ";
    if(RC)
    {
        cout << "(RC-scrolling)";
    } else {
        cout << "(non-RC)";
    }

    if (shallow)
    {
        testMap.setUseShallowCopy();
        cout << "(shallow copy) ";

        if (compressed)
        {
            testMap.setCompressOnSerialize();
            cout << "(compressed) ";
        }
    }
    cout << endl;

    ScrollingDoubleMap::iterator it;;
    double val = 0;
    RecPoint2D from(-19.875,-19.875), to(29.875,29.875);
    for (it=testMap.begin(from,to); it!=testMap.end();++it, val+=0.25)
    {
        it.set( val );
    }

    std::string filename = "test";
    if(RC)
        filename += "-RC";
    if(shallow)
        filename += "-shallow";
    if(compressed)
        filename += "-compressed";
    filename += ".txt";

    save(filename,testMap);
    ScrollingDoubleMap restoreMap;
    load(filename,restoreMap);

    val =0;
    for (it = restoreMap.begin(from,to);
         it!= restoreMap.end();
         ++it, val+=0.25)
    {
        BOOST_CHECK_EQUAL((*it).first,val);
        BOOST_CHECK_EQUAL((*it).second,true);
        if((*it).second != true)
        {
            RecPoint2D centerpt = it.getCenterPoint();
            fprintf(stderr," -> Error occurred at iterator(%d,%d), at (%lf,%lf)\n",it.getRow(),it.getCol(),centerpt.x,centerpt.y);
        }
    }
}

void mapSerializeTestNormalCopy(void)
{
    mapSerializeTest(false,false, false);
}

void mapSerializeTestShallowCopy(void)
{
    mapSerializeTest(false,true, false);
}

void mapSerializeTestCompressed(void)
{
    mapSerializeTest(false,true, true);
}

void mapSerializeTestNormalCopyRC(void)
{
    mapSerializeTest(true,false, false);
}

void mapSerializeTestShallowCopyRC(void)
{
    mapSerializeTest(true,true, false);
}

void mapSerializeTestCompressedRC(void)
{
    mapSerializeTest(true,true, true);
}

void RCTest(void)
{
    cout << "Running the Row-Column Scrolling Test " << endl;
    double range = 5;
    double cellRes = 0.5;    
    bool RC = 1;
    double voided = -1;
    bool isValid = false;
    RecPoint2D point;
    bool talky = false;
    bool verbose = 0;
    double booga;
    
    ScrollingDoubleMap testMap(range,cellRes,RC,voided);

    // Test that a map covering the origin can handle the (-1,0) in the cell at (-1,-1) placed by mapSetUp
    for( point.x = -1; point.x < 1 ; point.x += cellRes )
    {
        for( point.y = -1; point.y < 1; point.y += cellRes )
        {
            testMap.set( point, point.x + point.y );
        }
    }

    for( point.x = -1; point.x < 1 ; point.x += cellRes )
    {
        for( point.y = -1; point.y < 1; point.y += cellRes )
        {
            testMap.get( point, isValid );
            BOOST_CHECK_EQUAL( isValid, 1);
        }
    }
    
    // fill in map with initial values
    for( point.x = 0; point.x < 5 ; point.x += 1 )
    {
        for( point.y = -1; point.y < 1; point.y += cellRes )
        {
            testMap.set( point, point.x + point.y );
        }
    }
    
    for( point.x = 0; point.x < 5; point.x += 1 )   
    {
        for( point.y = -1; point.y < 1; point.y += cellRes )
        {
            testMap.get( point, isValid );
            BOOST_CHECK_EQUAL( isValid, 1);
        }
    }
    
    // now move a point over 1 column (y-shift) and check that all points in that new column are valid.
    if( talky == 1 ) cout << "move the point one column " << endl;
    point.x = 2; point.y = 5;    
    testMap.set( point, 99 );
    for( point.x = 0; point.x < 5; point.x += 1 )   // y is 5, run through the x values
    {
        double booga = testMap.get( point, isValid ); 
        BOOST_CHECK_EQUAL( isValid, 1);
        if( verbose == 1 ) cout << " at " << point << " the value is: " << booga << " and isValid is: " << isValid << endl;
    }

    // now move the point over 2 columns check that the points are all still valid
    if( talky == 1 ) cout << "move the point 2 columns " << endl;
    point.x = 2; point.y = 7;
    testMap.set( point, 101 );
    for( point.y = 7; point.y > 5; point.y -= 1 )
    {
        for( point.x = 0; point.x < 5; point.x += 1 )   // y is 5, run through the x values
        {
            double booga = testMap.get( point, isValid ); 
            BOOST_CHECK_EQUAL( isValid, 1);
            if( verbose == 1 ) cout << " at " << point << " the value is: " << booga << " and isValid is: " << isValid << endl;
        }
    }

    // now move the point over 6 rows- total rewrite, check that all points are still valid.
    if( talky == 1 ) cout << "move the point 6 rows over. " << endl;
    point.x = 2; point.y = 13;
    testMap.set( point, 101 );
    for( point.y = 9; point.y <= 13; point.y += 1 )
    {
        for( point.x = 0; point.x <= 4; point.x += 1 )   // y is 5, run through the x values
        {
            double booga = testMap.get( point, isValid ); 
            BOOST_CHECK_EQUAL( isValid, 1);
            if( verbose == 1 ) cout << " at " << point << " the value is: " << booga << " and isValid is: " << isValid << endl;
        }
    }    

    // now move the map to the lower left-- total rewrite, check that all points are still valid
    if( talky == 1 ) cout << "move the point far down left. " << endl;
    point.x = -20; point.y = -20;
    testMap.set( point, 999997 );
    for( point.y = -20; point.y < -15; point.y += 1 ){
        for( point.x = -20; point.x < -15; point.x += 1 )   // y is 5, run through the x values
        {
            booga = testMap.get( point, isValid ); 
            BOOST_CHECK_EQUAL( isValid, 1);
            if( verbose == 1 ) cout << " at " << point << " the value is: " << booga << " and isValid is: " << isValid;
        }
    }
    
    point.x = -20; point.y = -20;
    booga = testMap.get( point, isValid ); 
    BOOST_CHECK_EQUAL( booga, 999997);
}




void lineTest(void)
{
    cout << "Running the Line iterator test" << endl;

    double range = 60;      // map needs to be large enough to cover the range in chris's tests
    double cellRes = 0.25;    
    bool RC = 1;
    double voided = -1;
    bool isValid=false;

    ScrollingDoubleMap testMap(range,cellRes,RC,voided);
    //ScrollingDoubleMap testMap(range,cellRes);
    ScrollingDoubleMap::LineIterator lit;
    
    RecPoint2D point( 32, 54 );
    double value = 311;
    testMap.set( point, value );
    BOOST_CHECK_EQUAL(testMap.get(point,isValid),value);
    BOOST_CHECK_EQUAL(isValid,true);


    // first horizontal line check
    value = 536;
    point.x = 15; point.y = 3;
    for( point.x = 15; point.x <= 35; point.x += 0.25, value++ )
    {
        testMap.set( point, value );
    }

    double val = 536;
    for(lit= testMap.beginLine(RecPoint2D(15,3),
                           RecPoint2D(35,3));
        lit!=testMap.endLine();
        ++lit, val++)
    {    
        BOOST_CHECK_EQUAL((*lit).first,val);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }

    // second horizontal line check
    val = 17;
    point.y = 7;
    for (point.x =15; point.x >= -30; point.x-=0.25, val++)
        testMap.set(point,val);
   
    val = 17;
    for(lit= testMap.beginLine(RecPoint2D(15,7),
                           RecPoint2D(-30,7));
        lit!=testMap.endLine();
        ++lit, val++)
    {
        BOOST_CHECK_EQUAL((*lit).first,val);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }

    // vertical line check
    val =15;
    point.x= 35;
    for (point.y = 25; point.y<=36; point.y+=0.25, val++)
        testMap.set(point,val);

    val = 15;
    for(lit= testMap.beginLine(RecPoint2D(35,25),
                               RecPoint2D(35,36));
        lit!=testMap.endLine();
        ++lit, val++)
    {
        BOOST_CHECK_EQUAL((*lit).first,val);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }

    //positive diagonal line check
    int cnt=0;
    for (point.x = -1, point.y =-1; point.x<50; point.y+=0.25, point.x+=0.25,cnt++)
        testMap.set(point,cnt); // set the value of the cell to the count value

    cnt = 0;
    for (lit = testMap.beginLine(RecPoint2D(-1,-1),
                                 RecPoint2D(49.75,49.75));
         lit!=testMap.endLine();
         ++lit, cnt++)
    {
        BOOST_CHECK_EQUAL((*lit).first,cnt);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }

    //negative traversal of positive diagonal line check
    cnt=0;
    for (point.x = 1.05, point.y =1.05; point.x>=-5; point.y-=0.25, point.x-=0.25,cnt++)
    {
        testMap.set(point,cnt);
    }

    cnt = 0;
    for (lit = testMap.beginLine(RecPoint2D(1.05,1.05),
                                 RecPoint2D(-5,-5));
         lit!=testMap.endLine();
         ++lit, cnt++)
    {
        BOOST_CHECK_EQUAL((*lit).first,cnt);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }
    

    //positive traversal of negative diagonal line check
    cnt=0;
    for (point.x = 0, point.y =0; point.x>=-50 && point.y<50; point.y+=0.25, point.x-=0.25,cnt++)
    {
        testMap.set(point,cnt);
    }

    cnt =0 ;
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(-49.75,49.75));
         lit!=testMap.endLine();
         ++lit, cnt++)
    {
        BOOST_CHECK_EQUAL((*lit).first,cnt);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }
        

    cnt=0;
    for (point.x = 0, point.y =0; point.x<50; point.y-=0.25, point.x+=0.25,cnt++) 
    {
        testMap.set(point,cnt);
    }

    cnt =0 ;
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(49.75,-49.75));
         lit!=testMap.endLine();
         ++lit, cnt++)
    {
        BOOST_CHECK_EQUAL((*lit).first,cnt);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }

    
    // Test setting a single value
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(0,0));
         lit!=testMap.endLine();
         ++lit)
    {
        lit.set(59210);

        BOOST_CHECK_EQUAL(59210.0,testMap.get(RecPoint2D(0,0),isValid));
        BOOST_CHECK_EQUAL( isValid, 1 );
    }


    // testing setting a line via a line iterator
    int ii=0;
    for (lit = testMap.beginLine(RecPoint2D(-30,30),
                                RecPoint2D(0,30));
         lit!=testMap.endLine();
         ++lit, ++ii)
    {
        lit.set(7);
    }
    for (lit = testMap.beginLine(RecPoint2D(-30,30),
                                RecPoint2D(0,30));
         lit!=testMap.endLine();
         ++lit)
    {
        BOOST_CHECK_EQUAL(7,(*lit).first);
        BOOST_CHECK_EQUAL(true,(*lit).second);
    }

    for (lit = testMap.beginLine(RecPoint2D(-100,-100),
                                RecPoint2D(-500,-200));
         lit!=testMap.endLine();
         ++lit)
    {
        BOOST_CHECK_EQUAL(false,(*lit).second);
    }

         
    ii=0;
    point.x = -30; point.y =30;
    for (  ; point.x<=0; point.x+=0.25, ii++) {
        BOOST_CHECK_EQUAL(testMap.get(point,isValid),7);
        BOOST_CHECK_EQUAL( 1, isValid );
    }

    // ensure that the previous accessing methods still work
    point.x = 0; point.y = 0;
    lit = testMap.beginLine(point, RecPoint2D(0,0));
    lit.set(9999);
    BOOST_CHECK_EQUAL(9999, (*lit).first);
    BOOST_CHECK_EQUAL((*lit).second,true);
    BOOST_CHECK_EQUAL(testMap.get(point,isValid),9999);


    // Check that the set method for the line iterator doesn't kill things and returns proper values

    range = 5;
    cellRes = 0.25;    
    RC = 1;
    voided = -4;

    ScrollingDoubleMap testMap1(range,cellRes,RC,voided);
 
    RecPoint2D startPt( 4, 4);
    RecPoint2D endPt(8.9, 8.9);
    // fill in the map so that we're all in one location.
    for (ScrollingDoubleMap::iterator i = testMap1.begin(startPt, endPt); i != testMap1.end(); i++ ) 
    {
        i.set(24);
    }
    for (ScrollingDoubleMap::iterator i = testMap1.begin(startPt, endPt); i != testMap1.end(); i++ ) 
    {
        BOOST_CHECK_EQUAL((*i).first,24);
        BOOST_CHECK_EQUAL((*i).second,true);
    }


    // check that line iterator properly sets the value at a point previously off the map and causes the map to scoll
    RecPoint2D lineStartPt( 6,6 );
    RecPoint2D lineEndPt( 10.9, 10.9 );
    for (lit = testMap1.beginLine(lineStartPt,lineEndPt); lit!=testMap1.endLine(); ++lit)
    {
        lit.set(67);
    }
   
    // check that we can set values on a line in a region outside the map.    
    for (lit = testMap1.beginLine(lineStartPt,lineEndPt); lit!=testMap1.endLine(); ++lit)
    {
        BOOST_CHECK_EQUAL((*lit).first,67);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }
    // point inside the old retangle but not in the new one should be invalid.
    RecPoint2D checkStartPt( 4.5,5 );
    RecPoint2D checkEndPt( 5.5, 10 );
    for (lit = testMap1.beginLine(checkStartPt,checkEndPt); lit!=testMap1.endLine(); ++lit)
    {
        BOOST_CHECK_EQUAL((*lit).second,false);
    }
    
    testMap1.set( lineEndPt, 67 );
    for (lit = testMap1.beginLine(checkStartPt,checkEndPt); lit!=testMap1.endLine(); ++lit)
    {
        BOOST_CHECK_EQUAL((*lit).second,false);
    }

    
    // points inside new rectange but not in old one should be valid and contain the voided value.
    RecPoint2D secondCheckStartPt( 6,10 );
    RecPoint2D secondCheckEndPt( 10, 10.5 );
    for (lit = testMap1.beginLine(secondCheckStartPt,secondCheckEndPt); lit!=testMap1.endLine(); ++lit)
    {
        BOOST_CHECK_EQUAL((*lit).first,voided);
        BOOST_CHECK_EQUAL((*lit).second,true);
    }

}



void iteratorTest(void)
{
    cout << "Starting the standard iterator check" << endl;
    double range = 5;      // map needs to be large enough to cover the range in chris's tests
    double cellRes = 0.25;    
    bool RC = 1;
    bool talky = 0;
    RecPoint2D point;


    // test that an initial map can be iterated over without crashing...
    unsigned char voidedByte = 0;
    ScrollingByteMap testMap1( 100, 0.25, RC, voidedByte);
    cout << "Check for iterator crashing and inital set/valid checks" << endl;
   
    RecPoint2D startPt( 400, 400);
    RecPoint2D endPt( 499.75, 499.75 );
    // Check that we can properly set the cells using iterator.set and that they are considered valid.  First, check that
    // these areas are invalid before setting.
    for (ScrollingByteMap::iterator i = testMap1.begin( startPt, endPt ); i != testMap1.end(); i++ ) 
    {
        // this check should actually return invalid, but it shouldn't crash
        BOOST_CHECK_EQUAL( (*i).second, false );
    }
    // check other method for checking isValid.
    for (ScrollingByteMap::iterator i = testMap1.begin(startPt, endPt); i != testMap1.end(); i++ ) 
    {
        BOOST_CHECK_EQUAL(i.isValid(),0);
    }
    for (ScrollingByteMap::iterator i = testMap1.begin(startPt, endPt); i != testMap1.end(); i++ ) 
    {
        i.set(24);
    }
    for (ScrollingByteMap::iterator i = testMap1.begin(startPt, endPt); i != testMap1.end(); i++ ) 
    {
        BOOST_CHECK_EQUAL((*i).first,24);
        BOOST_CHECK_EQUAL((*i).second,true);
        BOOST_CHECK_EQUAL(i.isValid(),1);
    }



    unsigned char booga = 1;
    ScrollingDoubleMap testMap(range,cellRes,RC,booga);
    // simple placement test...center of area is different value, check iterators over portions of both areas.
    
    talky = 0;
    if( talky == 1 ) cout << "fill in map with initial values" << endl;
    RecPoint2D bigStartPt(0,0);
    RecPoint2D bigEndPt(4.9,4.9);
    for (ScrollingDoubleMap::iterator i = testMap.begin( bigStartPt, bigEndPt ); i != testMap.end(); i++ ) 
    {
        i.set( -99 );
    }
    RecPoint2D smallStartPt( 2, 2);
    RecPoint2D smallEndPt( 2.9, 2.9);
    for (ScrollingDoubleMap::iterator i = testMap.begin( smallStartPt, smallEndPt ); i != testMap.end(); i++ ) 
    //for( point.x = 2; point.x < 3 ; point.x += 0.25 )
    {
        i.set( 67 );
    }

    // check that the setting was correct and that the cells are valid
    for (ScrollingDoubleMap::iterator i = testMap.begin(smallStartPt, smallEndPt); i != testMap.end(); i++ ) 
    {
        BOOST_CHECK_EQUAL((*i).first, 67);
        BOOST_CHECK_EQUAL((*i).second,true);
    }

    RecPoint2D regionEndPt(1.75,4.9);
    for (ScrollingDoubleMap::iterator i = testMap.begin(bigStartPt, regionEndPt); i != testMap.end(); i++ ) 
    {
        BOOST_CHECK_EQUAL((*i).first, -99);
        BOOST_CHECK_EQUAL((*i).second,true);
    }

    // Check that iterators outside of this area are labeled invalid
    RecPoint2D outsideStart( 5.25, 0);
    RecPoint2D outsideEnd( 7,2 );
    for (ScrollingDoubleMap::iterator i = testMap.begin(outsideStart, outsideEnd); i != testMap.end(); i++ ) 
    {
        BOOST_CHECK_EQUAL(i.isValid(),0);
    }

    
}


void boundingBoxTest(void)
{
    cout << "Running the bounding box test " << endl;
    double range = 10;
    double cellRes = 0.5;    
    bool RC = 1;
    double voided = -1;
    RecPoint2D minPt(0,0);
    RecPoint2D maxPt(0,0);
    RecPoint2D point, otherMinPt, otherMaxPt;
    RecAxisAlignedBox2D box;
    bool talky = 0;
    bool isValid = false;
    
    ScrollingDoubleMap testMap(range,cellRes,RC,voided);
    otherMinPt.x = -5, otherMinPt.y = -5;   // if we set the map with RC=1, these are the bounds for a map with a range of 10
    otherMaxPt.x = 5, otherMaxPt.y = 5;         // ditto, see constructor for RC = 1 to verify method
   
    // test the bounding box that is returned at initialization
    if( talky == 1 )  cout << "BoundingBox:  check initial box" << endl;
    box = testMap.getBounds();
    BOOST_CHECK_EQUAL( box.getMinPoint(),otherMinPt );
    BOOST_CHECK_EQUAL( box.getMaxPoint(),otherMaxPt );
   
    // Test the bounding box for a partially filled in box
    // should return a box that is outside the written area.
    if( talky == 1 )  cout << "BoundingBox:  check partially filled box" << endl;
    for( point.x = -3; point.x < 2 ; point.x += cellRes )
    {
        for( point.y = -2; point.y < 3; point.y += cellRes )
        {
            testMap.set( point, point.x + point.y );
        }
    }
    minPt.x = -3; minPt.y = -2;     // the theoretical bounding box if the RC=1 constructor isn't called
    maxPt.x = 6.5;  maxPt.y = 7.5;  // ditto
    box = testMap.getBounds();
    BOOST_CHECK( (box.getMinPoint()==minPt)||(box.getMinPoint()== otherMinPt)  );
    BOOST_CHECK( (box.getMaxPoint()==maxPt)||(box.getMaxPoint()== otherMaxPt)  );

    
    // Test that the bounding box of a benign box is valid
    if( talky == 1 )  cout << "BoundingBox:  check fully filled box" << endl;
    for( point.x = -15; point.x < -10 ; point.x += cellRes )
    {
        for( point.y = -15; point.y < -10; point.y += cellRes )
        {
            testMap.set( point, point.x + point.y );
        }
    }
    minPt.x = -15; minPt.y = -15;
    maxPt.x = -5;  maxPt.y = -5;
    box = testMap.getBounds();
    BOOST_CHECK_EQUAL( box.getMinPoint(),minPt );
    BOOST_CHECK_EQUAL( box.getMaxPoint(),maxPt );

    // Test that setting a point off in the distance still returns the proper bounding box
    point.x = 1000;  point.y = 1000;
    testMap.set( point, 999 );    box = testMap.getBounds();
    cout <<"point is: " << point << " range is: " << range << "  " <<  box.getMinPoint() << "  ";
    testMap.get( box.getMinPoint(), isValid );
    cout << "isValid?: " << isValid;
    cout << "  " << box.getMaxPoint();
    testMap.get( box.getMaxPoint(), isValid );
    cout << "isValid?: " << isValid << endl;;
    
    point.x = -10000;  point.y = 1000;
    testMap.set( point, 456 );    box = testMap.getBounds();
    cout <<"point is: " << point << " range is: " << range << "  " <<  box.getMinPoint() << "  ";
    testMap.get( box.getMinPoint(), isValid );
    cout << "isValid?: " << isValid;
    cout << "  " << box.getMaxPoint();
    testMap.get( box.getMaxPoint(), isValid );
    cout << "isValid?: " << isValid << endl;;

    point.x = -100;  point.y = -100;
    testMap.set( point, 456 );    box = testMap.getBounds();
    cout <<"point is: " << point << " range is: " << range << "  " <<  box.getMinPoint() << "  ";
    testMap.get( box.getMinPoint(), isValid );
    cout << "isValid?: " << isValid;
    cout << "  " << box.getMaxPoint();
    testMap.get( box.getMaxPoint(), isValid );
    cout << "isValid?: " << isValid << endl;;
}

test_suite * init_unit_test_suite(int argc, char *argv[]) 
{
    test_suite *test = BOOST_TEST_SUITE("scrolling map tests");
    test->add(BOOST_TEST_CASE(&changeMapTest),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTestNormalCopy),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTestShallowCopy),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTestCompressed),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTestNormalCopyRC),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTestShallowCopyRC),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTestCompressedRC),0);
    test->add(BOOST_TEST_CASE(&RCTest),0);
    test->add(BOOST_TEST_CASE(&lineTest),0);
    test->add(BOOST_TEST_CASE(&iteratorTest),0); 
    test->add(BOOST_TEST_CASE(&boundingBoxTest),0); 
   return test;

}

#endif //ifndef _SCROLLINGMAPUNITTEST_CC_
