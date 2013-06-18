/**
 * @file basicUnitTest.cc
 * @author: Michael Taylor (taylor_michael_a5@cat.com)
 * @date: 09/18/06
 *
 */
#ifndef _BASICUNITTEST_CC_
#define _BASICUNITTEST_CC_

// so we get our main function
#define BOOST_TEST_DYN_LINK 1
#define BOOST_TEST_MAIN 1
#define BOOST_TEST_ALTERNATIVE_INIT_API 1

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include "recGeometry.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/test/floating_point_comparison.hpp>
//MAT added the <boost/test/floating_point_comparison.hpp> to use BOOST_CHECK_CLOSE tool
#include <math.h>
//MAT added <math.h> for use as a math check while setting up tests

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

#define TEST_RW(out,in) \
  save("test.txt",out); \
  load("test.txt",in); \
  BOOST_CHECK_EQUAL(in,out);



void testRecPoint2D(void) 
{
//NOTE: This Test Intentionally Ignores the Testing of Operators

    cout << "Running RecPoint3D" << endl;

    //Testing initialization  
    RecPoint2D Point;
    BOOST_CHECK_MESSAGE( (Point.x == 0)&(Point.y == 0), "New Point not initialized to zero.");
	
    //Testing copying odd variables
    int intDumX = 9;  int intDumY = 8;
    RecPoint2D Point2(intDumX, intDumY);
    BOOST_CHECK_MESSAGE( (Point2.x==intDumX)&(Point2.y==intDumY), "New Point not properly copied Integers");
	
    double doubleDumX = .8555; double doubleDumY = 1.338;
    RecPoint2D Point3(doubleDumX,doubleDumY);
    BOOST_CHECK_MESSAGE( (Point3.x==doubleDumX)&(Point3.y==doubleDumY), "New Point not properly copied Doubles");
	
    //Testing copying large amp variables
    double maxDum = 99999999;  double minDum = -maxDum;
    RecPoint2D Point4(maxDum,maxDum);
    BOOST_CHECK_MESSAGE( (Point4.x==maxDum)&(Point4.y==maxDum), "New Point not properly copied large values");
    RecPoint2D Point5(minDum,minDum);
    BOOST_CHECK_MESSAGE( (Point5.x==minDum)&(Point5.y==minDum), "New Point not properly copied min values");

    //Testing placing odd variable 
    Point.x = intDumX;  Point.y = intDumY;
    BOOST_CHECK_MESSAGE( (Point.x==intDumX)&(Point.y==intDumY), "Failed to properly set 'x' or 'y' to an int");


    //Testing 'interesting' functions- basically just the distance functions
    Point.x = 55690980.23;  Point.y = -99987243;
    Point2 = Point;
    BOOST_CHECK_MESSAGE( (Point.distance(Point2)==0) , "Distance of zero failed");
    BOOST_CHECK_MESSAGE( (Point.distanceSq(Point2)==0) , "DistanceSq of zero failed");
    
    Point2.x = Point.x + 3000;  Point2.y = Point.y + 4000;
    BOOST_CHECK_MESSAGE( (Point.distance(Point2)==5000) , "Distance of 5000 failed");
    BOOST_CHECK_MESSAGE( (Point.distanceSq(Point2)==25000000) , "DistanceSq of 25000000 failed");
    
    Point2.x = Point.x - 3000;  Point2.y = Point.y - 4000;
    BOOST_CHECK_MESSAGE( (Point.distance(Point2)==5000) , "Distance of 5000 in opposite direction failed");
    BOOST_CHECK_MESSAGE( (Point.distanceSq(Point2)==25000000) , "DistanceSq of 25000000 in opposite direction failed");
    
    Point2 -= Point2;  Point -= Point;  // Set both points to zero
    BOOST_CHECK_MESSAGE( (Point.distance(Point2)==0) , "Distance between two origin points failed");
    BOOST_CHECK_MESSAGE( (Point.distanceSq(Point2)==0) , "DistanceSq between two origin points failed");

}



void testRecPoint3D(void) 
{
    cout << "Running RecPoint3D" << endl;

    //Testing initialization  
    RecPoint3D Point;
    BOOST_CHECK_MESSAGE( (Point.x == 0)&(Point.y == 0)&(Point.z == 0), "New Point not initialized to zero.");
	
    //Testing copying Ints
    int intDumX = 9;  int intDumY = 8;  int intDumZ = -7;
    RecPoint3D Point2(intDumX, intDumY, intDumZ);
    BOOST_CHECK_MESSAGE( (Point2.x==intDumX)&(Point2.y==intDumY)&(Point2.z==intDumZ), 
        "New Point not properly copied Integers");

    //Testing copying Doubles- Large Amp	
    double doubleDumX = 85556543.12; double doubleDumY = .04098235;  double doubleDumZ = -3338654.651;
    RecPoint3D Point3(doubleDumX,doubleDumY,doubleDumZ);
    BOOST_CHECK_MESSAGE( (Point3.x==doubleDumX)&(Point3.y==doubleDumY)&(Point3.z==doubleDumZ), 
        "New Point not properly copied Doubles");

    //Testing placing odd variable 
    Point.x = intDumX;  Point.y = doubleDumY;  Point.z = doubleDumZ;
    BOOST_CHECK_MESSAGE( (Point.x==intDumX)&(Point.y==doubleDumY)&(Point.z==doubleDumZ), 
        "Failed to properly set 'x' or 'y' to an int");

    //No real methods to test
}




void testRecBox2D(void) 
{
    cout << "Running RecBox2D" << endl;

//This Test Intentionally Ignores the Testing of Operators

// This file has one issue:  The RecBox has two elements, a min and max point, but there is little control on these
// points.  The box can be initialized with the min point to the upper right of the max point.  The box can be 
// initialized with two origin points, making it a zero area box.  However, the min or max point can not be written to
// UNLESS the proposed min point is to the lower left of the max point (and vice versa for writing to the max point).  
// Additionally, the box cannot be initialized unless the LineSegment used has a positive slope.

//Not sure when we want to fix this...


//Common Prep Area

    RecPoint2D NegPoint;  NegPoint.x = -9872234.2342;  NegPoint.y = -123412.1234;  //Prep for large neg passing test
    RecPoint2D PosPoint;  PosPoint.x =  289234.6185;   PosPoint.y = 839384.9873;
    RecPoint2D Origin;    Origin.x = 0; 	           Origin.y = 0; 
    RecPoint2D SmallNeg;  SmallNeg.x = -5;  	   SmallNeg.y = -5; 
    RecPoint2D SmallPos;  SmallPos.x = 5;  	           SmallPos.y = 5; 
    RecPoint2D HugeNegPoint( NegPoint.x - 1234, NegPoint.y - 9872);
    RecPoint2D HugePosPoint( PosPoint.x + 1234, PosPoint.y + 9872);
    RecPoint2D MiniNegPoint( SmallNeg.x + .654654, SmallNeg.y + .85465);
    RecPoint2D MiniPosPoint( SmallNeg.x + .16945, SmallNeg.y + .2548);
    RecPoint2D SmallUpperCorner = SmallPos;
    RecPoint2D SmallLowerCorner = SmallNeg;
    RecPoint2D SmallOverlapLower(3.234,3.1234);
    RecPoint2D SmallOverlapUpper(13.78,13.7890);
    RecAxisAlignedBox2D SmallOverlap( SmallOverlapLower, SmallOverlapUpper);	// Box for testing overlap
    RecPoint2D SmallSeparateLower(13.78,13.7890);
    RecPoint2D SmallSeparateUpper(23.78,23.7890);
    RecAxisAlignedBox2D SmallSeparate( SmallSeparateLower, SmallSeparateUpper);	// Box for testing overlap
    RecAxisAlignedBox2D ZeroBox;
    RecAxisAlignedBox2D TargetBox;
    RecAxisAlignedBox2D DoubleBox(-30.009, 37.0, -99.0, .82);
    RecAxisAlignedBox2D IntBox((int)-30, (int)37, (int)-99, (int)82);  //Prep for int passing test
    RecAxisAlignedBox2D BigBox;
    RecAxisAlignedBox2D SmallBox(SmallNeg.x, SmallPos.x, SmallNeg.y, SmallPos.y);
    RecLineSegment2D Line(NegPoint,PosPoint);  //A line Segment with large values
    RecLineSegment2D IntLine(SmallNeg, SmallPos);	//A line segment with integer points
    RecLineSegment2D BackwardLine(SmallPos,SmallNeg);  //A line Seg where first point is lower left of second point


//Test: Initialization  
    BOOST_CHECK_MESSAGE( (ZeroBox.getMinPoint().x==0)&(ZeroBox.getMinPoint().y==0), "Not initialized to zero");
    BOOST_CHECK_MESSAGE( (ZeroBox.getMaxPoint().x==0)&(ZeroBox.getMaxPoint().y==0), "Not initialized to zero");
    BOOST_CHECK_MESSAGE( (DoubleBox.getMinPoint().x==-30.009)&(DoubleBox.getMinPoint().y==-99.0),
        "Bad double initialization");
    BOOST_CHECK_MESSAGE( (DoubleBox.getMaxPoint().x==37.0)&(DoubleBox.getMaxPoint().y==.82),
        "Bad double initialization");
    BOOST_CHECK_MESSAGE( (IntBox.getMinPoint().x==-30)&(IntBox.getMinPoint().y==-99),"Bad int initialization");
    BOOST_CHECK_MESSAGE( (IntBox.getMaxPoint().x==37)&(IntBox.getMaxPoint().y==82),"Bad int initialization");


//Test:  Constructor from min and max points
    // Check to make sure that order does not matter
    RecAxisAlignedBox2D BoxTarget(NegPoint,PosPoint);
    BOOST_CHECK( (BoxTarget.getMinPoint()==NegPoint)&(BoxTarget.getMaxPoint()==PosPoint) );
    RecAxisAlignedBox2D CheckBox(PosPoint,NegPoint);
    BOOST_CHECK( (BoxTarget.getMinPoint()==NegPoint)&(BoxTarget.getMaxPoint()==PosPoint) );


//Test:  Constructor from line segment--- may cause errors from getMin and Max points
    RecAxisAlignedBox2D BoxLine( Line );  
    BOOST_CHECK( (BoxLine.getMinPoint()==NegPoint)&(BoxLine.getMaxPoint()==PosPoint) );
    
    RecAxisAlignedBox2D BoxIntLine( IntLine );  
    BOOST_CHECK( (BoxIntLine.getMinPoint()==SmallNeg)&(BoxIntLine.getMaxPoint()==SmallPos) );
    
    RecAxisAlignedBox2D BoxBackwardLine( BackwardLine );
    BOOST_CHECK_MESSAGE( BoxBackwardLine.getMinPoint()==BackwardLine.p1() , 
        "Prevented from making a box with a neg sloped line");
    //This shouldn't print a zero...but it will if the line handed in has a negative slope
    cout << "	This min point shouldn't be zero if we could make a box with neg sloped line:  " <<
        BoxBackwardLine.getMinPoint().x << endl;	


//Test:  Copy Constructor
    RecAxisAlignedBox2D DummyBox(DoubleBox);
    BOOST_CHECK( (DummyBox.getMinPoint().x==DoubleBox.getMinPoint().x)
        &(DummyBox.getMinPoint().y==DoubleBox.getMinPoint().y) );
    BOOST_CHECK( (DummyBox.getMaxPoint().x==DoubleBox.getMaxPoint().x)
        &(DummyBox.getMaxPoint().y==DoubleBox.getMaxPoint().y) );


//Test:  getMinPoint and getMaxPoint
    BigBox.setMinPoint(NegPoint);
    BOOST_CHECK_MESSAGE( (BigBox.getMinPoint().x == NegPoint.x)&(BigBox.getMinPoint().y == NegPoint.y),
        "Couldn't set large neg min point");
    
    BigBox.setMaxPoint(PosPoint);
    BOOST_CHECK_MESSAGE( (BigBox.getMaxPoint().x == PosPoint.x)&(BigBox.getMaxPoint().y == PosPoint.y),
        "Couldn't set large pos max point");
	

//Test:  setMinPoint and setMaxPoint
    TargetBox.setMinPoint(PosPoint);
    BOOST_CHECK_MESSAGE( TargetBox.getMinPoint()==Origin,"Oops, shouldn't have overwritten those points");
    TargetBox.setMaxPoint(NegPoint);
    BOOST_CHECK_MESSAGE( ZeroBox.getMaxPoint()==Origin,"Oops, shouldn't have overwritten those points");
    TargetBox.setMinPoint( NegPoint );  BOOST_CHECK( TargetBox.getMinPoint()==NegPoint );	//Large Amp Test
    TargetBox.setMaxPoint( PosPoint );  BOOST_CHECK( TargetBox.getMaxPoint()==PosPoint );	//Large Amp Test
    TargetBox.setMinPoint( SmallNeg );  BOOST_CHECK( TargetBox.getMinPoint()==SmallNeg);	//Int test


//Test:  area
    double AreaCheck = (SmallPos.y-SmallNeg.y)*(SmallPos.x-SmallNeg.x);
    BOOST_CHECK_MESSAGE( SmallBox.area() == AreaCheck, "Area int calcs messed up");
    AreaCheck = (PosPoint.y-NegPoint.y)*(PosPoint.x-NegPoint.x);
    BOOST_CHECK_CLOSE( BigBox.area(), AreaCheck, 1e-30);
	

//Test:  isInside and isOutside
    BOOST_CHECK_MESSAGE( SmallBox.isInside(Origin), "Missed the point being inside" );	//Int test
    BOOST_CHECK_MESSAGE( !SmallBox.isInside(PosPoint), "Missed the point being outside" );
    BOOST_CHECK_MESSAGE( SmallBox.isOutside(PosPoint), "Missed the point being outside" );
    BOOST_CHECK_MESSAGE( !SmallBox.isOutside(Origin), "Missed the point being inside" );
    BOOST_CHECK_MESSAGE( BigBox.isInside(Origin), "Missed the point being inside" );	//Int test
    BOOST_CHECK_MESSAGE( !BigBox.isInside(HugePosPoint), "Missed the point being outside" );
    BOOST_CHECK_MESSAGE( BigBox.isOutside(HugeNegPoint), "Missed the point being outside" );
    BOOST_CHECK_MESSAGE( !BigBox.isOutside(Origin), "Missed the point being inside" );
	

//Test:  isInsideTiled and isOutsideTiled
    BOOST_CHECK_MESSAGE( SmallBox.isInsideTiled(SmallLowerCorner),"Missed tiled Inside");	//Int test
    BOOST_CHECK_MESSAGE( !SmallBox.isInsideTiled(SmallUpperCorner),"Missed tiled Outside");	//Int test
    BOOST_CHECK_MESSAGE( SmallBox.isOutsideTiled(SmallUpperCorner),"Missed tiled Outside");	//Int test	
    BOOST_CHECK_MESSAGE( !SmallBox.isOutsideTiled(SmallLowerCorner),"Missed tiled Inside");	//Int test
    BOOST_CHECK_MESSAGE( BigBox.isInsideTiled(NegPoint), "Missed tiled Inside");	//double test
    BOOST_CHECK_MESSAGE( !BigBox.isInsideTiled(PosPoint), "Missed tiled Outside");	//double test
    BOOST_CHECK_MESSAGE( BigBox.isOutsideTiled(PosPoint), "Missed tiled Outside");	//double test	
    BOOST_CHECK_MESSAGE( !BigBox.isOutsideTiled(NegPoint), "Missed tiled Inside");	//double test
	

//Test:  isOnBox
    BOOST_CHECK_MESSAGE( SmallBox.isOnBox(SmallUpperCorner), "Missed On Box" );
    BOOST_CHECK_MESSAGE( !SmallBox.isOnBox(PosPoint), "Missed Not On Box" );
    BOOST_CHECK_MESSAGE( BigBox.isOnBox(PosPoint), "Missed On Box" );


//Test:  overlap
    BOOST_CHECK( SmallBox.overlap(SmallOverlap) );		//Clear overlap
    BOOST_CHECK( !SmallBox.overlap(SmallSeparate) );	//Separate boxes
    BOOST_CHECK( SmallOverlap.overlap(SmallSeparate) );	//Sharing an edgeline
    BOOST_CHECK( BigBox.overlap(SmallOverlap) );		//One Box wholly in another
    BOOST_CHECK( SmallOverlap.overlap(BigBox) );		//One Box wholly in another


//Test:  overlapArea
    RecAxisAlignedBox2D Center(-10,10,-10,10);  // This is the center box that won't move
    RecAxisAlignedBox2D NE(5,15,5,15);
    RecAxisAlignedBox2D NW(-5,-15,5,15);
    RecAxisAlignedBox2D SW(-5,-15,-5,-15);
    RecAxisAlignedBox2D SE(5,15,-5,-15);
    RecPoint2D Max, Min;

    TargetBox = Center.overlapArea( NE );  Min = TargetBox.getMinPoint();  Max = TargetBox.getMaxPoint();
    BOOST_CHECK( (Min.x == 5) );
    BOOST_CHECK( (Min.y == 5) );
    BOOST_CHECK( (Max.x == 10) );
    BOOST_CHECK( (Max.y == 10) );
    
    cout << "The Min Point of the Overlap Area: " << Min.x << " " << Min.y << endl;
    cout << "The Max Point of the Overlap Area: " << Max.x << " " << Max.y << endl;

    TargetBox = Center.overlapArea( NW );  Min = TargetBox.getMinPoint();  Max = TargetBox.getMaxPoint();
    BOOST_CHECK( (Min.x == -10) );
    BOOST_CHECK( (Min.y == 5) );
    BOOST_CHECK( (Max.x == -5) );
    BOOST_CHECK( (Max.y == 10) );
    cout << "The Min Point of the Overlap Area: " << Min.x << " " << Min.y << endl;
    cout << "The Max Point of the Overlap Area: " << Max.x << " " << Max.y << endl;

}  //end testRecBox2D


void testRecPolygon2D(void) 
{
    cout << "Running RecPolygon2D" << endl;
//This Test Intentionally Ignores the Testing of Operators

//Prep: This section may cause errors that will also be caught in other areas as it relys on some member functions	
    RecPolygon2D Poly;  // Basic Polygon for tests-- DO NOT ALTER!
    RecPolygon2D PolyLocked;  // This poly holds onto Vert0:5, not to be altered outside that.
    RecPolygon2D PolyEmpty;  // Constant Poly for checking the operation of methods on a newly constructed poly.
    RecPoint2D   Vert0(  (int)-6, (int)3);  //Quadrant 2
    RecPoint2D   Vert1(  -2.72, -11.11);  	//Quadrant 3
    RecPoint2D   Vert2( 6.67, -6.28);  //Quadrant 4
    RecPoint2D   Vert3( 2.72,  3.14);  //Quadrant 1
    RecPoint2D   Vert4( .01,    11.11);
    RecPoint2D   BigVert;	BigVert.x = -98345.987;  BigVert.y = 323983.394;
    RecPoint2D   Origin;	Origin.x = 0;	  Origin.y = 0;
    RecPoint2D   Blank;
    RecPoint2D   PointTarget, PointTarget2, PointTarget3, PointTarget4, PointTarget5, PointTarget6, PointTarget7; 
    RecPoint2D   PointTarget8, PointTarget9;	
    RecPoint2D   IntVert( -3, 5 );
    double	 adjust = .0001;  //small number
    RecPoint2D   CloseXVert;  CloseXVert.x = Vert1.x+adjust;  CloseXVert.y = Vert1.y;
    RecPoint2D   CloseYVert;  CloseYVert.x = Vert3.x;   	  CloseYVert.y = Vert3.y+adjust;	
    int          Num;

    // Develop Polygons for later testing:

    // These points provide large values for testing, will be used to make the large ~square and the concave polygon
    RecPoint2D  B1(3872345.2345, 1230987.234), B2(-2349287.2342, 909875.0982), B3(-2382104.235,-785243.20398);
    RecPoint2D  B4(2343790.234,-1245798.123);
    RecPoint2D inbig(-1789432,-52487);  //Just a large point that is inside of Big Polygon
    RecPoint2D offset(0,0);  // Allows movement of all points, but keep results simple for verification
    // These points are used to make two side-by-side squares...or one large rectangle
    RecPoint2D R1(2,1), R2(0,1), R3(-2,1), R4(-2,-1), R5(0,-1), R6(2,-1);
    // These points add diamond tips to the ends of the squares
    RecPoint2D D1(-1,0), D2(1,0), D3(3,0);
    // These points make a triangle that passes through one square an shares part of a line with the other square
    RecPoint2D T1(0,3), T2(-2,-3), T3(0,-3);
    // These points make a simple, smaller concave section that's more easily envisioned
    RecPoint2D SC1(10,10), SC2(-10,10), SC3(-10,-10), SC4(10,-10), SC5(10,-8), SC6(-8,-8), SC7(-8,8), SC8(10,8);
    // These points make an even smaller concave section whose convex hull will split Sq2 in half
    RecPoint2D OC1(1,10), OC2(-10,10), OC3(-10,-10), OC4(1,-10), OC5(1,-8), OC6(-8,-8), OC7(-8,8), OC8(1,8);
    // These points will be used to create a complex semi-heart shaped poly using simpleconcave as a base
    RecPoint2D H1(3,-8.5), H2(-3,-8.5), H3(0,-9.5);
    // These points will be used to create a complex, concave shaped poly using simpleconcave as a base
    RecPoint2D C1(3,-7.5), C2(-3,-7.5), C3(0,-6.5);
    // These points create a star-shaped body to fake out the edge-alligning min enclosing rectange
    RecPoint2D S1(-10,0), S2(0,-10), S3(10,0), S4(0,10), S5(2,2), S6(-2,2), S7(-2,-2), S8(2,-2);


    // Construct the Polygons for geometric testing-- make some of them intentionally out of order as a check.
    RecPolygon2D Tri;  Tri.addVertex(T1); Tri.addVertex(T2); Tri.addVertex(T3);
    RecPolygon2D Sq1;  Sq1.addVertex(R2); Sq1.addVertex(R3); Sq1.addVertex(R4); Sq1.addVertex(R5);
    RecPolygon2D Sq2;  Sq2.addVertex(R1); Sq2.addVertex(R2); Sq2.addVertex(R5); Sq2.addVertex(R6);
    RecPolygon2D Rect;  Rect.addVertex(R1); Rect.addVertex(R2); Rect.addVertex(R3); Rect.addVertex(R4); 
        Rect.addVertex(R5); Rect.addVertex(R6);
    RecPolygon2D Big;  Big.addVertex(B1); Big.addVertex(B2); Big.addVertex(B3); Big.addVertex(B4);
    RecPolygon2D Concave(Big);  Concave.addVertex(R4); Concave.addVertex(R3);
    RecPolygon2D Backspace; Backspace.addVertex(R1); Backspace.addVertex(R2); Backspace.addVertex(D1);
        Backspace.addVertex(R5); Backspace.addVertex(R6);
    RecPolygon2D Tab;  Tab.addVertex(R2); Tab.addVertex(R3); Tab.addVertex(R4); Tab.addVertex(R5); Tab.addVertex(D2);
    RecPolygon2D MiniConcave(Sq1);  MiniConcave.addVertex(D1);
    RecPolygon2D Dia;  Dia.addVertex(D3); Dia.addVertex(R1);  Dia.addVertex(D2);  Dia.addVertex(R6);  
    RecPolygon2D SimpleConcave; SimpleConcave.addVertex(SC1);SimpleConcave.addVertex(SC2);
        SimpleConcave.addVertex(SC3);SimpleConcave.addVertex(SC4); SimpleConcave.addVertex(SC5); 
        SimpleConcave.addVertex(SC6);  SimpleConcave.addVertex(SC7);  SimpleConcave.addVertex(SC8);
    RecPolygon2D OverlapConcave; OverlapConcave.addVertex(OC1); OverlapConcave.addVertex(OC2); 
        OverlapConcave.addVertex(OC3); OverlapConcave.addVertex(OC4); OverlapConcave.addVertex(OC5);
        OverlapConcave.addVertex(OC6); OverlapConcave.addVertex(OC7); OverlapConcave.addVertex(OC8);
    RecPolygon2D Heart(SimpleConcave);  Heart.addVertex(H1);  Heart.addVertex(H2);  Heart.addVertex(H2);
    RecPolygon2D Complex(SimpleConcave);  Complex.addVertex(C1); Complex.addVertex(C2);  Complex.addVertex(C3);
    RecPolygon2D Star; Star.addVertex(S1); Star.addVertex(S2); Star.addVertex(S3); Star.addVertex(S4); 
        Star.addVertex(S5); Star.addVertex(S6); Star.addVertex(S7); Star.addVertex(S8); 
	

// Test:  Constructor
    PolyLocked.getCenter(PointTarget);
    BOOST_CHECK_MESSAGE( (PolyLocked.getNumVertices() == 0)&(PointTarget.x==0)&(PointTarget.y==0), 
        "Polygon was developed incorrectly");
	
    // Prep: adding vertices (with checks) to check copy
    PolyLocked.addVertex(Vert0);  PolyLocked.addVertex(Vert1);  PolyLocked.addVertex(Vert2);  
    PolyLocked.addVertex(Vert3);
    BOOST_CHECK_MESSAGE( PolyLocked.getVertex(0,PointTarget) == 0, "The getVertex did not work!" ); //Simple
    BOOST_CHECK_MESSAGE( (PointTarget.x == Vert0.x)&(PointTarget.y == Vert0.y), 
        "The addVertex didn't seem to write correctly." );
    BOOST_CHECK_MESSAGE( PolyLocked.getVertex(2,PointTarget) == 0, "The getVertex did not work!" );
    BOOST_CHECK_MESSAGE( (PointTarget.x == Vert2.x)&(PointTarget.y == Vert2.y),
        "The addVertex didn't seem to write correctly." );	


//Test:  Copy Constructor
    RecPolygon2D PolyTarget(PolyLocked);  
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(3,PointTarget2);
    BOOST_CHECK_MESSAGE( (PointTarget.x==Vert0.x)&(PointTarget2.y==Vert3.y), 
        "Copy Constructor Doesn't Work- or add vertex failed");
    BOOST_CHECK_MESSAGE( (!(PointTarget.x==Vert1.x))&(!(PointTarget2.y==Vert2.y)), 
        "Copy Constructor Doesn't Work- or add vertex failed"); 
    cout << endl << "While there is a Copy Constructor, there is no '=' operator defined.  Something to think about."
        << endl << endl;	
	

//Test:  Destructor-- not sure how to properly check and report on this one
	

// Test: getNumVertices
    BOOST_CHECK_MESSAGE( Poly.getNumVertices()==0, "Get Num Vertices Failed for Poly");
    BOOST_CHECK_MESSAGE( PolyLocked.getNumVertices()==4, "Get Num Vertices Failed for PolyLocked");
	
	
// Test:  getVertex
    // This is a duplicate check at this point-- unless this become more indepth 
    BOOST_CHECK_MESSAGE( PolyLocked.getVertex(3,PointTarget) == 0, "The getVertex did not work!" ); //Simple
    BOOST_CHECK_MESSAGE( (PointTarget.x == Vert3.x)&(PointTarget.y == Vert3.y), 
        "The addVertex didn't seem to write correctly." );
		
    // Problem~ when I ask for the 5th vertex of a sqaure, I'm given the 1st vertex.  Shouldn't this error?
    BOOST_CHECK_MESSAGE( !PolyLocked.getVertex(4,PointTarget) == 0, 
        "Asking for the 5th vertex of a square passes without error.  Is this permissable?" ); 
    BOOST_CHECK_MESSAGE( !(PointTarget.x == Vert0.x)&!(PointTarget.y == Vert0.y), 
        "Handed the 1st vertex of a square when asking for the 5th vertex.  Is this permissable?" );

    // SERIOUS PROBLEM- MEMORY ACCESS VIOLATION WHEN GETTING A VERTEX THAT DOESN'T EXIST.
    cout << endl << "If you're reading this, we still haven't fixed the memory access error in the getVertex function"
        << endl << endl;	
    // BOOST_CHECK_MESSAGE( PolyEmpty.getVertex(0,Blank)==-1, "Didn't clear the vertices properly");
    // The above line causes a memory access violation because getVertex can't currently handle this


// Test:  getCenter
    RecPoint2D P0(3.1,3.1), P1(-.9,3.1), P2(-.9,-.9), P3(3.1,-.9);
    RecPolygon2D Vanilla;  Vanilla.addVertex(P0); Vanilla.addVertex(P1); Vanilla.addVertex(P2); 
        Vanilla.addVertex(P3); Vanilla.getCenter(PointTarget);  
    BOOST_CHECK_MESSAGE( (PointTarget.x==1.1)&(PointTarget.y==1.1), "Center Calc may be messed up");


// Test:  getBoundingBox
    Vanilla.getBoundingBox(PointTarget2,PointTarget3);
    BOOST_CHECK_MESSAGE( (PointTarget2.x==-.9)&(PointTarget3.y==3.1), "BoundingBox Calc may be messed up");
    Big.getBoundingBox(PointTarget4,PointTarget5);
    BOOST_CHECK_MESSAGE( (PointTarget4.x==B3.x)&(PointTarget5.y==B1.y), "BoundingBox errored on Big" );


// Test:  addVertex

    // Make sure tolerance check catches adding a dupiclate vertex, check by vertex number increasing
    RecPolygon2D Dummy(PolyLocked);
    Num = Dummy.getNumVertices();  Dummy.addVertex(Vert1);
    BOOST_CHECK_MESSAGE ( Dummy.getNumVertices()==Num, "Oops: Added Vertex that already existed");
	
    // Make sure the tolerance check catches another vertex 'close' to existing one, check by vertex number increasing
    Num = Dummy.getNumVertices();  
    Dummy.addVertex(CloseXVert);
    BOOST_CHECK_MESSAGE( Dummy.getNumVertices()==Num, "Shucks: Added Vertex that already existed");
    Num = Dummy.getNumVertices();  
    Dummy.addVertex(CloseYVert);
    BOOST_CHECK_MESSAGE( Dummy.getNumVertices()==Num, "Dang: Added Vertex that already existed");	


//Test: sortVerticesCCW 	
    // working on 4 vertices, cast as 3 doubles and 1 int.  Oddly, from the perspective of points, the sorting starts
    // with the point closest to the negative x-axis in the 3rd quadrant, then sorts CCW from there.  Given the angle 
    //convention, though, this sets the angles (from the center of the bounding box to each vertex) in increasing 
    //order, starting from most negative (-pi).
    cout << "sortVerticiesCCW does not have a solid way to order two vertices that have the same angle to the center" 
        << endl;
    RecPolygon2D Dummy2(PolyLocked);
    Dummy2.sortVerticesCCW();  
    Dummy2.getVertex(0,PointTarget);  Dummy2.getVertex(3,PointTarget2);
    BOOST_CHECK_MESSAGE( (PointTarget.x == Vert1.x)&(PointTarget2.y == Vert0.y), "Sorting of points was not in order");
    
    RecPolygon2D Dummy3(Big);  
    Dummy3.sortVerticesCCW(); 
    Dummy3.getVertex(0,PointTarget); Dummy3.getVertex(2,PointTarget3);  Dummy3.getVertex(3,PointTarget4);
    BOOST_CHECK( (PointTarget==B3) & (PointTarget3==B1) & (PointTarget4==B2) );

	
// Test:  clearVertices 
    PolyTarget.clearVertices();  
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices()==0, "Didn't clear the vertices PolyTarget");
	

// Test:  containtsPoint---sorta weak  
    // Vertices are not considered to be contained within their own polygon
    // This function passes the CONCAVE TEST
    cout << endl <<"containsPoint does not consider the vertices of the polygon to be contained within. "  << 
        " Is this permissable?" << endl << endl;
    BOOST_CHECK( SimpleConcave.containsPoint(SC1) == 0 ); // Check to make sure vertices are not contained within
    BOOST_CHECK( Tri.containsPoint(R5) == 0 );            // Point on an edge is not contained
    BOOST_CHECK( Sq1.containsPoint(D1) == 1 );            // A Point at the center of the square is contained
    BOOST_CHECK( Sq1.containsPoint(B1) == 0 );            // Tossing it a big point
    BOOST_CHECK( SimpleConcave.containsPoint(R4) == 0 );  // Point inside of concave area should not be contained
    BOOST_CHECK( Big.containsPoint(inbig) == 1 );         // Checking a slightly larger point-- this should be outside
    BOOST_CHECK( SimpleConcave.containsPoint(R6) == 0 );  // Checking on another point in a divot


// Test:  hasOverlap
    cout << endl << "hasOverlap considers coincident faces of two polygons sufficient to call it Overlap."  <<
        "Seems inconsistent with containsPoint." << endl << endl;
    BOOST_CHECK( Big.hasOverlap(Backspace) );  //Total containment
    BOOST_CHECK( Tri.hasOverlap(Backspace) );  //Partial overlap
    BOOST_CHECK( !Sq1.hasOverlap(Dia) );       //Separate
    // The next is actually separate, but the Diamond is totally inside the concave's divot.
    BOOST_CHECK_MESSAGE( !Concave.hasOverlap(Dia), "One Poly in the divot of another should not have overlap" );   
    BOOST_CHECK( Sq1.hasOverlap(Sq2) );	   //The two squares are SHARING AN EDGE?
    BOOST_CHECK( Sq1.hasOverlap(Tri) );	   //The square and triangle are sharing an edge
    BOOST_CHECK( Sq1.hasOverlap(Sq1) );	   //What happens when it's handed the same object?
    BOOST_CHECK_MESSAGE( !SimpleConcave.hasOverlap(Sq2), 
        "A polygon sitting in a divot should not have overlap.  But it does." );
    BOOST_CHECK_MESSAGE( !OverlapConcave.hasOverlap(Sq2), 
        "A polygon sitting in a divot should not have overlap.  But it does." );


// Test:  computeArea---kinda weak
    BOOST_CHECK( Tri.computeArea() == 6 );
    BOOST_CHECK( Backspace.computeArea() == 5 );
    BOOST_CHECK( Vanilla.computeArea() == 16 );
    double areacalc = Big.computeArea();  //cout << "The area of BIG is " << areacalc << endl; // <-- cheat...
    BOOST_CHECK_CLOSE( areacalc, 1.14724e13, 1e-3);	//This is a pretty rough calc, based on a cheat
    BOOST_CHECK_MESSAGE( SimpleConcave.computeArea() == 112, "Compute area on simple concave didn't work");
    BOOST_CHECK_MESSAGE( MiniConcave.computeArea() == 3, "Compute area on mini concave didn't work");


// Test  computeIntersection-  
    // Relying on the fact that this method sorts the vertices at the end in order to know which vertex to ask for.
    PolyTarget.computeIntersection(Tri, Tab);
    PolyTarget.getVertex(0,PointTarget); PolyTarget.getVertex(1,PointTarget2);  PolyTarget.getVertex(3,PointTarget3);
    BOOST_CHECK( PolyTarget.getNumVertices() == 4 );
    BOOST_CHECK( PointTarget2==R5 );			//Check that the vertex of the tab is the same as the new vertex
    double interp = ((R2.x - R3.x)*2/3) + R3.x;  	//Calcing where the triangle and tab intersect at top
    double interped = ((R5.x - R4.x)*1/3) + R4.x;  	//Calcing where the triangle and tab intersect at top
    BOOST_CHECK( PointTarget3.y==R2.y );		//Check the y coord of the upper triangle/tab intersection
    BOOST_CHECK_CLOSE( PointTarget3.x, interp, 1e-6 );  //Check the x coord of the upper triangle/tab intersection
    BOOST_CHECK_CLOSE( PointTarget.x, interped, 1e-6 ); //Check the x coord of the lower triangle/tab intersection

    PolyTarget.computeIntersection(Big, Sq1);
    PolyTarget.getVertex(0,PointTarget);
    BOOST_CHECK( PolyTarget.getNumVertices() == 4 );
    BOOST_CHECK( PointTarget == R4 );			

    // Check to make sure that there is no intersection for two separate polys	
    PolyEmpty.clearVertices();
    PolyEmpty.computeIntersection(Sq1, Dia);
    BOOST_CHECK_MESSAGE( PolyEmpty.getNumVertices() == 0, "Wholly separate polys shouldn't have intersections" );
    
    // Check to make sure that a poly sitting in the divot of another is does not intersect
    PolyEmpty.clearVertices();
    PolyEmpty.computeIntersection(Concave, Dia);
    BOOST_CHECK_MESSAGE( PolyEmpty.getNumVertices() == 0, 
        "A poly sitting in the divot of another should have no intersection" );

    PolyTarget.clearVertices();  
    PolyTarget.computeIntersection(SimpleConcave, Sq2);
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices() == 0, "Simple concave intersection doesn't work either.");

    PolyTarget.clearVertices();  
    PolyTarget.computeIntersection(OverlapConcave, Sq2);
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices() == 0, 
        "A square partially in the divot of another poly should have no intersection.");


// Test:  computeUnion
    // Relying on the fact that this method sorts the vertices at the end in order to know which vertex to ask for.

    // There is no check on the condition of any overlap for this union. How do two non-overlapping polygons get 
    // unioned?  This seems like an ill-pose problem.  Do we just want to bail if we hit this condition?

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( Backspace, Tab );  // Partial overlap test
    BOOST_CHECK( PolyTarget.getNumVertices()==6 );
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(2,PointTarget2);  PolyTarget.getVertex(4,PointTarget3);  
    BOOST_CHECK( (PointTarget==R4)&(PointTarget2==R6)&(PointTarget3==R2) );
	
    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( Tri, Sq1 );  // Partial overlap test
    BOOST_CHECK( PolyTarget.getNumVertices()==9 );
    PolyTarget.getVertex(1,PointTarget);  PolyTarget.getVertex(2,PointTarget2);  PolyTarget.getVertex(3,PointTarget3);
        PolyTarget.getVertex(7,PointTarget4);
    interp = ((R2.x - R3.x)*2/3) + R3.x;  
    RecPoint2D Top(interp,R2.y);	//Calcing triangle and tab top intersect
    interped = ((R5.x - R4.x)*1/3) + R4.x;  
    RecPoint2D Bottom(interped,R5.y);  //Calcing triangle and tab bottom intersect
    // To account for the ~unclear sorting order when two vertices have same angle.
    BOOST_CHECK( (PointTarget==Bottom)||(PointTarget2==Bottom) );  
    BOOST_CHECK( (PointTarget3==T3)&(PointTarget4==Top) );

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( Big, Tab );  // Full overlap test, large amp values test
    BOOST_CHECK( PolyTarget.getNumVertices()==4 );
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  
        PolyTarget.getVertex(2,PointTarget3);  PolyTarget.getVertex(3,PointTarget4);
    BOOST_CHECK( (PointTarget==B3)&(PointTarget2==B4)&(PointTarget3==B1)&(PointTarget4==B2) );

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( Sq1, Sq2 );  // Common edge test
    BOOST_CHECK( PolyTarget.getNumVertices()==6 );
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(2,PointTarget2); PolyTarget.getVertex(4,PointTarget3);
    BOOST_CHECK( (PointTarget==R4)&(PointTarget2==R6)&(PointTarget3==R2) );

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( Sq1, Sq1 );  // Same polygon test
    BOOST_CHECK( PolyTarget.getNumVertices()==4 );
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(2,PointTarget2); PolyTarget.getVertex(3,PointTarget3);
    BOOST_CHECK( (PointTarget==R4)&(PointTarget2==R2)&(PointTarget3==R3) );

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( Concave, Dia );  // Concave test
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices()==8, 
        "Union with Concave still contains all points, with no overlap" );

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( OverlapConcave, Dia );  // Concave (overlap) test
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices()==8, 
        "Union with OverlapConcave still contains all points, with no overlap" );
    //PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(2,PointTarget2); PolyTarget.getVertex(4,PointTarget3);
    //BOOST_CHECK( (PointTarget==OC3)&(PointTarget2==OC5)&(PointTarget3==OC7) );

    PolyTarget.clearVertices();  
    PolyTarget.computeUnion( SimpleConcave, Dia );  // Concave (simple) test
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices()==8, 
        "Union with SimpleConcave still contains all points, with no overlap" );


// Test:  computeConvexHull
    // NOTE:  computeConvexHull does not sort the points when finished....odd
    MiniConcave.computeConvexHull(PolyTarget);  BOOST_CHECK( PolyTarget.getNumVertices() == 4 );
	
    SimpleConcave.computeConvexHull(PolyTarget);  BOOST_CHECK( PolyTarget.getNumVertices() == 6 ); 
    // Check that it actually got the right points in the convex hull:  (i.e. check the right hand side points)
    PolyTarget.getVertex(3,PointTarget); PolyTarget.getVertex(4,PointTarget2); PolyTarget.getVertex(5,PointTarget3);
    BOOST_CHECK_MESSAGE(( PointTarget==SC3 )&( PointTarget2==SC4)&( PointTarget3==SC7), 
        "Incorrect Points found for convex hull of SimpleConcave." );
	
    Heart.computeConvexHull(PolyTarget); 
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices() == 4 , "Convex hull failed on the Heart");
	

    Complex.computeConvexHull(PolyTarget);  
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices() == 4 , "Convex hull failed on the Complex Convex");
	
    Concave.computeConvexHull(PolyTarget);  
    BOOST_CHECK_MESSAGE( PolyTarget.getNumVertices() == 4 , "Failed on the original Convex");  
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  
        PolyTarget.getVertex(2,PointTarget3);  PolyTarget.getVertex(3,PointTarget4);
        BOOST_CHECK( (PointTarget==B4)&(PointTarget2==B1)&(PointTarget3==B2)&(PointTarget4==B3) ); 
	
    Sq1.computeConvexHull(PolyTarget);  BOOST_CHECK( PolyTarget.getNumVertices() == 4 );
	
    Tri.computeConvexHull(PolyTarget);  BOOST_CHECK( PolyTarget.getNumVertices() == 3 );


// Test:  computeMinEnclosingRectangle
    PolyTarget.computeMinEnclosingRectangle(Sq1);
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  PolyTarget.getVertex(2,PointTarget3);  
        PolyTarget.getVertex(3,PointTarget4);
    BOOST_CHECK( (PointTarget==R4)&(PointTarget2==R5)&(PointTarget3==R2)&(PointTarget4==R3) );
    
    PolyTarget.computeMinEnclosingRectangle(Dia);
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  PolyTarget.getVertex(2,PointTarget3);
        PolyTarget.getVertex(3,PointTarget4);
    BOOST_CHECK_CLOSE( PointTarget.x,R6.x,1e-3 );  BOOST_CHECK_CLOSE( PointTarget.y,R6.y,1e-3 );  
    BOOST_CHECK_CLOSE( PointTarget2.x,D3.x,1e-3 );  BOOST_CHECK_CLOSE( PointTarget2.y,D3.y,1e-3 ); 
    BOOST_CHECK_CLOSE( PointTarget3.x,R1.x,1e-3 );  BOOST_CHECK_CLOSE( PointTarget3.y,R1.y,1e-3 );
    BOOST_CHECK_CLOSE( PointTarget4.x,D2.x,1e-3 );  BOOST_CHECK_CLOSE( PointTarget4.y,D2.y,1e-3 );
	
    PolyTarget.computeMinEnclosingRectangle(MiniConcave);
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  PolyTarget.getVertex(2,PointTarget3);
        PolyTarget.getVertex(3,PointTarget4);
    BOOST_CHECK( (PointTarget==R4)&(PointTarget2==R5)&(PointTarget3==R2)&(PointTarget4==R3) );
    
    PolyTarget.computeMinEnclosingRectangle(Big);
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  PolyTarget.getVertex(2,PointTarget3);
        PolyTarget.getVertex(3,PointTarget4);
    // Can't easily compute the min enclosing rectangle of the four ~random points, so just check to make sure that
    // the points are not contained in the poly
    BOOST_CHECK( (PolyTarget.containsPoint(B1)==0)&(PolyTarget.containsPoint(B2)==0) );
    BOOST_CHECK( (PolyTarget.containsPoint(B3)==0)&(PolyTarget.containsPoint(B4)==0) );
	

    PolyTarget.computeMinEnclosingRectangle(Star);
    PolyTarget.getVertex(0,PointTarget);  PolyTarget.getVertex(1,PointTarget2);  PolyTarget.getVertex(2,PointTarget3);
        PolyTarget.getVertex(3,PointTarget4);
    cout << "Star's Enclosing Rectangle:  This isn't right...Should be (10,0),(0,10),(-10,0),(0,-10)" << endl;
    cout << PointTarget.x << " " << PointTarget.y << endl;
    cout << PointTarget2.x << " " << PointTarget2.y << endl;
    cout << PointTarget3.x << " " << PointTarget3.y << endl;
    cout << PointTarget4.x << " " << PointTarget4.y << endl;
}



void testRecVector2D(void) 
{
    cout << "Running RecVector2D" << endl;
// NOTE: This test intentionally ignores operator testing for now
// NOTE: Test the constructors before setting up variables for rest of testing to avoid confusion on what variables 
//       have been declared.


// Basic Prep
    double dot, length;
    double zero = 0.0;
    RecVector2D NE( 36851.98, 495123.651 );
    RecVector2D NW( 495123.651, -36851.98 );
    RecVector2D SW(-10.11111111,-10.11111111);
    RecVector2D SE(-10.11111111,10.11111111);
    RecVector2D NEInt(2, 5), NWInt(5, -2 ), SWInt(-8,-9), SEInt(8,-9);
    RecVector2D ShortVector;
    RecPoint2D Origin(0.0,0.0);
	
// Test:  Constructor
    RecVector2D BlankVector;  BOOST_CHECK( BlankVector == Origin );
    RecVector2D Vector;  Vector.x = SE.x;  Vector.y = SE.y;
    RecVector2D Vector2(Vector);  BOOST_CHECK( Vector2==SE );
    RecVector2D Vector3(3.5,5.5);  BOOST_CHECK( (Vector3.x == 3.5)&(Vector3.y == 5.5) );
	
// Test:  *Operator ( Dot Product )
    BOOST_CHECK( NE*NW == 0 );
    double dotted = (NE.x*SE.x) + (NE.y*SE.y);  // Simple dot prod check
    BOOST_CHECK_CLOSE( NE*SE, dotted, 1e-9 );


// Test: =Operator
    RecVector2D TargetVector;  
    TargetVector = Vector;  
    BOOST_CHECK( TargetVector == SE );

    TargetVector = NE;
    BOOST_CHECK( TargetVector == NE );

    TargetVector = SW;
    BOOST_CHECK( TargetVector != NE );

    TargetVector = SEInt;
    BOOST_CHECK( TargetVector == SEInt );

    TargetVector = SW;
    BOOST_CHECK( TargetVector == SW );

// Test:  /Operator
    //This operator doens't have any protection against dividing by zero.  Should it?
    TargetVector = NE/zero;
    BOOST_CHECK( TargetVector == NE );    //Make sure divide by zero didn't go through
    BOOST_CHECK_CLOSE( TargetVector.length(), NE.length(), 1e-9 );    //Make sure divide by zero didn't go through
    
    TargetVector = NW/3;
    BOOST_CHECK_CLOSE( TargetVector.x, NW.x/3, 1e-9 );
 

// Test: dot
    dot = NE.x*NW.x + NE.y*NW.y;
    BOOST_CHECK_CLOSE( NE.dot(NW)+1e-6, dot+1e-6, 1e-9 ); //Perp doubles 
    //Needed to add offset since check_close uses PERCENTAGE
    
    dot = NE.x*SEInt.x + NE.y*SEInt.y;
    BOOST_CHECK_CLOSE( NE.dot(SEInt)+1e-6, dot+1e-6, 1e-9 ); 
   
    dot = NE.x*SW.x + NE.y*SW.y;
    BOOST_CHECK_CLOSE( NE.dot(SW)+1e-6, dot+1e-6, 1e-9 ); 
    
    dot = SWInt.x*SEInt.x + SWInt.y*SEInt.y;  //Perp Ints
    BOOST_CHECK_CLOSE( SWInt.dot(SEInt)+1e-6, dot+1e-6, 1e-9 );


// Test: lengthSq and length
    length = (NE.x*NE.x + NE.y*NE.y);
    BOOST_CHECK_CLOSE( NE.lengthSq(), length, 1e-9 );
    length = (SW.x*SW.x + SW.y*SW.y);
    BOOST_CHECK_CLOSE( SW.lengthSq(), length, 1e-9 );
    length = (SEInt.x*SEInt.x + SEInt.y*SEInt.y);
    BOOST_CHECK_CLOSE( SEInt.lengthSq(), length, 1e-9 );
    length = sqrt(NW.x*NW.x + NW.y*NW.y);
    BOOST_CHECK_CLOSE( NW.length(), length, 1e-9 );
    length = sqrt(SW.x*SW.x + SW.y*SW.y);
    BOOST_CHECK_CLOSE( SW.length(), length, 1e-9 );

    BOOST_CHECK_CLOSE( BlankVector.length()+1e-6, 1e-6, 1e-9 );  //length deals with zero vectors fine

// Test: Normalize

// Problem-  We can normalize a zero length vector, but we get an error that we divided by zero.  Should the normalize
//           method catch this for us?  Or do we just want to fix the /Operator?
    ShortVector = BlankVector;
    BOOST_CHECK( ShortVector == Origin );
    ShortVector.normalize();
    BOOST_CHECK( ShortVector == Origin );
    BOOST_CHECK_CLOSE( ShortVector.length(), 0.0, 1e-9 );  //Make sure normalizing doesn't change zero length vector
    
    ShortVector = NE;
    ShortVector.normalize();
    BOOST_CHECK_CLOSE( ShortVector.length(), 1.0, 1e-9 );
}
 
void testRecVector3D(void) 
{
    cout << "Running RecVector3D" << endl;
// NOTE:  This test intentionally ignores Operators

// Basic Prep
    double dot, length;
    double zero = 0;
    RecVector2D NW2D( -495123.651, 36851.98 );
    RecVector3D NEU( 36851.98, 495123.651, 90845.2 );
    RecVector3D NWU( 495123.651, -36851.98, 90845.2 );
    RecVector3D SWD(-10.11111111,-10.11111111, 5645.651 );
    RecVector3D SED(-10.11111111,10.11111111, 5645.651 );
    RecVector3D NEDInt(2, 5,-9), NWDInt(5, -2, -9), SWUInt(-8,-9,11), SEUInt(8,-9,11);
    RecVector3D EmptyVector;
    RecVector3D ShortVector;
    RecPoint3D Origin(0,0,0);
	

// Test:  Constructor
    RecVector3D BlankVector;  BOOST_CHECK( BlankVector == Origin );
    RecVector3D Vector;  Vector.x = SED.x;  Vector.y = SED.y;  Vector.z = SED.z;
    RecVector3D Vector2(Vector);  BOOST_CHECK( Vector2==SED );
    RecVector3D Vector3(NW2D);  BOOST_CHECK( (Vector3.x == NW2D.x)&(Vector3.y == NW2D.y)&(Vector3.z == 0) );
    RecVector3D Vector4(3.5,5.5,-9.5);  BOOST_CHECK( (Vector4.x == 3.5)&(Vector4.y == 5.5)&(Vector4.z == -9.5) );


// Test: =Operator
    RecVector3D TargetVector;  
    TargetVector = Vector;  //assignment to vector  
    BOOST_CHECK( TargetVector == Vector );
    
    TargetVector = Vector;
    BOOST_CHECK( TargetVector == Vector );
	
    TargetVector = NEU;
    BOOST_CHECK( TargetVector == NEU );

    TargetVector = SWD;
    BOOST_CHECK( TargetVector != NEU );

    TargetVector = SEUInt;
    BOOST_CHECK( TargetVector == SEUInt );

    TargetVector = SWD;
    BOOST_CHECK( TargetVector == SWD );


// Test:  /Operator
    //Testing protection against dividing by zero.
    TargetVector = Vector/zero;


// Test:  *Operator in both orders of vector and double
    TargetVector = Vector*3.5;
    RecVector3D TargetVector2 = 3.5*Vector;
    BOOST_CHECK( TargetVector == TargetVector2 );


// Test:  ^Operator for Cross Product
    TargetVector = NEU^SWD;
    TargetVector2.x = NEU.y*SWD.z - NEU.z*SWD.y;
    TargetVector2.y = NEU.z*SWD.x - NEU.x*SWD.z;
    TargetVector2.z = NEU.x*SWD.y - NEU.y*SWD.x;
    BOOST_CHECK( TargetVector == TargetVector2 );

    TargetVector = NEU;
    TargetVector^=SWD;
    BOOST_CHECK( TargetVector == TargetVector2 );


// Test:  cross
    TargetVector = NEU.cross(SWD);
    BOOST_CHECK( TargetVector == TargetVector2 );


// Test: dot and *Operator that returns dot product
    dot = NEU.x*NWU.x + NEU.y*NWU.y + NEU.z*NWU.z;
    BOOST_CHECK_CLOSE( NEU.dot(NWU)+1e-6, dot+1e-6, 1e-9 ); //Perp doubles
    BOOST_CHECK_CLOSE( NEU*NWU+1e-6, dot+1e-6, 1e-9 ); 
    //Needed to add offset since check_close uses PERCENTAGE
    
    dot = NEU.x*SEUInt.x + NEU.y*SEUInt.y + NEU.z*SEUInt.z;
    BOOST_CHECK_CLOSE( NEU.dot(SEUInt)+1e-6, dot+1e-6, 1e-9 ); 
    BOOST_CHECK_CLOSE( NEU*SEUInt+1e-6, dot+1e-6, 1e-9 ); 

    dot = NEU.x*SWD.x + NEU.y*SWD.y + NEU.z*SWD.z;
    BOOST_CHECK_CLOSE( NEU.dot(SWD)+1e-6, dot+1e-6, 1e-9 ); 
    BOOST_CHECK_CLOSE( NEU*SWD+1e-6, dot+1e-6, 1e-9 ); 

    dot = NWDInt.x*NEDInt.x + NWDInt.y*NEDInt.y + NWDInt.z*NEDInt.z;  //Perp Ints
    BOOST_CHECK_CLOSE( NWDInt.dot(NEDInt)+1e-6, dot+1e-6, 1e-9 );
    BOOST_CHECK_CLOSE( NWDInt*NEDInt+1e-6, dot+1e-6, 1e-9 );


// Test: lengthSq and length
    length = (NWU.x*NWU.x + NWU.y*NWU.y + NWU.z*NWU.z );
    BOOST_CHECK_CLOSE( NWU.lengthSq(), length, 1e-9 );
    length = sqrt(length);
    BOOST_CHECK_CLOSE( NWU.length(), length, 1e-9 );

    length = (NEDInt.x*NEDInt.x + NEDInt.y*NEDInt.y + NEDInt.z*NEDInt.z);
    BOOST_CHECK_CLOSE( NEDInt.lengthSq(), length, 1e-9 );
    length = sqrt(length);
    BOOST_CHECK_CLOSE( NEDInt.length(), length, 1e-9 );

    BOOST_CHECK( BlankVector.length() == 0.0 );  //length deals with zero vectors fine


// Test:  Normalize
    ShortVector = EmptyVector;
    // Testing roundabout divide by zero- normalize a zero length vector
    ShortVector.normalize();  // Normalizing a zero length vector will cause a divide by zero unless we protect.l;
    ShortVector = NWU;
    ShortVector.normalize();
    BOOST_CHECK_CLOSE( ShortVector.length(), 1.0, 1e-9 );

    ShortVector = NWDInt;
    ShortVector.normalize();
    BOOST_CHECK_CLOSE( ShortVector.length(), 1.0, 1e-9 );
}



void testRecQuaternion(void) 
{
    cout << "Running RecQuaternion" << endl;
//NOTE:  Unfortunately, this file does all it's work by convertinging the quaternions to 3D Transforms and then using   
//       Transform math.  It misses out on a ton of speed increases.  Do we want to rewrite this?


// Test:  Initialization Checks

    // NOTE: If the Quat -> 3D Trans and Quat -> 3D Pose are not passed unit quaternions, the conversion back to a
    //       quaterion produces junk.
    // NOTE: As of now, the 3D Trans -> Quat appears to have a math error affecting the calculations.  The local copy
    //       of the file was changed, but not yet checked back in.  If passing in Transforms that were made with a unit
    //       quaterion, it does not matter (the error is only in the switch statements)
    // NOTE: The quaterion must also be normalized before calling getAxisAngle.  First, the answer is wrong if the 
    //       quaternoin isn't normalized.  Second, acos doesn't accept numbers > |1|, so you will get back nan 
    //       if |q[3]| > 1.  Only the angle is affected. Should we include a .normalize call at the start of this 
    //       method as a safeguard?
    // NOTE: Given the need to normalize the quaternion in most every situation, should we simply add normalize calls to
    //       every method that doesn't do this already?  Or do we want to rewrite the code so that it doesn't rely on a 
    //       normalized Quat to make the math work?
    cout<<"If you're reading this, we still need to fix the math error in RecQuaternion and deal with the normalization"
	<< " of quaternions issue." << endl;


    // Test to make sure the new quaternion is initialized to the proper values:
    RecQuaternion Quoi;  
    BOOST_CHECK_MESSAGE( (Quoi.quat[0] == 0)&(Quoi.quat[1] == 0)&(Quoi.quat[2] == 0)&(Quoi.quat[3] == 1), 
        "New RecQuaternion not intialized with proper values");


    // Test to see that 3D Trans -> Quat works
    Quoi.quat[0] = .23; Quoi.quat[1] = 6.12; Quoi.quat[2] = 2.9; Quoi.quat[3] = 3.2; 
    // This next line is needed cause the code doing the reverse (Trans->Quat) assumes that you used a unit quat
    Quoi.normalize(); //see above 
    RecTransform3D Trans(Quoi);  // Cheat: using the 3D Trans -> Quat conversion, relies on that being correct
    RecQuaternion TargetQuoi(Trans);

    BOOST_CHECK_CLOSE( TargetQuoi.quat[0], Quoi.quat[0], 1e-6 );
    BOOST_CHECK_CLOSE( TargetQuoi.quat[1], Quoi.quat[1], 1e-6 );
    BOOST_CHECK_CLOSE( TargetQuoi.quat[2], Quoi.quat[2], 1e-6 );
    BOOST_CHECK_CLOSE( TargetQuoi.quat[3], Quoi.quat[3], 1e-6 );

    // Test to see that 3D Pose -> Quat works
    Quoi.quat[0] = 1.57; Quoi.quat[1] = 3.99; Quoi.quat[2] = 6.27; Quoi.quat[3] = 5.2;
    // This next line is needed cause the code doing the reverse (Trans->Quat) assumes that you used a unit quat
    Quoi.normalize();  // see above
    RecPose3D Pose(Quoi);  // Cheat: using the 3D Pose -> Quat conversion, relies on 3D Trans -> Quat being correct     
    RecQuaternion TargetQuoi2(Pose);
    BOOST_CHECK_CLOSE( TargetQuoi2.quat[0], Quoi.quat[0], 1e-6 );
    BOOST_CHECK_CLOSE( TargetQuoi2.quat[3], Quoi.quat[3], 1e-6 );


// Test =Operator
    TargetQuoi2 = TargetQuoi;
    BOOST_CHECK( (TargetQuoi2.quat[0] == TargetQuoi.quat[0])&(TargetQuoi2.quat[1] == TargetQuoi.quat[1]) );
    BOOST_CHECK( (TargetQuoi2.quat[2] == TargetQuoi.quat[2])&(TargetQuoi2.quat[3] == TargetQuoi.quat[3]) ); 


// Test:  get/setAxisAndAngle

    // Check on back and forth passing- should come out the same
    RecVector3D vec, vec2;
	vec.x = 3;  vec.y = 4; vec.z = 5;
    RecRadians phi = 0.5, phi2;
    TargetQuoi.setAxisAndAngle( vec, phi );  // Write the 
    TargetQuoi.getAxisAndAngle( &vec2, &phi2 );  // Get the associated axis and angle from the writen-to quaternion
  
    vec.normalize(); // The vector returned from getAxisAngle is normalized
    BOOST_CHECK_CLOSE( vec.x, vec2.x, 1e-10 );  // The two must be within arg3 PERCENT of each other in order to pass
    BOOST_CHECK_CLOSE( vec.y, vec2.y, 1e-10 );  // doubles represent about 15 digits
    BOOST_CHECK_CLOSE( vec.z, vec2.z, 1e-10 ); 
    BOOST_CHECK_CLOSE( (double)phi, (double)phi2, 1e-10 );


    // getAxisAndAngle check for proper answer:
    Quoi.quat[0] = .23;
    Quoi.quat[1] = .987;
    Quoi.quat[2] = 2.34;
    Quoi.quat[3] = 0.5;

    double mag = sqrt( (Quoi.quat[0]*Quoi.quat[0])+(Quoi.quat[1]*Quoi.quat[1])+(Quoi.quat[2]*Quoi.quat[2]) );
    phi = 2*atan2(mag,Quoi.quat[3]);  // Book equations- different than in code
    vec.x = Quoi.quat[0]/mag;
    vec.y = Quoi.quat[1]/mag;
    vec.z = Quoi.quat[2]/mag;

    Quoi.normalize();  //This is a necessary safeguard to get right answer, or one at all: if q[3] > |1|, it returns nan
    Quoi.getAxisAndAngle(&vec2,&phi2);
    BOOST_CHECK_CLOSE( vec2.x, vec.x, 1e-10 );  
    BOOST_CHECK_CLOSE( vec2.y, vec.y, 1e-10 ); 
    BOOST_CHECK_CLOSE( vec2.z, vec.z, 1e-10 ); 
    BOOST_CHECK_CLOSE( (double)phi2, (double)phi, 1e-10 );

    // setAxisAndAngle check for proper answer:
    vec.x = -654.6351; vec.y = .635;  vec.z = -9568.18;
    vec.normalize();
    phi = -6351.6515;
    TargetQuoi.quat[0] = vec.x*sin(phi/2.0);  //Book equations agree
    TargetQuoi.quat[1] = vec.y*sin(phi/2.0);
    TargetQuoi.quat[2] = vec.z*sin(phi/2.0);
    TargetQuoi.quat[3] = cos(phi/2.0);

    TargetQuoi2.setAxisAndAngle(vec,phi);
    BOOST_CHECK_CLOSE( TargetQuoi2.quat[0], TargetQuoi.quat[0], 1e-10 );  
    BOOST_CHECK_CLOSE( TargetQuoi2.quat[1], TargetQuoi.quat[1], 1e-10 ); 
    BOOST_CHECK_CLOSE( TargetQuoi2.quat[2], TargetQuoi.quat[2], 1e-10 ); 
    BOOST_CHECK_CLOSE( TargetQuoi2.quat[3], TargetQuoi.quat[3], 1e-10 ); 

  
// Test:  Normalize
    TargetQuoi.quat[0] = .23; TargetQuoi.quat[1] = .987; TargetQuoi.quat[2] = 2.34; TargetQuoi.quat[3] = 1.57;
    TargetQuoi.normalize();
    BOOST_CHECK_CLOSE( TargetQuoi.magnitude(), 1.0, 1e-10 );
    TargetQuoi.quat[0] = -618.81; TargetQuoi.quat[1] = 89753.1; TargetQuoi.quat[2] = 1322.34; TargetQuoi.quat[3] = .001;
    TargetQuoi.normalize();
    BOOST_CHECK_CLOSE( TargetQuoi.magnitude(), 1.0, 1e-10 );


// Test:  Magnitude--- this is the first problem- it returned the SQUARE of the magnitude...I fixed it
    Quoi.quat[0] = -1.0; Quoi.quat[1] = -1.0; Quoi.quat[2] = 1.0; Quoi.quat[3] = 1.0;
    BOOST_CHECK_CLOSE(  Quoi.magnitude(),2.0,1e-6 );
    Quoi.quat[0] = 3; Quoi.quat[1] = -1.0; Quoi.quat[2] = 1.0; Quoi.quat[3] = -5.0;
    BOOST_CHECK_CLOSE(  Quoi.magnitude(),6.0,1e-6 );
}


void testRecBox3D(void) 
{
    cout << "Running RecBox3D" << endl;
    // NOTE: It seems best that the constructor from a 3D line uses '<=' in the if-statements.  This biases towards 
    //       constructing the box with point 1 as the minimum point which is more consistant with the other constructors
    // NOTE: The constructor for entering min and max values does not have a check, as the constructor from a line 
    //       segment does.  Do we want to add this?

// Common Prep Area
    RecAxisAlignedBox3D ZeroBox, TargetBox;
    RecPoint3D Origin(0,0,0), TargetPoint1, TargetPoint2, TargetPoint3;
    double minx(-324.34), maxx(875.23), miny(-2435.24), maxy(8975.45), minz(-234.245), maxz(397.0984);
    RecPoint3D face;
    RecPoint3D huge(99999999,99999999,9999999),nhuge(-9e10,-8e10,-7e10),small(4.3,5.7,-9.9);
    RecPoint3D MinPoint;  MinPoint.x = minx;  MinPoint.y = miny;  MinPoint.z = minz;
    RecPoint3D MaxPoint;  MaxPoint.x = maxx;  MaxPoint.y = maxy;  MaxPoint.z = maxz;
    RecPoint3D PosPoint(681635.4,654.65,987354.651);
    RecPoint3D NegPoint(-351.63,-6813.91886,-65138.1);
    RecPoint3D IntPosPoint(684,9624,12478);
    RecPoint3D IntNegPoint(-25671,-6815,-158);
    RecLineSegment3D Line(NegPoint, PosPoint);
    RecLineSegment3D Line2(MaxPoint, MinPoint);          // Line based on doubles, but neg slope
    RecLineSegment3D IntLine(IntPosPoint, IntNegPoint);  // Line based on ints, but neg slope
    RecLineSegment3D PointLine(NegPoint, NegPoint);


// Test:  Initializations  --- May cause errors for some member functions
 
    // Test zero constructor
    BOOST_CHECK( ZeroBox.getMin() == Origin );
    BOOST_CHECK( ZeroBox.getMax() == Origin );

    // Test constructor from mins and maxs
    RecAxisAlignedBox3D Box1(minx,maxx,miny,maxy,minz,maxz);
    BOOST_CHECK( Box1.getMin() == MinPoint );
    BOOST_CHECK( Box1.getMax() == MaxPoint );

    // Test constructor from mins and maxs, but in reverse order-  should fail
    RecAxisAlignedBox3D Box2(maxx,minx,maxy,miny,maxz,minz);
    BOOST_CHECK_MESSAGE( Box2.getMin() == MinPoint, "No correction on min, max point assignment" );
    BOOST_CHECK_MESSAGE( Box2.getMax() == MaxPoint, "No correction on min, max point assignment" );

    // Test constructor from min and max points
    RecAxisAlignedBox3D Box3(NegPoint,PosPoint);
    BOOST_CHECK( Box3.getMin() == NegPoint );
    BOOST_CHECK( Box3.getMax() == PosPoint );

    // Test constructor from line segment
    RecAxisAlignedBox3D NegLineBox(Line2);      // Line with neg slopes
    RecAxisAlignedBox3D IntLineBox(IntLine);    // Line from ints
    RecAxisAlignedBox3D PointBox(PointLine);  // Line with both points same
    BOOST_CHECK( NegLineBox.getMin() == MinPoint );
    BOOST_CHECK( IntLineBox.getMin() == IntNegPoint );
    BOOST_CHECK( PointBox.getMax() == NegPoint );    // Test handling ints and neg slope

    // Test copy constructor
    RecAxisAlignedBox3D Box7(NegLineBox);
    BOOST_CHECK( (Box7.getMin()==NegLineBox.getMin())&(Box7.getMax()==NegLineBox.getMax()) );


//  Test: volume
    double volume = (NegLineBox.getMax().x-NegLineBox.getMin().x)*(NegLineBox.getMax().y-NegLineBox.getMin().y)
                    *(NegLineBox.getMax().z-NegLineBox.getMin().z);
    BOOST_CHECK_CLOSE( volume, NegLineBox.volume(), 1e-6 );
    volume = (PointBox.getMax().x-PointBox.getMin().x)*(PointBox.getMax().y-PointBox.getMin().y)
             *(PointBox.getMax().z-PointBox.getMin().z);
    BOOST_CHECK( volume == PointBox.volume() );


// Test: isInside and is outSide
    BOOST_CHECK( PointBox.isOutside(Origin) );   //Test handling pointbox and origin
    BOOST_CHECK( !IntLineBox.isInside(huge) );  //Test outside and int box
    BOOST_CHECK( IntLineBox.isOutside(huge) );
    BOOST_CHECK( !Box2.isInside(Box2.getMin()) );  //Test if vertices are in box- they should not be
    BOOST_CHECK( !NegLineBox.isOutside(NegLineBox.getMin()) );  //Test if vertices are outside box- they should not be
    BOOST_CHECK( !ZeroBox.isInside(small) );
    BOOST_CHECK( ZeroBox.isOutside(small) );


// Test: isOnbox
    BOOST_CHECK( !ZeroBox.isOnBox(small) );
    BOOST_CHECK( ZeroBox.isOnBox(Origin) );  //Technically, the origin is on the surface of a zero sized box at origin
    BOOST_CHECK( !Box2.isOnBox(small) );
    BOOST_CHECK( !Box2.isOnBox(huge) );
    BOOST_CHECK( Box3.isOnBox(Box3.getMin()) );  //Vertices should be ON box, not in
    face = NegLineBox.getMax();
    face.x /= 2;;
    BOOST_CHECK( NegLineBox.isOnBox(face) );


// Test: getMin, getMax
    // This would be redundant...see above
}



void testRecLineSegment2D(void)
{
    cout << "Running RecLineSegment2D" << endl;
//Note:  This class does not enforce a particular slope of the line...other classes (RecBox2D), sometimes expect it to
    
// Common Prep Area
    RecLineSegment2D ZeroLine;
    RecPoint2D Origin(0,0), NE(2983.8,90734.4), NW(-7489.24,78249.24), SW(-7235.2,-23498.2), SE(324.123,-124.234); 
    RecPoint2D NEInt(10,10), NWInt(-10,10), SWInt(-10,-10), SEInt(10,-10), North(0,10), East(10,0), South(0,-10);
    RecPoint2D West(-10,0);
    RecPoint2D PointTarget1, PointTarget2, PointTarget3;
    RecLineSegment2D NELine(Origin, NE), SWLine(Origin, SW), SELine(Origin, SE);  //NWLine(Origin, NW)- declared later
    RecLineSegment2D NEIntLine(Origin, NE), NWIntLine(Origin, NW), SWIntLine(Origin, SW);
    RecLineSegment2D NSLine(South, North), EWLine(West, East), NtoELine(North, East), NtoWLine(North, West);
    RecPoint2D UpRight(10,1), UpLeft(-10,1), TopRight(1,10), BottomRight(1,-10);
    RecLineSegment2D OffsetEW( UpRight, UpLeft), OffsetNS( BottomRight, TopRight);

 
// Test:  Initialization
    BOOST_CHECK( (ZeroLine.p1() == Origin)&(ZeroLine.p2() == Origin) );   //Default constructor check
    RecLineSegment2D SEIntLine(Origin,SEInt);
    BOOST_CHECK( (SEIntLine.p1() == Origin)&(SEIntLine.p2() == SEInt) );  //2 Point constructor check
    RecLineSegment2D NWLine(Origin,NW);
    BOOST_CHECK( (NWLine.p1() == Origin)&(NWLine.p2() == NW) );  //2 Point constructor check
    RecLineSegment2D TargetLine(NWLine);
    BOOST_CHECK( (TargetLine.p1() == NWLine.p1())&(TargetLine.p2() == NWLine.p2()) );  //Copy constructor check


// Test:  length and lengthSq
    double length = sqrt( (10*10) + (10*10) );
    BOOST_CHECK_CLOSE( SEIntLine.length(), length, 1E-6 );
    length = ( (10*10) + (10*10) );
    BOOST_CHECK_CLOSE( SEIntLine.lengthSq(), length, 1E-6 );
    length = sqrt( (SW.x*SW.x) + (SW.y*SW.y) );
    BOOST_CHECK_CLOSE( SWLine.length(), length, 1E-6 );


// Test:  isPointOnLineSegment   // Won't work right now, moving on...
    //RecPoint2D SWfrac = SW*.85;
    //int check = isPointOnLineSegment( (double)SWfrac.x,(double)SWfrac.y,(double)Origin.x,(double)Origin.y,
    //    (double)SW.x,(double)SW.y  );
    // double length = sqrt( (10*10) + (10*10) );
    // RecPoint2D OnNE(length,length);
    // BOOST_CHECK( NorthEast


// Test: getIntersection  // This isn't working very well, either
    int check = NSLine.getIntersection(EWLine, PointTarget1, PointTarget2);
    BOOST_CHECK( check == 1 );
    PointTarget3.x = 0.0;  PointTarget3.y = 0.0;
    BOOST_CHECK_CLOSE( PointTarget1.x, PointTarget3.x, 1e-6 );
    BOOST_CHECK_CLOSE( PointTarget1.y, PointTarget3.y, 1e-6 );

    check = NEIntLine.getIntersection(NtoELine, PointTarget1, PointTarget2);
    BOOST_CHECK( check == 1 );
    PointTarget3.x = 5.0;  PointTarget3.y = 5.0;
    BOOST_CHECK_CLOSE( PointTarget1.x, PointTarget3.x, 1e-6 );
    BOOST_CHECK_CLOSE( PointTarget1.y, PointTarget3.y, 1e-6 );

    check = NWIntLine.getIntersection(NtoWLine, PointTarget1, PointTarget2);
    BOOST_CHECK( check == 1 );
    PointTarget3.x = -5.0;  PointTarget3.y = 5.0;
    BOOST_CHECK_CLOSE( PointTarget1.x, PointTarget3.x, 1e-6 );
    BOOST_CHECK_CLOSE( PointTarget1.y, PointTarget3.y, 1e-6 );

    check = OffsetEW.getIntersection(OffsetNS, PointTarget1, PointTarget2);
    BOOST_CHECK( check == 1 );
    PointTarget3.x = 1.0;  PointTarget3.y = 1.0;
    BOOST_CHECK_CLOSE( PointTarget1.x, PointTarget3.x, 1e-6 );
    BOOST_CHECK_CLOSE( PointTarget1.y, PointTarget3.y, 1e-6 );
}



void testRecLineSegment3D(void)
{
    cout << "Running LineSegment3D" << endl;
// Common Prep Area
    RecPoint3D Origin( 0.0,0.0,0.0 ), NEU( 3685.9, 493.1, 945.2 ), SWD(-10.11,-10.11, 5645.1 );
    RecPoint3D NWU( 495123.651, -36851.98, 90845.2 ),TargetPoint;
    RecPoint3D SEDInt( (int)-10, (int)25, (int)-45 ),  NWUInt( (int)10, (int)-25, (int)45 );
    RecPoint2D NW2D( -495123.651, 36851.98 ), SE2D( 615.651, -5736.98 );
    RecLineSegment3D EmptyLine;  
    RecLineSegment3D Line3D( NEU, SWD ), Line3D2( SWD, NWU ); 
    RecLineSegment2D Line2D( NW2D, SE2D ); 


// Test Constructors
    BOOST_CHECK( (EmptyLine.p1 == Origin)&(EmptyLine.p2 == Origin) );  // check default constructor
    RecLineSegment3D TargetLine( NEU, SWD );
    BOOST_CHECK(  (TargetLine.p1 == NEU)&(TargetLine.p2 == SWD) );  // check two point constructor
    RecLineSegment3D TargetLine2( Line2D );
    BOOST_CHECK( (TargetLine2.p1.x== NW2D.x)&(TargetLine2.p2.y==SE2D.y)&(TargetLine2.p2.z==0) ); // 2D Line construct
    RecLineSegment3D TargetLine3( TargetLine );
    BOOST_CHECK( ( TargetLine3.p1.x==TargetLine.p1.x )&( TargetLine3.p2.y==TargetLine.p2.y )
                &( TargetLine3.p2.z==TargetLine.p2.z ) );  // 3D Line construct


// Test: length and lengthSq
    TargetPoint = NEU - SWD;
    double lengthSq = (TargetPoint.x*TargetPoint.x) + (TargetPoint.y*TargetPoint.y) + (TargetPoint.z*TargetPoint.z);
    double length = sqrt(lengthSq);
    BOOST_CHECK_CLOSE( lengthSq, Line3D.lengthSq(), 1e-6 );
    BOOST_CHECK_CLOSE( length, Line3D.length(), 1e-6 );

    TargetPoint = NWU - SWD;
    lengthSq = (TargetPoint.x*TargetPoint.x) + (TargetPoint.y*TargetPoint.y) + (TargetPoint.z*TargetPoint.z);
    length = sqrt(lengthSq);
    BOOST_CHECK_CLOSE( lengthSq, Line3D2.lengthSq(), 1e-6 );
    BOOST_CHECK_CLOSE( length, Line3D2.length(), 1e-6 );

    BOOST_CHECK_CLOSE( 0.0, EmptyLine.length(), 1e-9);

}



void testRecRadians(void)
{
    cout << "Running RecRadians" << endl;
// Common Prep Area:
    RecRadians Empty, TargetRad1;


// Test Constructors
    BOOST_CHECK((double) Empty == 0 );                //default constructor
    TargetRad1 = 1.637;
    RecRadians TargetRad2(TargetRad1);  
    BOOST_CHECK( (double)TargetRad2 == (double)TargetRad1 );  //copy constructor- RecRadian
    double check = -2.978;
    RecRadians TargetRad3(check);  
    BOOST_CHECK( (double)TargetRad3 == check );       //copy constructor- double


// Test: /Operator
    check = TargetRad3/0.0;  //divide by zero check
    BOOST_CHECK_CLOSE( (double)TargetRad3, check, 1e-9 );  //make sure it just returns the value uneditted
    
    TargetRad3 = .78933;
    check = TargetRad3/5.0;
    BOOST_CHECK_CLOSE( check, .78933/5.0, 1e-9 );  // check for proper division


// Test: /=Operator
    check = (double)TargetRad3;
    BOOST_CHECK_CLOSE( (double)TargetRad3, check, 1e-9 );  //safety check
    TargetRad3 /= 0.0;  //divide by zero check
    BOOST_CHECK_CLOSE( (double)TargetRad3, check, 1e-9 );//make sure it just returns the value uneditted

    TargetRad3 = .987234;
    check = TargetRad3/3.0;
    TargetRad3 /= 3.0;
    BOOST_CHECK_CLOSE( (double)TargetRad3, check, 1e-9 ); // check for proper division


// Test: +Operator --- check to see that it properly wraps around Pi
    TargetRad1 = PI;
    TargetRad2 = TargetRad1 + PI/4;
    BOOST_CHECK_CLOSE( TargetRad2.getDegrees(), -135.0, 1e-6 );

// Test: -Operator --- check to see it properly wraps around Pi
    TargetRad1 = PI-PI/4;
    TargetRad2 = TargetRad1 - (-PI + PI/4);
    BOOST_CHECK_CLOSE( TargetRad2.getDegrees(), -90.0, 1e-6);

    TargetRad1 = -PI+PI/4;
    TargetRad2 = TargetRad1 - (PI - PI/4);
    BOOST_CHECK_CLOSE( TargetRad2.getDegrees(), 90.0, 1e-6);


// Test: getDegrees  
    TargetRad1 = PI;
    BOOST_CHECK_CLOSE( 180.0, TargetRad1.getDegrees(), 1e-6 );
    TargetRad1 = 1.5*PI;
    BOOST_CHECK_CLOSE( -90.0, TargetRad1.getDegrees(), 1e-6 );


// Test: setDegrees
    TargetRad1.setDegrees(90);
    BOOST_CHECK_CLOSE( PI/2, (double)TargetRad1, 1e-6 );
    
    TargetRad1.setDegrees(0);  
    BOOST_CHECK_CLOSE( 0.0, (double)TargetRad1, 1e-6 );

    TargetRad1.setDegrees(-90);  
    BOOST_CHECK_CLOSE( -PI/2, (double)TargetRad1, 1e-6 );

}


void testRecPose2D(void)  // NOT DONE- JUST QUICK TEST TO CONFIRM THAT POSE -> TRANSFORM -> POSE
{
    cout << "Running RecPose2D" << endl;

    // Test constructors
    RecPose2D Pose;
    BOOST_CHECK( (Pose.x == 0)&(Pose.y == 0)&((double)Pose.rotZ == 0) );

    double xx(89732234.23), yy(-897298724), rzz(.873897);
    RecPose2D Pose2( xx, yy, rzz );
    BOOST_CHECK( (Pose2.x==xx)&(Pose2.y==yy)&((double)Pose2.rotZ==rzz) );

    RecPose2D Pose3(Pose2);
    BOOST_CHECK( (Pose3.x==Pose2.x)&(Pose3.y==Pose2.y)&((double)Pose3.rotZ==(double)Pose2.rotZ) );

    //Test back and forth from a transform
    RecTransform2D Trans(Pose);
    RecPose2D Pose4(Trans);
    BOOST_CHECK_CLOSE( Pose4.x, Pose.x, 1e-6);
    BOOST_CHECK_CLOSE( Pose4.y, Pose.y, 1e-6);
    BOOST_CHECK_CLOSE( (double)Pose4.rotZ, (double)Pose.rotZ, 1e-6);  // Need to type cast or else complier fails

    //Test placement
    Pose.x = 30; Pose.y = 50;  Pose.rotZ = .1;
    BOOST_CHECK( (Pose.x == 30)&(Pose.y == 50)&((double)Pose.rotZ == .1) );

    // No real methods to check.
}



void testRecDifferentialPose2D(void)
{
    cout << "Running RecDifferentialPose2D" << endl;
    RecDifferentialPose2D Pose;
    BOOST_CHECK( (Pose.x == 0)&(Pose.y == 0)&((double)Pose.rotZ == 0) );
 
    double xx(9839734), yy(-209887.2342), rzz(-1.2234);
    Pose.x = xx;  Pose.y = yy;  Pose.rotZ = rzz;
    BOOST_CHECK( (Pose.x == xx)&(Pose.y == yy)&( (double)Pose.rotZ == rzz) );

    RecDifferentialPose2D Pose2(Pose);
    BOOST_CHECK( (Pose2.x == Pose.x)&(Pose2.y == Pose.y)&((double)Pose2.rotZ == (double)Pose.rotZ) ); 

    RecDifferentialPose2D Pose3(xx,yy,rzz);
    BOOST_CHECK( (Pose3.x == xx)&(Pose3.y == yy)&((double)Pose3.rotZ == rzz ) ); 

    // No real methods to check.
}



void testRecPose3D(void)  // NOT DONE- JUST QUICK TEST TO CONFIRM THAT POSE -> TRANSFORM -> POSE
{
    cout << "Running Pseudo-RecPose3D" << endl;
    RecPose3D Pose;
    Pose.x = 30; Pose.y = 50;  Pose.z = 2;  Pose.rot1 = .1;  Pose.rot2 = .2;  Pose.rot3 = 1.57;
    RecTransform3D Trans(Pose);
    RecPose3D Pose2(Trans);
    BOOST_CHECK_CLOSE( Pose2.x, Pose.x, 1e-6);
    BOOST_CHECK_CLOSE( Pose2.y, Pose.y, 1e-6);
    BOOST_CHECK_CLOSE( Pose2.z, Pose.z, 1e-6);
    BOOST_CHECK_CLOSE( (double)Pose2.rot1, (double)Pose.rot1, 1e-6);  // Need to type cast or else complier fails
    BOOST_CHECK_CLOSE( (double)Pose2.rot2, (double)Pose.rot2, 1e-6);
    BOOST_CHECK_CLOSE( (double)Pose2.rot3, (double)Pose.rot3, 1e-6);

    // No real methods to check.
}



test_suite * init_unit_test_suite( int argc, char * argv[] ) 
{
  cout << endl << endl << endl << endl << "*********** New BOOST Run ***********" << endl << endl << endl << endl;
  test_suite* test = BOOST_TEST_SUITE( " Basic Unit Testing File" );
  test->add(BOOST_TEST_CASE(&testRecPoint2D),0);  	//1
  test->add(BOOST_TEST_CASE(&testRecPoint3D),0);  	//2
  //test->add(BOOST_TEST_CASE(&testRecBox2D),0);		//3
  test->add(BOOST_TEST_CASE(&testRecBox3D),0);		//4
  test->add(BOOST_TEST_CASE(&testRecPolygon2D),0);	//5
  test->add(BOOST_TEST_CASE(&testRecVector2D),0);	//6
  test->add(BOOST_TEST_CASE(&testRecVector3D),0);	//7
  //test->add(BOOST_TEST_CASE(&testRecQuaternion),0);	//8
  //test->add(BOOST_TEST_CASE(&testRecLineSegment2D),0);	//9
  test->add(BOOST_TEST_CASE(&testRecLineSegment3D),0);	//10
  test->add(BOOST_TEST_CASE(&testRecRadians),0);	//11
  test->add(BOOST_TEST_CASE(&testRecPose2D),0);		//12
  test->add(BOOST_TEST_CASE(&testRecDifferentialPose2D),0);    //13
  test->add(BOOST_TEST_CASE(&testRecPose3D),0);		//14

  return test;
}


#endif //ifndef _BASICUNITTEST_CC_
