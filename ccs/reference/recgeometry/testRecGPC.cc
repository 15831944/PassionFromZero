/**
 * @file RecKDTree2DUnitTest.cc
 * @author: Christopher Baker (tallbaker@cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */

// so we get our main function
#define BOOST_TEST_DYN_LINK 1
#define BOOST_TEST_MAIN 1
#define BOOST_TEST_ALTERNATIVE_INIT_API 1

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include "recGPC.h"

using boost::unit_test::test_suite;
using boost::unit_test::test_case;
using namespace std;

class IntervalTimer
{
  public:
    IntervalTimer();
    double getInterval();
    void reset();
  private:
    boost::posix_time::ptime tvs;
};

IntervalTimer::IntervalTimer()
{
    reset();
}

void IntervalTimer::reset()
{
    tvs =  boost::posix_time::microsec_clock::universal_time();
}

double IntervalTimer::getInterval()
{
    boost::posix_time::ptime tve;
    tve =  boost::posix_time::microsec_clock::universal_time();
    return 0.001 * (double)(tve-tvs).total_milliseconds();
}

#define TIME(SOMETHING) { IntervalTimer it; SOMETHING; printf(" - %s: %.3lfs\n",#SOMETHING,it.getInterval());}

static RecPolygon2D unitSquare;
static RecPolygon2D unitCircle;
static RecPolygon2D offsetSquare;
static RecPolygon2D largeSquare;
static RecPolygon2D rotatedSquare;

static std::vector<RecPolygon2D> twoSquares1, twoSquares2;

void createTestPolygons()
{
    std::vector<RecPoint2D> vertices;

    // unit square
    vertices.resize(4);
    vertices[0] = RecPoint2D(0,0);
    vertices[1] = RecPoint2D(1,0);
    vertices[2] = RecPoint2D(1,1);
    vertices[3] = RecPoint2D(0,1);

    unitSquare = RecPolygon2D(vertices);

    rotatedSquare = unitSquare.transform(RecTransform2D(RecPose2D(0,0,M_PI/4)));

    
    // offset square
    for (unsigned int ii=0;ii< 4; ++ii)
    {
        vertices[ii].x+=0.5;
        vertices[ii].y+=0.5;
    }
    offsetSquare = RecPolygon2D(vertices);


    vertices[0] = RecPoint2D(0,0);
    vertices[1] = RecPoint2D(2,0);
    vertices[2] = RecPoint2D(2,2);
    vertices[3] = RecPoint2D(0,2);

    largeSquare = RecPolygon2D(vertices);



    // unit circle: 360 pts
    vertices.resize(360);
    RecRadians angle_rad;
    for(unsigned int ii = 0; ii < 360; ++ii)
    {
        angle_rad.setDegrees(ii);
        RecVector2D unitVec = RecVector2D::getUnitVector(angle_rad);
        vertices[ii].x = unitVec.x;
        vertices[ii].y = unitVec.y;
    }

    unitCircle = RecPolygon2D(vertices);


    vertices.resize(4);
    vertices[0] = RecPoint2D(0,0);
    vertices[1] = RecPoint2D(1,0);
    vertices[2] = RecPoint2D(1,1);
    vertices[3] = RecPoint2D(0,1);

    RecPolygon2D tmp(vertices);

    twoSquares1.push_back(tmp);
    for (unsigned int ii=0; ii < 4; ii++)
        vertices[ii].x+= 4;
    RecPolygon2D tmp2(vertices);

    twoSquares1.push_back(tmp2);

    twoSquares2.push_back(tmp2);
    for (unsigned int ii=0; ii < 4; ii++)
        vertices[ii].x+= 4;
    RecPolygon2D tmp3(vertices);    
    twoSquares2.push_back(tmp3);



}



void printRecPoly(const RecPolygon2D &poly, const char *prefix)
{
    for(std::vector<RecPoint2D>::const_iterator jj = poly.getVertices().begin();
        jj != poly.getVertices().end(); ++jj)
    {
        printf("%s%.3lf, %.3lf\n", prefix, jj->x, jj->y);
    }
}

void printRecPolyList(const std::list<RecPolygon2D> &polyList, const char *prefix)
{
    unsigned int ii = 0;
    for(std::list<RecPolygon2D>::const_iterator pp = polyList.begin();
        pp != polyList.end(); ++pp)
    {
        printf("%s %d (%d pts):\n",prefix,ii++,pp->getVertices().size());
        printRecPoly(*pp," ");
    }
}

void printRecPolyVector(const std::vector<RecPolygon2D> &polyList, const char *prefix)
{
    unsigned int ii = 0;
    for(std::vector<RecPolygon2D>::const_iterator pp = polyList.begin();
        pp != polyList.end(); ++pp)
    {
        printf("%s %d (%d pts):\n",prefix,ii++,pp->getVertices().size());
        printRecPoly(*pp," ");
    }
}


// just poke using
void shallowCopyTest(void)
{
    if(RecGPC::usingShallowCopy())
    {
        printf("RecGPC is using Shallow Copy!\n");
    } else {
        printf("RecGPC is NOT using Shallow Copy\n");
    }
}

// self-intersection test
void selfIntersectionTest()
{
    std::list<RecPolygon2D> polyList;
    BOOST_CHECK(RecGPC::computeIntersection(unitSquare,unitSquare,polyList));
    printRecPolyList(polyList,"Unit Square Self-Intersection");
    polyList.clear();
    BOOST_CHECK(RecGPC::computeIntersection(unitCircle,unitCircle,polyList));
    printRecPolyList(polyList,"Unit Circle Self-Intersection");
}

// self-intersection test
void selfUnionTest()
{
    std::list<RecPolygon2D> polyList;
    BOOST_CHECK(RecGPC::computeUnion(unitSquare,unitSquare,polyList));
    printRecPolyList(polyList,"Unit Square Self-Union");
    polyList.clear();
    BOOST_CHECK(RecGPC::computeUnion(unitCircle,unitCircle,polyList));
    printRecPolyList(polyList,"Unit Circle Self-Union");
}

// intersect and union circles and squares
void circleSquareTest()
{
    std::list<RecPolygon2D> polyList;
    BOOST_CHECK(RecGPC::computeUnion(unitSquare,unitCircle,polyList));
    printRecPolyList(polyList,"Unit Square/Circle Union");
    polyList.clear();
    BOOST_CHECK(RecGPC::computeIntersection(unitSquare,unitCircle,polyList));
    printRecPolyList(polyList,"Unit Square/Circle Intersection");
}


void differenceTest()
{
    std::list<RecPolygon2D> polyList;
    BOOST_CHECK(RecGPC::computeDifference(unitSquare,unitSquare,polyList) == false);
    printRecPolyList(polyList,"Unit Square/Square Diff");

    BOOST_CHECK(RecGPC::computeDifference(unitSquare,largeSquare,polyList) == false);
    printRecPolyList(polyList,"Unit Square/largeSquare Diff");

    BOOST_CHECK(RecGPC::computeDifference(unitSquare,rotatedSquare,polyList));
    printRecPolyList(polyList,"Unit Square/rotatedSquare Diff");



    polyList.clear();
    BOOST_CHECK(RecGPC::computeDifference(unitSquare,offsetSquare,polyList));
    printRecPolyList(polyList,"Unit Square/OffsetSquare Diff");

}

void vectorTest()
{
    std::vector<RecPolygon2D> result;
    BOOST_CHECK(RecGPC::computeDifference(twoSquares1, twoSquares1, result) == false);
    printRecPolyVector(result,"two squares Diff");
    result.clear();

    BOOST_CHECK(RecGPC::computeUnion(twoSquares1, twoSquares2, result) );
    printRecPolyVector(result,"two squares Union");
    result.clear();

    BOOST_CHECK(RecGPC::computeDifference(twoSquares1, twoSquares2, result) );
    printRecPolyVector(result,"two squares diff 2");
    result.clear();
    

}


test_suite * init_unit_test_suite(int argc, char *argv[])
{
    createTestPolygons();

    test_suite *test = BOOST_TEST_SUITE("RecGPC Tests");
    test->add(BOOST_TEST_CASE(&shallowCopyTest),0);
    test->add(BOOST_TEST_CASE(&selfIntersectionTest),0);
    test->add(BOOST_TEST_CASE(&selfUnionTest),0);
    test->add(BOOST_TEST_CASE(&circleSquareTest),0);
    test->add(BOOST_TEST_CASE(&differenceTest),0);
    test->add(BOOST_TEST_CASE(&vectorTest),0);
    return test;
}
