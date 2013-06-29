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
#include "RecKDTree2D.h"

using boost::unit_test::test_suite;
using boost::unit_test::test_case;
using namespace std;

typedef std::vector< std::pair< RecPoint2D, double > > RawElementVector;

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

#define TIME(SOMETHING) { IntervalTimer it; SOMETHING; printf(" - %s: %.3lfs\n",#SOMETHING,it.getInterval());}

double IntervalTimer::getInterval()
{
    boost::posix_time::ptime tve;
    tve =  boost::posix_time::microsec_clock::universal_time();
    return 0.001 * (double)(tve-tvs).total_milliseconds();
}

// these functions should not do dumb things if no tree is built
void nullProtectionTest(void)
{
    RecKDTree2D<double> doubleTree;
    double dd;
    RecKDTree2D<double>::PayloadVector payloadVector;

    RecPoint2D queryPoint(0,0);

    bool getClosestElement = doubleTree.getClosestElement(queryPoint,dd);
    BOOST_CHECK(!getClosestElement);
    bool getElementsWithin = doubleTree.getElementsWithin(queryPoint,1.0,payloadVector);
    BOOST_CHECK(!getElementsWithin);
    BOOST_CHECK_EQUAL(payloadVector.size(),(unsigned)0);
}

// build a simple tree and run some simple tests
void simpleConstructionTest()
{
    const double interval = 1.0;
    const int count = 15;
    RecKDTree2D<double> doubleTree;

    // make a tree
    doubleTree.reserve(count);
    for(int ii=0; ii < count; ++ii)
    {
        const double dd = interval * (double)(ii);
        const double expectedValue = dd*dd;
        bool addElement = doubleTree.addElement(RecPoint2D(-dd,dd),expectedValue);
        BOOST_CHECK(addElement);
    }

    bool buildTree = doubleTree.buildTree();
    BOOST_CHECK(buildTree);

    // query the tree for specific points
    for(int ii=0; ii < count; ++ii)
    {
        const double dd = interval * (double)(ii);
        const double expectedValue = dd*dd;
        double treeValue = 0;
        bool getClosestElement = doubleTree.getClosestElement(RecPoint2D(-dd,dd),treeValue);
        BOOST_CHECK(getClosestElement);
        BOOST_CHECK_EQUAL(expectedValue,treeValue);
    }

    // query the tree for groups small groups of points
    for(int ii=1; ii < count-1; ++ii)
    {
        std::vector<double> expectedValues;
        for(int jj=ii-1; jj<=ii+1; ++jj)
        {
            const double dd = interval * (double)(jj);
            expectedValues.push_back(dd * dd);
        }

        const double dd = interval * (double)(ii);
        std::vector<double> treeValues;
        bool getElementsWithin = doubleTree.getElementsWithin(RecPoint2D(-dd,dd),2.0 * interval,treeValues);
        BOOST_CHECK(getElementsWithin);
        BOOST_CHECK_EQUAL(expectedValues.size(),treeValues.size());

        for(std::vector<double>::const_iterator vv = expectedValues.begin();
            vv != expectedValues.end(); ++vv)
        {
            bool found = std::find(treeValues.begin(),treeValues.end(),*vv) != treeValues.end();
            BOOST_CHECK(found);
        }
    }

    // make sure the various addElements do not work once the tree is built
    bool addElement = doubleTree.addElement(RecPoint2D(0,0),0);
    BOOST_CHECK(!addElement);
    bool addElements = doubleTree.addElements(RawElementVector());
    BOOST_CHECK(!addElements);

    // try clearing the tree
    bool clearTree = doubleTree.clearTree();
    BOOST_CHECK(clearTree);

    // make sure addElement works again
    addElement = doubleTree.addElement(RecPoint2D(0,0),0);
    BOOST_CHECK(addElement);
}


bool buildLinearElements(const double &interval, const int count,
                         RawElementVector &rawElements)
{
    rawElements.reserve(count);
    for(int ii=0; ii < count; ++ii)
    {
        const double dd = interval * (double)(ii);
        const double expectedValue = dd*dd;
        rawElements.push_back(std::make_pair(RecPoint2D(-dd,dd),expectedValue));
    }
    return true;
}

bool naiveGetClosestElement(const RecPoint2D &queryPoint,
                            const RawElementVector &rawElements,
                            double &out)
{
    if(rawElements.size() > 0)
    {
        double closestDistanceSq=INFINITY;
        for(RawElementVector::const_iterator ii=rawElements.begin();
            ii != rawElements.end(); ++ii)
        {
            double dd = ii->first.distanceSq(queryPoint);
            if(dd < closestDistanceSq)
            {
                closestDistanceSq = dd;
                out = ii->second;
            }
        }
        return true;
    } else {
        return false;
    }
}

bool naiveGetElementsWithin(const RecPoint2D &queryPoint,
                            const RawElementVector &rawElements,
                            const double &radius,
                            std::vector<double> &out)
{
    bool changed = false;
    const double radiusSq = radius * radius;
    for(RawElementVector::const_iterator ii=rawElements.begin();
        ii != rawElements.end(); ++ii)
    {
        double dd = ii->first.distanceSq(queryPoint);
        if(dd <= radiusSq)
        {
            out.push_back(ii->second);
            changed = true;
        }
    }
    return changed;
}

void orderedInputTimingTest()
{
    printf("\nRunning orderedInputTimingTest\n");
    for(unsigned int count=100; count <= 10000; count*=10)
    {
        RecKDTree2D<double> doubleTree;
        RawElementVector rawElements;
        buildLinearElements(1.0,count,rawElements);
        printf("Count = %d\n",count);
        TIME(doubleTree.addElements(rawElements));
        TIME(doubleTree.buildTree());
        IntervalTimer timer;
        for(RawElementVector::const_iterator ii=rawElements.begin();
            ii != rawElements.end(); ++ii)
        {
            double naiveExpected = ii->second;
            double naiveLookup = 0;
            bool foundNaive = naiveGetClosestElement(ii->first,rawElements,naiveLookup);
            BOOST_CHECK(foundNaive);
            BOOST_CHECK_EQUAL(naiveLookup,naiveExpected);
        }
        printf(" - Naive getClosest: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=rawElements.begin();
            ii != rawElements.end(); ++ii)
        {
            double treeExpected = ii->second;
            double treeLookup = 0;
            bool foundTree = doubleTree.getClosestElement(ii->first,treeLookup);
            BOOST_CHECK(foundTree);
            BOOST_CHECK_EQUAL(treeLookup,treeExpected);
        }
        printf(" - Tree  getClosest: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=rawElements.begin();
            ii != rawElements.end(); ++ii)
        {
            std::vector<double> naiveLookup;
            bool foundNaive = naiveGetElementsWithin(ii->first,rawElements,1.5,naiveLookup);
            BOOST_CHECK(foundNaive);
            unsigned int expectedSize = 3;
            if(ii == rawElements.begin() || (ii+1) == rawElements.end())
                expectedSize = 2;
            BOOST_CHECK_EQUAL(naiveLookup.size(),expectedSize);
        }
        printf(" - Naive getWithin: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=rawElements.begin();
            ii != rawElements.end(); ++ii)
        {
            std::vector<double> treeLookup;
            bool foundTree = doubleTree.getElementsWithin(ii->first,1.5,treeLookup);
            BOOST_CHECK(foundTree);
            unsigned int expectedSize = 3;
            if(ii == rawElements.begin() || (ii+1) == rawElements.end())
                expectedSize = 2;
            BOOST_CHECK_EQUAL(treeLookup.size(),expectedSize);
        }
        printf(" - Tree  getWithin: %.3lfs\n",timer.getInterval());
    }
}

bool buildRandomizedElements(const int count,
                             RawElementVector &rawElements)
{
    rawElements.reserve(count);
    for(int ii=0; ii < count; ++ii)
    {
        RecPoint2D pt(rand(),rand());
        const double expectedValue = pt.x * pt.y;
        rawElements.push_back(std::make_pair(pt,expectedValue));
    }
    return true;
}

void randomInputTimingTest()
{
    printf("\nRunning randomInputTimingTest\n");
    for(unsigned int count=100; count <= 10000; count*=10)
    {
        RecKDTree2D<double> doubleTree;
        RawElementVector rawElements;
        srand(0xDEADBEEF);
        buildRandomizedElements(count,rawElements);
        // and we want to query with some other random bunch
        RawElementVector queryElements;
        buildRandomizedElements(count,queryElements);
        printf("Count = %d\n",count);
        TIME(doubleTree.addElements(rawElements));
        TIME(doubleTree.buildTree());
        IntervalTimer timer;
        for(RawElementVector::const_iterator ii=queryElements.begin();
            ii != queryElements.end(); ++ii)
        {
            double naiveLookup = 0;
            bool foundNaive = naiveGetClosestElement(ii->first,rawElements,naiveLookup);
            BOOST_CHECK(foundNaive);
        }
        printf(" - Naive getClosest: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=queryElements.begin();
            ii != queryElements.end(); ++ii)
        {
            double treeLookup = 0;
            bool foundTree = doubleTree.getClosestElement(ii->first,treeLookup);
            BOOST_CHECK(foundTree);
        }
        printf(" - Tree  getClosest: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=queryElements.begin();
            ii != queryElements.end(); ++ii)
        {
            std::vector<double> naiveLookup;
            naiveGetElementsWithin(ii->first,rawElements,1.5,naiveLookup);
        }
        printf(" - Naive getWithin: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=queryElements.begin();
            ii != queryElements.end(); ++ii)
        {
            std::vector<double> treeLookup;
            doubleTree.getElementsWithin(ii->first,1.5,treeLookup);
        }
        printf(" - Tree  getWithin: %.3lfs\n",timer.getInterval());
    }
}

void randomInputEquivalenceTest()
{
    printf("\nRunning randomInputEquivalenceTest\n");
    RecKDTree2D<double> doubleTree;
    RawElementVector rawElements;
    srand(0xDEADBEEF);
    buildRandomizedElements(1000,rawElements);
    // and we want to query with some other random bunch
    RawElementVector queryElements;
    buildRandomizedElements(1000,queryElements);
    TIME(doubleTree.addElements(rawElements));
    TIME(doubleTree.buildTree());
    for(RawElementVector::const_iterator ii=queryElements.begin();
        ii != queryElements.end(); ++ii)
    {
        double naiveLookup = 0;
        bool foundNaive = naiveGetClosestElement(ii->first,rawElements,naiveLookup);
        BOOST_CHECK(foundNaive);
        double treeLookup = 0;
        bool foundTree = doubleTree.getClosestElement(ii->first,treeLookup);
        BOOST_CHECK(foundTree);
        BOOST_CHECK_EQUAL(naiveLookup,treeLookup);
    }
    for(RawElementVector::const_iterator ii=queryElements.begin();
        ii != queryElements.end(); ++ii)
    {
        std::vector<double> naiveLookupVec;
        naiveGetElementsWithin(ii->first,rawElements,1.5,naiveLookupVec);
        std::vector<double> treeLookupVec;
        doubleTree.getElementsWithin(ii->first,1.5,treeLookupVec);
        BOOST_CHECK_EQUAL(naiveLookupVec.size(),treeLookupVec.size());
        if(naiveLookupVec.size() == treeLookupVec.size())
        {
            for(unsigned int jj=0; jj<naiveLookupVec.size(); ++jj)
            {
                BOOST_CHECK_EQUAL(naiveLookupVec[jj], treeLookupVec[jj]);
            }
        }
    }
}

void enormousTreeTest()
{
    printf("\nRunning enormousTreeTest\n");
    for(unsigned int count=100000; count <= 10000000; count*=10)
    {
        RecKDTree2D<double> doubleTree;
        RawElementVector rawElements;
        srand(0xDEADBEEF);
        buildRandomizedElements(count,rawElements);
        // and we want to query with a smaller set of some other random bunch
        RawElementVector queryElements;
        buildRandomizedElements(count/100,queryElements);
        printf("Count = %d\n",count);
        TIME(doubleTree.addElements(rawElements));
        TIME(doubleTree.buildTree());
        IntervalTimer timer;
        for(RawElementVector::const_iterator ii=queryElements.begin();
            ii != queryElements.end(); ++ii)
        {
            double treeLookup = 0;
            bool foundTree = doubleTree.getClosestElement(ii->first,treeLookup);
            BOOST_CHECK(foundTree);
        }
        printf(" - Tree  getClosest: %.3lfs\n",timer.getInterval());
        timer.reset();
        for(RawElementVector::const_iterator ii=queryElements.begin();
            ii != queryElements.end(); ++ii)
        {
            std::vector<double> treeLookup;
            doubleTree.getElementsWithin(ii->first,1.5,treeLookup);
        }
        printf(" - Tree  getWithin: %.3lfs\n",timer.getInterval());
    }
}

test_suite * init_unit_test_suite(int argc, char *argv[])
{
    test_suite *test = BOOST_TEST_SUITE("RecKDTree2D Tests");
    test->add(BOOST_TEST_CASE(&nullProtectionTest),0);
    test->add(BOOST_TEST_CASE(&simpleConstructionTest),0);
    test->add(BOOST_TEST_CASE(&orderedInputTimingTest),0);
    test->add(BOOST_TEST_CASE(&randomInputTimingTest),0);
    test->add(BOOST_TEST_CASE(&randomInputEquivalenceTest),0);
    test->add(BOOST_TEST_CASE(&enormousTreeTest),0);
    return test;
}
