/**
 * @file serializeUnitTest.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 09/10/06
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _SERIALIZEUNITTEST_CC_
#define _SERIALIZEUNITTEST_CC_

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

void testRecRadiansSerialization(void)
{
  cout << "Testing recRadians" <<endl;
  RecRadians r(0),r1;
  TEST_RW(r,r1);

  r = 7.6;
  TEST_RW(r,r1);

  r = 2*PI;
  TEST_RW(r,r1);

  r = -2.2*PI;
  TEST_RW(r,r1);

}

void testRecPointVectors(void)
{
  cout << "Testing points/vectors" << endl;
  RecPoint2D p2o(0.1, -6.8), p2i;
  TEST_RW(p2o,p2i);

  RecPoint3D p3o(-9.4, 3.6,2.1), p3i;
  TEST_RW(p2o,p2i);

  RecVector2D v2o(p2o), v2i;
  TEST_RW(v2o,v2i);

  RecVector3D v3o(p3o), v3i;
  TEST_RW(v3o,v3i);

}

void testPose(void)
{
  cout << "Testing pose/transform types" << endl;
  RecPose2D p2o(0.3,-0.6,PI/2), p2i;
  TEST_RW(p2o,p2i);

  RecDifferentialPose2D dp2o(-0.2,-0.34,PI/3), dp2i;
  TEST_RW(dp2o,dp2i);

//  enum RecPose3D::EulerAngleOrder order =
//    enum RecPose3D::EulerAngleOrder::YXY;

  RecPose3D p3o(-0.36, +923942, 9.3,
            PI/3,-PI/2, RecPose3D::YXY), p3i;

  TEST_RW(p3o,p3i);

  RecDifferentialPose3D dp3o(-0.36, +923942, 9.3,
                             PI/3,-PI/2, RecPose3D::XZX), dp3i;
  TEST_RW(dp3o,dp3i);

  RecTransform2D t2o(p2o), t2i;
  TEST_RW(t2o,t2i);

  RecTransform3D t3o(p3o), t3i;
  TEST_RW(t3o,t3i);

  // RecQuaternion doesn't specify an operator==
  // Regardless of what quaternion equality is, here we just want to know that the member variables are set the same
  RecQuaternion qo(t3o), qi;
  save("test.txt",qo);
  load("test.txt",qi);
  for (int i = 0; i < 4; i++)
    BOOST_CHECK_EQUAL(qo.quat[i], qi.quat[i]);
}

void testLinesAndBoxes(void)
{
  cout << "Test lines and boxes" << endl;
  RecPoint2D a(5,-6), b(3,9);
  RecPoint3D c(5,6,7), d(10,23,34);

  RecLineSegment2D l2o(a,b), l2i;
  save("test.txt",l2o);
  load("test.txt",l2i);
  BOOST_CHECK_EQUAL(l2o.p1(),l2i.p1());
  BOOST_CHECK_EQUAL(l2o.p2(),l2i.p2());

  RecLineSegment3D l3o(c,d), l3i;
  save("test.txt",l3o);
  load("test.txt",l3i);
  BOOST_CHECK_EQUAL(l3o.p1,l3i.p1);
  BOOST_CHECK_EQUAL(l3o.p2,l3i.p2);


  RecAxisAlignedBox2D b2o(a,b), b2i;
  save("test.txt",b2o);
  load("test.txt",b2i);
  BOOST_CHECK_EQUAL(b2o.getMinPoint(),b2i.getMinPoint());
  BOOST_CHECK_EQUAL(b2o.getMaxPoint(),b2i.getMaxPoint());

  RecAxisAlignedBox3D b3o(c,d), b3i;
  save("test.txt",b3o);
  load("test.txt",b3i);
  BOOST_CHECK_EQUAL(b3o.getMin(),b3i.getMin());
  BOOST_CHECK_EQUAL(b3o.getMax(),b3i.getMax());

}

void testPolygon(void)
{
  cout << "Testing polygon" << endl;
  RecPolygon2D polyOut, polyIn;
  RecPoint2D p(0,0);
  polyOut.addVertex(p);
  p.x = 10; p.y =10;
  polyOut.addVertex(p);
  p.x = -10; p.y = 10;
  polyOut.addVertex(p);

  save("test.txt",polyOut);
  load("test.txt",polyIn);

  RecPoint2D a,b;
  polyOut.getCenter(a); polyIn.getCenter(b);
  cout << "center: " << a <<" " << b << endl;
  BOOST_CHECK_EQUAL(a, b);

  polyOut.getVertex(0, a); polyIn.getVertex(0, b);
  cout << "v: " << a <<" " << b << endl;
  BOOST_CHECK_EQUAL(a, b);
  polyOut.getVertex(1, a); polyIn.getVertex(1, b);
  cout << "v: " << a <<" " << b << endl;
  BOOST_CHECK_EQUAL(a, b);
  polyOut.getVertex(2, a); polyIn.getVertex(2, b);
  cout << "v: " << a <<" " << b << endl;
  BOOST_CHECK_EQUAL(a, b);

}

#include <unistd.h>
#include <sys/time.h>

class IntervalTimer
{
  public:
    IntervalTimer();
    double getInterval();
    void reset();
  private:
    struct timeval tvs;
};

IntervalTimer::IntervalTimer()
{
    reset();
}

void IntervalTimer::reset()
{
    gettimeofday(&tvs,NULL);
}

double IntervalTimer::getInterval()
{
    struct timeval tve;
    gettimeofday(&tve,NULL);
    return (double)(tve.tv_sec-tvs.tv_sec) + 0.000001 * (double)(tve.tv_usec-tvs.tv_usec);
}

void buildUnitCircle(RecPolygon2D &out, const double &step, const RecVector2D &offset)
{
    std::vector<RecVector2D> pointVec;
    pointVec.reserve(1 + (int)(360.0/step));
    for(double angle = 0.0; angle < 360.0; angle += step)
    {
        RecRadians angle_rad;
        angle_rad.setDegrees(angle);
        pointVec.push_back(offset + RecVector2D::getUnitVector(angle_rad));
    }
    out = RecPolygon2D(pointVec);
}

void testPolygonStress(void)
{
    // make a big, dense circle
    RecPolygon2D bigCircle;
    RecPolygon2D otherCircle;
    IntervalTimer it;
    for(int ii=0; ii<10; ++ii)
    {
        buildUnitCircle(bigCircle,0.125,RecVector2D(ii,ii));
        save("test.txt",bigCircle);
        it.reset();
        load("test.txt",otherCircle);
        printf(" - Deserialize %d: %.3lfs\n",ii+1,it.getInterval());
        BOOST_CHECK_EQUAL(bigCircle.getNumVertices(),otherCircle.getNumVertices());
    }
}

test_suite * init_unit_test_suite( int argc, char * argv[] )
{
  test_suite* test = BOOST_TEST_SUITE( "recGeometry serialization tests" );
  test->add(BOOST_TEST_CASE(&testRecRadiansSerialization),0);
  test->add(BOOST_TEST_CASE(&testRecPointVectors),0);
  test->add(BOOST_TEST_CASE(&testPose),0);
  test->add(BOOST_TEST_CASE(&testLinesAndBoxes),0);
  test->add(BOOST_TEST_CASE(&testPolygon),0);
  test->add(BOOST_TEST_CASE(&testPolygonStress),0);
  return test;
}


#endif //ifndef _SERIALIZEUNITTEST_CC_
