/**
 * @file TimeStampTest.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 11/11/06
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _TIMESTAMPTEST_CC_
#define _TIMESTAMPTEST_CC_

// so we get our main function
#define BOOST_TEST_DYN_LINK 1
#define BOOST_TEST_MAIN 1
#define BOOST_TEST_ALTERNATIVE_INIT_API 1


#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include "TimeStamp.h"
using boost::unit_test::test_suite;
using boost::unit_test::test_case;
//using namespace boost::posix_time;
using namespace std;


void timeCheckEqual(ptime a, ptime b, time_duration accuracy)
{
    time_duration deltaT = a-b;
    if (deltaT.is_negative())
        deltaT.invert_sign();
    cout << "comparing " << a << " and " << b << " for accuracy better than: " << accuracy << endl;
    BOOST_CHECK(deltaT < accuracy);

}


void basicOps(void)
{
    cout << "checking to and from double are correct conversions" << endl;
    ptime t= boost::posix_time::microsec_clock::universal_time();
    long double ldoubleTime = convertToSeconds(t);
    cout << "long double of time is: "<< ldoubleTime << endl;
    ptime testT = convertToptime(ldoubleTime);

    timeCheckEqual(t,testT, microseconds(1));

    double doubleTime = convertToSeconds(t);
    testT = convertToptime(doubleTime);
    timeCheckEqual(t,testT,milliseconds(1));

    cout << "checking elapsed time is correct" << endl;

    const static ptime epoch(boost::gregorian::date(1970,1,1));
    long double ets = elapsedTimeInSeconds(epoch,t);
    time_duration td = secondsToDuration(ets);
    long double postConvert = toSeconds(td);
    BOOST_CHECK_EQUAL(ets,postConvert);


}


test_suite * init_unit_test_suite( int argc, char * argv[] ) 
{
    test_suite *test = BOOST_TEST_SUITE( "TimeStamp Tests" );
    test->add(BOOST_TEST_CASE(&basicOps),0);
    return test;
}





#endif //ifndef _TIMESTAMPTEST_CC_
