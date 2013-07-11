/**
 * @file mapUnitTest.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _MAPUNITTEST_CC_
#define _MAPUNITTEST_CC_
#include <iostream>
#include <fstream>

// so we get our main function
#define BOOST_TEST_DYN_LINK 1
#define BOOST_TEST_MAIN 1
#define BOOST_TEST_ALTERNATIVE_INIT_API 1

#include <boost/test/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include "Map.h"

using boost::unit_test::test_suite;
using boost::unit_test::test_case;
using boost::archive::text_oarchive;
using boost::archive::text_iarchive;
using namespace std;

template <class T>
void saveMemory(const T &to_save, std::stringstream &serializedValueStream)
{
    ptime before = boost::posix_time::microsec_clock::local_time();
    boost::archive::binary_oarchive oa(serializedValueStream);
    oa << to_save;
    ptime after = boost::posix_time::microsec_clock::local_time();
    cout << "saveMemory took: " << (after-before) << endl;

}

template <class T>
void loadMemory(T &to_load, std::stringstream &serializedValueStream)
{
    ptime before = boost::posix_time::microsec_clock::local_time();
    boost::archive::binary_iarchive ia(serializedValueStream);
    ia >> to_load;
    ptime after = boost::posix_time::microsec_clock::local_time();
    cout << "load Memory took: " << (after-before) << endl;
}

template <class T>
void save(const string & name, const T & to_save)
{
    ofstream ofs(name.c_str());
    text_oarchive oa(ofs);
    ptime before = boost::posix_time::microsec_clock::local_time();
    oa << to_save;
    ptime after = boost::posix_time::microsec_clock::local_time();
//    cout << "save took: " << (after-before) << endl;
}

template <class T>
void load(const string & name, T & to_load)
{
    ifstream ifs(name.c_str());
    text_iarchive ia(ifs);
    ptime before = boost::posix_time::microsec_clock::local_time();
    ia >> to_load;
    ptime after = boost::posix_time::microsec_clock::local_time();
//    cout << "load took: " << (after-before) << endl;
}

void changeMapTest(void)
{
    DoubleMap testMap(RecPoint2D(0,0),10,10,0.25);
    for (DoubleMap::iterator it = testMap.begin();it!=testMap.end(); ++it)
        *it = 5;
    DoubleMap testMap2(testMap);
    testMap2.setOrigin(RecPoint2D(2,3));
    BoolMap delta(RecPoint2D(2,3),10,10,0.25);
    
    testMap.calculateChangeMap(testMap2,delta);
    // x,y in the region from -3->5,-2->5 should be marked with no change, every other square should indicate change
    RecPoint2D index;
    RecAxisAlignedBox2D bounds = delta.getBounds();
    for (index.x = bounds.getMinPoint().x; index.x <bounds.getMaxPoint().x; index.x+=0.25)
    {
        for (index.y = bounds.getMinPoint().y; index.y < bounds.getMaxPoint().y; index.y+=0.25)
        {
            if(index.x >=-3 && index.x <5 && index.y>=-2 && index.y<5)
                BOOST_CHECK(!delta(index));
            else
                BOOST_CHECK(delta(index));
        }
    }


}

void mapTest(void) 
{
    cout << "Testing iterators and indexing" << endl;
    DoubleMap testMap(RecPoint2D(0,0.0),100.0,100.0,0.25);
    // check errors around 0
    testMap(RecPoint2D(-0.1,-0.1)) = 1;
    testMap(RecPoint2D(-0.1,0.1)) = 2;
    testMap(RecPoint2D(0.1,-0.1)) = 3;
    testMap(RecPoint2D(0.1,0.1))=4;
    BOOST_CHECK_EQUAL(1,testMap(RecPoint2D(-0.1,-0.1)));
    BOOST_CHECK_EQUAL(2,testMap(RecPoint2D(-0.1,0.1)));
    BOOST_CHECK_EQUAL(3,testMap(RecPoint2D(0.1,-0.1)));
    BOOST_CHECK_EQUAL(4,testMap(RecPoint2D(0.1,0.1)));

    DoubleMap testMap2(testMap);
    // check again after copy
    BOOST_CHECK_EQUAL(1,testMap2(RecPoint2D(-0.1,-0.1)));
    BOOST_CHECK_EQUAL(2,testMap2(RecPoint2D(-0.1,0.1)));
    BOOST_CHECK_EQUAL(3,testMap2(RecPoint2D(0.1,-0.1)));
    BOOST_CHECK_EQUAL(4,testMap2(RecPoint2D(0.1,0.1)));

    testMap(RecPoint2D(49.95,49.95))=20;
    BOOST_CHECK_EQUAL(20,testMap(RecPoint2D(49.95,49.95)));
    testMap(RecPoint2D(-50,49.95))=20;
    BOOST_CHECK_EQUAL(20,testMap(RecPoint2D(-50,49.95)));
    testMap(RecPoint2D(-50,-50))=20;
    BOOST_CHECK_EQUAL(20,testMap(RecPoint2D(-50,-50)));
    testMap(RecPoint2D(49.95,-50))=20;
    BOOST_CHECK_EQUAL(20,testMap(RecPoint2D(49.95,-50)));

    // check setting over the map
    DoubleMap::iterator mapIt;
    int numSteps=0;
    for (mapIt = testMap.begin(); mapIt!=testMap.end(); ++mapIt, numSteps++)
    {
        *mapIt = 1000.0;
    }

    BOOST_CHECK_EQUAL(numSteps,400*400);

    // check values are set
    RecPoint2D index;
    for (index.x = -50; index.x<50; index.x+=0.25)
    {
        for (index.y = -50; index.y<50; index.y+=0.25)
         {
             BOOST_CHECK_EQUAL(testMap(index),1000);
         }
    }

    //check setting a window in the map
    for (mapIt = testMap.begin(RecPoint2D(-10,-10),RecPoint2D(0,20)), numSteps=0; 
         mapIt != testMap.end(); ++mapIt, ++numSteps)
    {
        *mapIt = 2000.0;
    }



    // ensure we touched the expected number of cells
    BOOST_CHECK_EQUAL(numSteps,10*4*30*4);

    //now check the map to make sure we set the map cells we thought we touched
    for (index.x = -50; index.x<50; index.x+=0.25)
    {
        for (index.y = -50; index.y<50; index.y+=0.25)
        {

            if (index.x>=-10 && index.x<0 && index.y>=-10 && index.y<20)
                BOOST_CHECK_EQUAL(testMap(index),2000);
            else
                BOOST_CHECK_EQUAL(testMap(index),1000);
         }
    }

    testMap.setUseShallowCopy();
    DoubleMap testMap3(testMap);
    // check stats after copy
    for (index.x = -50; index.x<50; index.x+=0.25)
    {
        for (index.y = -50; index.y<50; index.y+=0.25)
        {

            if (index.x>=-10 && index.x<0 && index.y>=-10 && index.y<20)
                BOOST_CHECK_EQUAL(testMap3(index),2000);
            else
                BOOST_CHECK_EQUAL(testMap3(index),1000);
         }
    }

    

}

void mapSerializeTest(void)
{
    cout <<"testing map serialization" << endl;
    DoubleMap testMap(RecPoint2D(0,0.0),100.0,100.0,0.25);

    DoubleMap::iterator mapIt;
    int numSteps=0;
    for (mapIt = testMap.begin(); mapIt!=testMap.end(); ++mapIt, numSteps++)
    {
        *mapIt = numSteps;
    }


    DoubleMap testMap2;
    cout << "Testing element-by-element write" << endl;
    save("test_plain.txt",testMap);
    load("test_plain.txt",testMap2);




    numSteps=0;
    for (mapIt = testMap2.begin(); mapIt!=testMap2.end(); ++mapIt, numSteps++)
    {
        BOOST_CHECK_EQUAL(*mapIt,numSteps);
    }

    for (int i=0;i<10;i++) 
    {
        std::stringstream serializedValueStream1;
        saveMemory(testMap,serializedValueStream1);
        loadMemory(testMap2,serializedValueStream1);
    }

    cout << endl << "Testing shallow copy write" << endl;

    // test shallow version of serialize
    DoubleMap testMap3;
    testMap2.setUseShallowCopy();
    save("test_shallow.txt",testMap2);
    load("test_shallow.txt",testMap3);
    
    for (int i=0;i<10;i++) 
    {
        std::stringstream serializedValueStream2;
        saveMemory(testMap2,serializedValueStream2);
        loadMemory(testMap3,serializedValueStream2);
    }

    numSteps=0;
    for (mapIt = testMap3.begin(); mapIt!=testMap3.end(); ++mapIt, numSteps++)
    {
        BOOST_CHECK_EQUAL(*mapIt,numSteps);
    }

    // test compress version of serialize
    DoubleMap testMap4;
    testMap2.setCompressOnSeriallize();

    cout << endl << "Testing compressed write" << endl;
    save("test_compressed.txt",testMap2);
    load("test_compressed.txt",testMap4);

    numSteps=0;
    for (mapIt = testMap4.begin(); mapIt!=testMap4.end(); ++mapIt, numSteps++)
    {
        BOOST_CHECK_EQUAL(*mapIt,numSteps);
    }

    
    for (int i=0;i<10;i++) 
    {
        std::stringstream serializedValueStream3;
        saveMemory(testMap2,serializedValueStream3);
        loadMemory(testMap4,serializedValueStream3);
    }

}

void lineTest(void)
{
    DoubleMap testMap(RecPoint2D(0,0.0),100.0,100.0,0.25);
    DoubleMap::LineIterator lit;
    
    RecPoint2D index;
    double val = 536;
    index.y = 3;
    for (index.x =15; index.x <= 35; index.x+=0.25, val++)
        testMap(index) = val;
    
    cout << "horizontal line check" << endl;
    val = 536;
    for(lit= testMap.beginLine(RecPoint2D(15,3),
                           RecPoint2D(35,3));
        lit!=testMap.endLine();
        ++lit, val++)
        BOOST_CHECK_EQUAL(*lit,val);


    val = 17;
    index.y = 7;
    for (index.x =15; index.x >= -30; index.x-=0.25, val++)
        testMap(index) = val;
    
    val = 17;
    for(lit= testMap.beginLine(RecPoint2D(15,7),
                           RecPoint2D(-30,7));
        lit!=testMap.endLine();
        ++lit, val++)
        BOOST_CHECK_EQUAL(*lit,val);


    val =15;
    index.x= 35;
    cout << "vertical line check" << endl;
    for (index.y = 25; index.y<=36; index.y+=0.25, val++)
        testMap(index) = val;

    val = 15;
    for(lit= testMap.beginLine(RecPoint2D(35,25),
                               RecPoint2D(35,36));
        lit!=testMap.endLine();
        ++lit, val++)
        BOOST_CHECK_EQUAL(*lit,val);

    int cnt = 0;


    for (lit = testMap.beginLine(RecPoint2D(-100,0),
                                 RecPoint2D(100,0));
         lit!=testMap.endLine();
         ++lit, cnt++)
    {
        BOOST_CHECK_EQUAL(lit.isValid(),cnt>=200 && cnt < 600);

    }
    
    cout << "positive diagonal line check" << endl;
    cnt=0;
    for (index.x = 0, index.y =0; index.x<50; index.y+=0.25, index.x+=0.25,cnt++)
        testMap(index)=cnt;

    cnt = 0;
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(49.75,49.75));
         lit!=testMap.endLine();
         ++lit, cnt++)
        BOOST_CHECK_EQUAL(*lit,cnt);

    cout << "negative traversal of positive diagonal line check" << endl;
    cnt=0;
    for (index.x = 0, index.y =0; index.x>=-50; index.y-=0.25, index.x-=0.25,cnt++)
        testMap(index)=cnt;

    cnt = 0;
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(-50,-50));
         lit!=testMap.endLine();
         ++lit, cnt++)
        BOOST_CHECK_EQUAL(*lit,cnt);
    

    cout << "positive traversal of negative diagonal line check" << endl;
    cnt=0;
    for (index.x = 0, index.y =0; index.x>=-50 && index.y<50; index.y+=0.25, index.x-=0.25,cnt++)
    {
//        cout << index << endl;
        testMap(index)=cnt;
    }

    cnt =0 ;
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(-49.75,49.75));
         lit!=testMap.endLine();
         ++lit, cnt++)
        BOOST_CHECK_EQUAL(*lit,cnt);
        

    cnt=0;
    for (index.x = 0, index.y =0; index.x<50; index.y-=0.25, index.x+=0.25,cnt++) 
    {
        testMap(index)=cnt;
    }

    cnt =0 ;
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(49.75,-49.75));
         lit!=testMap.endLine();
         ++lit, cnt++)
        BOOST_CHECK_EQUAL(*lit,cnt);

    cout << "Setting a single value" << endl;
    
    for (lit = testMap.beginLine(RecPoint2D(0,0),
                                 RecPoint2D(0,0));
         lit!=testMap.endLine();
         ++lit)
        *lit = 59210;

    BOOST_CHECK_EQUAL(59210.0,testMap(RecPoint2D(0,0)));


    cout << "Testing range of iterator is correct" << endl;
    int ii=0;
    for (lit = testMap.beginLine(RecPoint2D(-30,30),
                                RecPoint2D(0,30));
         lit!=testMap.endLine();
         ++lit, ++ii)
    {
//        cout << ii << ") ";
        *lit = 7;   
    }
         
    ii=0;
    index.x = -30; index.y =30;
    for (/**/; index.x<=0; index.x+=0.25, ii++) {
//        cout << ii << ") ";
//        cout << index << endl;
        BOOST_CHECK_EQUAL(testMap(index),7);

    }
             

}


test_suite * init_unit_test_suite(int argc, char *argv[]) 
{
    test_suite *test = BOOST_TEST_SUITE("map tests");
    test->add(BOOST_TEST_CASE(&lineTest),0);
    test->add(BOOST_TEST_CASE(&mapSerializeTest),0);
    test->add(BOOST_TEST_CASE(&mapTest),0);
    test->add(BOOST_TEST_CASE(&changeMapTest),0);
    return test;
}

#endif //ifndef _MAPUNITTEST_CC_
