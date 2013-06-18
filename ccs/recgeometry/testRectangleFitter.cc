/**
 * @file testRectangleFitter.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _TESTRECTANGLEFITTER_CC_
#define _TESTRECTANGLEFITTER_CC_
#include <iostream>
#include <fstream>
#include "rectangleFitter.h"
#include <string>
#include <string.h>
using namespace std;


int main(void)
{

    RectangleFitter rf;
    std::vector<RecPoint3D> pts;
    string line;
    float f1, f2, f3;
    // Read in the points
    cout<<"\n Rect Fitting:"<<endl;
    ifstream myfile ("clust2.dat");

    if (myfile.is_open())
    {
       while (! myfile.eof() )
      {
          getline (myfile,line);
          char line2[100];
          strcpy(line2, line.c_str());
          sscanf(line2, "%g %g %g", &f1, &f2, &f3);
	  pts.push_back(RecPoint3D(f1, f2, f3));
      }
      myfile.close();
    }

    std::vector<RecPoint2D> cornerPts;
    cornerPts = rf.findBestRectangle<RecPoint3D>(pts);
    //cornerPts = rf.findPCARectangle<RecPoint3D>(pts);

    cout << "\n corner points are: " << endl;
    cout << cornerPts[0] << " " << cornerPts[1] << " " << cornerPts[2] << " " << cornerPts[3] << endl;
}




#endif //ifndef _TESTRECTANGLEFITTER_CC_
