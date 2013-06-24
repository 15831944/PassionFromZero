#ifndef _STANDALONE_SEG_H_
#define _STANDALONE_SEG_H_

//basic includes
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <string>  // getline();


//STL includes
/////#include <utility>
/////#include <algorithm>
#include <set>
/////#include <vector>
#include "STLneed.h"



using namespace std;


#define FAILOPENSOURCEFILE -1
#define FAILOPENOUTPUTCHECKFILE -2


class CCS
{
protected:
    set<RecPoint3D, ltpt3D> sourceSet;

public:
    CCS() {};
    virtual ~CCS() {};

    void createSourceSet(const char* filename);
    void printSourceSet();






};









#endif // #ifndef _STANDALONE_SEG_H_
