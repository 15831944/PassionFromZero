#ifndef _STLNEED_H_
#define _STLNEED_H_

#include "../lib/RecGeometry.h"

/*** This allows us to create a stl::set<RecPoint3D, ltpt3D> ***/
struct ltpt3D
{
    bool operator()(const RecPoint3D & s1, const RecPoint3D & s2) const
    {
        if (s1.x < s2.x)
            return true;

        if ((s1.x == s2.x) && (s1.y < s2.y))
            return true;

        if ((s1.x == s2.x) && (s1.y == s2.y) && (s1.z < s2.z))
            return true;

        return false;
    }
};


#endif //#ifndef _STLNEED_H
