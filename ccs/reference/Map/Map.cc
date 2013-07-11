/**
 * @file Map.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _MAP_CC_
#define _MAP_CC_
#include "Map.h"
/**
 * @brief this ensures that row and column starts are smaler than row and colum ends
 */
void IndexRange::normalize(void)
{
    if (rs > re) 
    {
        int tmp = rs;
        rs = re;
        re = tmp;
    }
    if (cs > ce)
    {
        int tmp = cs;
        cs = ce;
        ce = tmp;
    }
}



#endif //ifndef _MAP_CC_
