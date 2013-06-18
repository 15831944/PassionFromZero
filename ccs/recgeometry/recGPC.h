/**
 * @file recGPC.h
 * @brief Declares GPC Bindings for the recGeometry Library
 * @author: Christopher Baker (tallbaker@cmu.edu)
 * @date: 07/20/2007
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 *
 * IMPORTANT: GPC is free for non-commercial use only, see
 * http://www.cs.man.ac.uk/~amurta/software/index.html#gpc
 * for licensing details.
 *
 * Based on libgpcl-dev 2.32-1 (Ubuntu/Dapper)
 */
#ifndef _RECGPC_H_
#define _RECGPC_H_

#include "recGeometry.h"
#include <list>

extern "C" {
#include <gpcl/gpc.h>
}

/**\brief Singleton class to expose GPC bindings for the recGeometry library
 * \ingroup recGeometryGroup
 *
 * IMPORTANT: GPC is free for non-commercial use only, see
 * http://www.cs.man.ac.uk/~amurta/software/index.html#gpc
 * for licensing details.
 *
 * Based on libgpcl-dev 2.32-1 (Ubuntu/Dapper)
 */
class RecGPC {
  public:
    // use these to compute intersections and whatnot
    static bool computeIntersection(const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out);
    static bool computeUnion(const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out);
    static bool computeDifference(const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out);

    static bool computeIntersection(const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out);
    static bool computeUnion(const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out);
    static bool computeDifference(const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out);


    // exposed publicly mostly for inquiry during the unit test
    static bool usingShallowCopy();
    static void disableShallowCopy(bool disable);
  protected:
    // conversion helpers
    static void RecPolyToGPCVertexList(const RecPolygon2D &recPoly, gpc_vertex_list &out);
    static void RecPolyVectorToGPCPoly(const std::vector<RecPolygon2D> & in, gpc_polygon &out);
    static void GPCVertexListToRecPoly(const gpc_vertex_list &gpcVertexList, RecPolygon2D &out);
    static bool GPCPolyToRecPolyVector(const gpc_polygon &gpcPolygon, std::vector<RecPolygon2D> &out);
    static void RecPolyToGPCPoly(const RecPolygon2D &recPoly, gpc_polygon &out);

    // operational helpers
    static bool gpcPolygonClip(const gpc_op operation, const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out);
    static bool gpcPolygonClip(const gpc_op operation, const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out);
  private:
    // shallow copy state
    static bool disableShallowCopy_; ///< Whether to explicitly disallow shallow copy
    static bool usingShallowCopy_; ///< Whether shallow copy is available for use
    static bool checkedShallowCopy_; ///< Whether we have checked for shallow copy compatibility
};
#endif
