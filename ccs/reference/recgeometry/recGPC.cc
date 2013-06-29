/**
 * @file recGPC.cc
 * @brief Implements GPC Bindings for the recGeometry Library
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

#include "recGPC.h"

#include <string.h> // for memcpy

static const gpc_vertex_list nilGPCVertexList = {0,NULL};
static const gpc_polygon     nilGPCPolygon    = {0,NULL,NULL};

// static members
bool RecGPC::disableShallowCopy_ = false;
bool RecGPC::usingShallowCopy_ = false;
bool RecGPC::checkedShallowCopy_ = false;

bool RecGPC::computeIntersection(const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out)
{
    return gpcPolygonClip(GPC_INT,first,second,out);
}

bool RecGPC::computeUnion(const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out)
{
    return gpcPolygonClip(GPC_UNION,first,second,out);
}

bool RecGPC::computeDifference(const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out)
{
    return gpcPolygonClip(GPC_DIFF, first, second, out);
}


bool RecGPC::computeIntersection(const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out)
{
    return gpcPolygonClip(GPC_INT, first, second, out);
}

bool RecGPC::computeUnion(const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out)
{
    return gpcPolygonClip(GPC_UNION, first, second, out);
}
bool RecGPC::computeDifference(const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out)
{
    return gpcPolygonClip(GPC_DIFF, first, second, out);
}

bool RecGPC::gpcPolygonClip(const gpc_op operation, const RecPolygon2D &first, const RecPolygon2D &second, std::list<RecPolygon2D> &out)
{
    std::vector<RecPolygon2D> vFirst, vSecond, vOut;
    bool changed= false;
    vFirst.push_back(first);
    vSecond.push_back(second);
    gpcPolygonClip(operation, vFirst, vSecond, vOut);

    if (!vOut.empty())
    {
        changed =true;
        out.insert(out.begin(), vOut.begin(), vOut.end());
    }

    return changed;
}



bool RecGPC::gpcPolygonClip(const gpc_op operation, const std::vector<RecPolygon2D> &first, const std::vector<RecPolygon2D> &second, std::vector<RecPolygon2D> &out)
{
    bool changed = false;
    gpc_polygon firstPoly = nilGPCPolygon;
    gpc_polygon secondPoly = nilGPCPolygon;
    gpc_polygon outputPoly = nilGPCPolygon;
    RecPolyVectorToGPCPoly(first,firstPoly);
    RecPolyVectorToGPCPoly(second,secondPoly);

    // the actual gpcl call
    gpc_polygon_clip( operation, &firstPoly, &secondPoly, &outputPoly );

    changed = GPCPolyToRecPolyVector(outputPoly,out);

    // cleanup
    gpc_free_polygon( &firstPoly );
    gpc_free_polygon( &secondPoly );
    gpc_free_polygon( &outputPoly );

    return changed;
}



/**\brief Mutator for disableShallowCopy_ Has no effect if shallow copy is not compatible
 *
 * \attention This can break thread-safety if it is called while something is processing.
 * Use with caution
 */
void RecGPC::disableShallowCopy(bool disable)
{
    disableShallowCopy_ = disable;
}

/**\brief Determine whether we are using shallow copy
 * \return boolean indication thereof
 *
 * This makes incremental checks to see whether a brute-force
 * memcpy is equivalent to piecemeal assignment for optimizing
 * conversions to and from the underlying GPC types and caches
 * the result in usingShallowCopy_
 */
bool RecGPC::usingShallowCopy()
{
    if(disableShallowCopy_)
    {
        // allow shallow copy to be disabled for testing
       return false; // embedded return
    } else {
        if(!checkedShallowCopy_)
        {
            printf("Checking for shallow copy compatibility between gpc_vertex and RecPoint2D...\n");
            if(sizeof(gpc_vertex) == sizeof(RecPoint2D) &&
               sizeof(gpc_vertex[1]) == sizeof(RecPoint2D[1]))
            {
                // same base and array sizes, memcpy will be safe, check datum;
                const double xTest = 1.23456789;
                const double yTest = -9.8234215;
                gpc_vertex gpcV;
                RecPoint2D recP;

                // populate gpcV
                gpcV.x = xTest;
                gpcV.y = yTest;

                // memcpy to recP
                memcpy(&recP,&gpcV,sizeof(gpc_vertex));

                // check recP
                if(recP.x == xTest && recP.y == yTest)
                {
                    // data is the same, we can shallow copy
                    printf(" -> Hooray! using shallow copy\n");
                    usingShallowCopy_ = true;
                } else {
                    printf(" -> Data mismatch on shallow copy: %lf -> %lf, %lf -> %lf, stuck with deep copy\n",
                           gpcV.x,recP.x,gpcV.y,recP.y);
                }
            } else {
                printf(" -> Size mismatch: (%lu,%lu) -> (%lu,%lu), stuck with deep copy\n",
                       sizeof(gpc_vertex), sizeof(gpc_vertex[1]), sizeof(RecPoint2D), sizeof(RecPoint2D[1]));
            }
            // we have checked
            checkedShallowCopy_ = true;
        }
    }
    return usingShallowCopy_;
}

/**\brief Protected conversion helper from a single RecPolygon2D to the underlying gpc_vertex_list type
 * \param recPoly The input polygon to convert
 * \param out The output vertex list
 */
void RecGPC::RecPolyToGPCVertexList(const RecPolygon2D &recPoly, gpc_vertex_list &out)
{
    out.num_vertices = recPoly.getVertices().size();
    if(out.vertex != NULL)
    {
        free(out.vertex);
        out.vertex = NULL;
    }

    out.vertex = reinterpret_cast<gpc_vertex *>(malloc(out.num_vertices * sizeof(gpc_vertex[1])));
    if(out.vertex != NULL)
    {
        if(usingShallowCopy())
        {
            // we cheat and memcpy out of the vector
            memcpy(out.vertex,
                   &(recPoly.getVertices()[0]),
                   out.num_vertices * sizeof(gpc_vertex[1]));
        } else {
            // iterate by hand
            // todo: see if it is actually slower...
            unsigned int jj = 0; // jj is for gpc_vertex_list::vertex indexing
            for(std::vector<RecPoint2D>::const_iterator ii=recPoly.getVertices().begin();
                ii != recPoly.getVertices().end(); ++ii)
            {
                out.vertex[jj].x = ii->x;
                out.vertex[jj].y = ii->y;
                ++jj;
            }
        }
    } else {
        fprintf(stderr,"RecGPC::RecPolyToGPCVertexList: failed to malloc %lu bytes for gpc_vertex_list\n",
                out.num_vertices * sizeof(gpc_vertex[1]));
        out.num_vertices = 0;
    }
}

/**\brief Protected conversion helper from the underlying gpc_vertex_list type to a single RecPolygon2D
 * \param gpcVertexList The input polygon (contour) to convert
 * \param out The output polyggon
 */
void RecGPC::GPCVertexListToRecPoly(const gpc_vertex_list &gpcVertexList, RecPolygon2D &out)
{
    std::vector<RecPoint2D> recVertices;
    recVertices.resize(gpcVertexList.num_vertices);
    if(usingShallowCopy())
    {
        // we cheat and memcpy into the vector
        memcpy(&(recVertices[0]),
               gpcVertexList.vertex,
               gpcVertexList.num_vertices * sizeof(gpc_vertex[1]));
    } else {
        // iterate by hand
        // todo: see if it is actually slower...
        unsigned int jj = 0; // jj is for gpc_vertex_list::vertex indexing
        for(std::vector<RecPoint2D>::iterator ii=recVertices.begin();
            ii != recVertices.end(); ++ii)
        {
            ii->x = gpcVertexList.vertex[jj].x;
            ii->y = gpcVertexList.vertex[jj].y;
            ++jj;
        }
    }

    // todo: replace with a polygon::setVertices operator
    out = RecPolygon2D(recVertices);
}

/**\brief Protected conversion helper from the underlying gpc_polygon type to a list of RecPolygon2D's
 * \param gpcPolygon The polygon to convert
 * \param out The output list to populate
 * \return boolean indication of whether the output list was modified
 *
 * Note: The gpc_polygon type is actually a list of contours, some of which are acually holes.
 * Todo: figure out what these holes are/mean
 */
bool RecGPC::GPCPolyToRecPolyVector(const gpc_polygon &gpcPolygon, std::vector<RecPolygon2D> &out)
{
    bool changed = false;
    for(int ii = 0; ii<gpcPolygon.num_contours; ++ii)
    {

        if(gpcPolygon.hole != NULL && gpcPolygon.hole[ii] == 1)
        {
            printf("RecGPC: Ignoring holy polygon\n");
        } else {
            RecPolygon2D tmpPoly;
            GPCVertexListToRecPoly(gpcPolygon.contour[ii],tmpPoly);
            out.push_back(tmpPoly);
            changed = true;
        }
    }
    return changed;
}

/**\brief Protected conversion helper to create a gpc_polygon from a RecPolygon2D
 * \param recPoly the polygon to convert
 * \param out The output polygon
 *
 * This is basically a wrapper over RecPolyToGPCPoly and gpc_add_contour
 */
void RecGPC::RecPolyToGPCPoly(const RecPolygon2D &recPoly, gpc_polygon &out)
{
    gpc_vertex_list contour = nilGPCVertexList;
    RecPolyToGPCVertexList(recPoly,contour);
    gpc_add_contour(&out,&contour,0);
    if(contour.vertex != NULL)
        free(contour.vertex);
}


void RecGPC::RecPolyVectorToGPCPoly(const std::vector<RecPolygon2D> & in, gpc_polygon &out)
{
    for (unsigned int ii=0; ii < in.size(); ++ii)
    {
        RecPolyToGPCPoly(in[ii], out);
    }
}
