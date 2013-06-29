/**
 * @file RecKDTree2D.h
 * @author: Christopher Baker (tallbaker@cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _RECKDTREE2D_H_
#define _RECKDTREE2D_H_

#include <algorithm>
#include <list>
#include <vector>
#include <math.h>
#include "recGeometry.h"

/**\brief Provide a generic KD-Tree implementation for mapping RecPoint2D's to some generic payload
 * \ingroup recGeometryGroup
 *
 * Notes:
 * - PAYLOAD must be able to live in a vector, which means unit constructor and assignment
 *     operator at least
 * - For large PAYLOAD classes, it may be advantageous to store the payloads separate from
 *     the tree elements and to let TreeElement::payload be a pointer, since the TreeElements
 *     are continuously swapped as part of sort operations and large PAYLOAD, or those with complex
 *     assignment operators may seriously degrade performance
 * - An additional O(lg(n)) speedup may be had by caching the O(n lg(n)) sort results and reducing the
 *     per-depth re-sorts to an O(n) operation.  Initial experiments point toward a speedup on the
 *     order of 20% for trees up to ~1M elements, but that the extra memory cost associated with this
 *     dynamic programming shifts the limiting agent to cache misses and actualy degrades performance
 *     above 10M elements
 */
template<class PAYLOAD>
class RecKDTree2D
{
  public:
    // con/de struction
    RecKDTree2D();
    virtual ~RecKDTree2D();

    // Handy Typedefs
    typedef std::vector<PAYLOAD> PayloadVector;
    typedef std::vector< std::pair < RecPoint2D, PAYLOAD > > PointPayloadPairVector;

    // building the tree
    bool reserve(unsigned long size);
    bool addElement(const RecPoint2D &location, const PAYLOAD &payload);
    bool addElements(const PointPayloadPairVector &pairs);
    bool buildTree();
    bool clearTree();

    // using the tree
    bool getClosestElement(const RecPoint2D &queryPoint, PAYLOAD &out, double maxDistSq = INFINITY, RecPoint2D *ploc = 0) const;
    bool getElementsWithin(const RecPoint2D &queryPoint, const double &radius, PayloadVector &out) const;

    void getElements( PointPayloadPairVector &pairs ) const;

  protected:

    /**\brief Used to be more complicated, now is basically a key-value pair */
    class TreeElement
    {
      public:
        RecPoint2D loc;
        PAYLOAD payload;
    };

    /**\brief This is how we store things */
    typedef std::vector<TreeElement> ElemVector;

    // predicates for sorting
    struct LessX
    {
        bool operator() (const TreeElement &a, const TreeElement &b) const {return a.loc.x < b.loc.x;};
    };

    struct LessY
    {
        bool operator() (const TreeElement &a, const TreeElement &b) const {return a.loc.y < b.loc.y;};
    };

    /**\brief Similar to an iterator, this encapsulates the tree-in-an-array indexing stuff */
    class TreeWalker
    {
      public:
        // construction
        TreeWalker(const TreeWalker &copy);
        TreeWalker(const ElemVector &elements);

        // assignment
        const TreeWalker & operator= (const TreeWalker &other);

        // element accessors
        const TreeElement &getElement() const;
        inline unsigned long getElementIndex() const {return ((from_ + to_) >> 1);};

        // walking
        bool getSmaller( TreeWalker &out ) const;
        bool getLarger( TreeWalker &out ) const;
      private:
        // alternate construction used by getLarger, getSmaller
        TreeWalker(const ElemVector &elements, unsigned int from, unsigned int to);

        // The elements we are walking
        const ElemVector &elements_;

        // The bounds of our walk
        unsigned long from_;
        unsigned long to_;
    };

    void buildTree(ElemVector &vec, unsigned int begin, unsigned int end, unsigned int depth);

    double findNearest(const TreeWalker &tree, const RecPoint2D & pt, double maxDistSq,
                       TreeElement * nearest, unsigned int depth) const;

    void findWithin(const TreeWalker &tree, const RecPoint2D &pt, double radiusSq,
                    typename std::list<const TreeElement *> &cluster, unsigned int depth) const;

    bool treeBuilt_; ///< Boolean indication of whether the tree has been built
    ElemVector elements_; ///< This is where we store everything
};

#include "RecKDTree2D.impl.h"

#endif //ifndef _RECKDTREE2D_H_
