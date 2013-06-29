/**
 * @file RecKDTree2D.cc
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2007
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _RECKDTREE2D_IMPL_H_
#define _RECKDTREE2D_IMPL_H_

#ifndef _RECKDTREE2D_H_
#error Must only be included from RecKDTree2D.h
#endif

#include <list>

/**
 * @brief constructor
 */
template<class PAYLOAD>
RecKDTree2D<PAYLOAD>::RecKDTree2D(void):
    treeBuilt_ ( false ),
    elements_ ( )
{
}

/**
 *@brief destructor
 */
template<class PAYLOAD>
RecKDTree2D<PAYLOAD>::~RecKDTree2D(void)
{
    clearTree(); // redundant with the vector destructor, but maintained just in case
}

/**
 * @brief Analagous to vector::reserve to hint at the number of element on the way
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::reserve(unsigned long size)
{
    if(treeBuilt_)
    {
        // tree is already built
        return false;
    } else {
        elements_.reserve(size);
        return true;
    }
}

/**\brief Add a single key/value pair to the elements to me KD-Tree'd
 * \param location The key
 * \param payload The value
 * \return boolean success (false implies the tree has already been build and you should clearTree() first
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::addElement(const RecPoint2D &location, const PAYLOAD &payload)
{
    if(treeBuilt_)
    {
        // tree is already built, adds are invalid
        return false;
    } else {
        TreeElement elem;
        elem.loc = location;
        elem.payload = payload;
        elements_.push_back(elem);
        return true;
    }
}

/**\brief Add a bunch of key/value pairs to the elements to me KD-Tree'd
 * \param pairs The datum
 * \return boolean success (false implies the tree has already been build and you should clearTree() first
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::addElements(const PointPayloadPairVector &pairs)
{
    if(treeBuilt_)
    {
        // tree is already built, adds are invalid
        return false;
    } else {
        // iterate and push back
        elements_.reserve(elements_.size() + pairs.size());
        typename PointPayloadPairVector::const_iterator ii;
        for(ii=pairs.begin(); ii != pairs.end(); ++ii)
        {
            TreeElement elem;
            elem.loc = ii->first;
            elem.payload = ii->second;
            elements_.push_back(elem);
        }
        return true;
    }
}

/**\brief Actuallly run the sort/build process
 * \return boolean success (false implies tree already built)
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::buildTree()
{
    if(treeBuilt_ || elements_.size() == 0)
    {
        // tree is already built
        return false;
    } else {
        // now recursively build the tree
        if(elements_.size() > 1)
        {
            buildTree(elements_, 0, elements_.size(), 0);
        }
        treeBuilt_ = true;
        return treeBuilt_;
    }
}

/**\brief Nuke the tree in-place
 * \return boolean success (always true for now)
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::clearTree()
{
    elements_.clear();
    treeBuilt_ = false;
    return true;
}

/**
 *@brief builds the KD-tree.  The split plane goes through points, and switches every depth change
 *@param vec the vector of TreeElements we're building the tree from
 *@param begin index into the vector this tree starts at
 *@param end index into the vector just after this tree ends
 *@param depth current depth we're building this tree at
 */
template<class PAYLOAD>
void RecKDTree2D<PAYLOAD>::buildTree(ElemVector & vec, unsigned int begin, unsigned int end, unsigned int depth)
{
    // split on the median... build the subtrees
    unsigned int median = (end+begin)>>1;

    // arrr, there be sorting to do
    if((depth & 1) == 0)
    {
        // split along x
        std::sort(&vec[begin],
                  &vec[end],
                  LessX());
    } else {
        // split along y
        std::sort(&vec[begin],
                  &vec[end],
                  LessY());
    }

    if((begin + 1) < median)
    {
        // there are at least two elements in the left half, sort some more
        buildTree(vec, begin,    median,  depth+1);
    }

    if((median + 2) < end)
    {
        // there are at least two elements in the right half, sort some more
        buildTree(vec, median+1,    end,  depth+1);
    }
}

/**\brief Public closest lookup
 * \param pt The query point
 * \param out The payload output (by reference)
 * \return boolean success (falses implies unbuilt/empty tree)
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::getClosestElement(const RecPoint2D &pt, PAYLOAD &out, double maxDistSq, RecPoint2D *ploc) const
{
    if(treeBuilt_)
    {
        TreeElement nearest;
        TreeWalker walker(elements_);
        if (findNearest(walker, pt, maxDistSq, &nearest,0)!=INFINITY)
        {
            out = nearest.payload;
	    if( ploc ) *ploc = nearest.loc;
            return true;
        } else {
            return false;
        }
    }
    return false;
}

/**\brief Public blob query
 * \param pt The query point
 * \param radius The query radius
 * \param out A vector to receive the points in the blob
 * \return boolean success (false implies unbuilt/empty tree or no elements withing radius of pt)
 */
template<class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::getElementsWithin(const RecPoint2D &pt, const double &radius, PayloadVector &out) const
{
    bool changed = false;
    if(treeBuilt_)
    {
        // use our internal lookup
        std::list<const TreeElement*> near;
        // we use radiusSq to save on sqrt's
        double radiusSq = radius * radius;
        TreeWalker walker(elements_);
        findWithin(walker, pt, radiusSq, near, 0);

        for(typename std::list<const TreeElement*>::iterator ii=near.begin();
            ii != near.end(); ++ii)
        {
            changed = true;
            const TreeElement *elem = *ii;
            out.push_back(elem->payload);
        }
    }
    return changed;
}

/**
 *@brief the recursive search that looks for tree elements that are within the given radius
 *@param tree the span we're looking at
 *@param pt the point of interest we're looking for things near
 *@param radiusSq square of the radius of interest
 *@param cluster the returned list of nearby tree elements
 *@param depth the current depth of the search
 */
template<class PAYLOAD>
void RecKDTree2D<PAYLOAD>::findWithin(const RecKDTree2D<PAYLOAD>::TreeWalker &tree,
                                      const RecPoint2D &pt,
                                      double radiusSq,
                                      std::list<const RecKDTree2D<PAYLOAD>::TreeElement *> &cluster,
                                      unsigned int depth) const
{
    bool smallerInRange, largerInRange;
    double hpDis;

    // figure out where the query point is relative to the hinge in our dimension
    if ((depth & 1) == 0)
    {
        hpDis = pt.x - tree.getElement().loc.x;
    } else {
        hpDis = pt.y - tree.getElement().loc.y;
    }

    double hpDisSq = hpDis * hpDis;
    if (hpDisSq < radiusSq)
    {
        // close enough to the hinge to require both be searched
        smallerInRange = true;
        largerInRange  = true;
    }
    else if ( hpDis < 0 )
    {
        // in the smaller range only
        smallerInRange = true;
        largerInRange  = false;
    }
    else
    {
        // in larger range only
        smallerInRange = false;
        largerInRange  = true;
    }

    if (smallerInRange && largerInRange)
    {
        // this hinge is a candidate
        double distSq = tree.getElement().loc.distanceSq(pt);
        if(distSq < radiusSq)
        {
            cluster.push_back(&(tree.getElement()));
        }
    }

    TreeWalker newWalker(tree);

    if (smallerInRange && tree.getSmaller(newWalker))
    {
        findWithin(newWalker, pt, radiusSq, cluster, depth+1);
    }
    if (largerInRange && tree.getLarger(newWalker))
    {
        findWithin(newWalker, pt, radiusSq, cluster, depth+1);
    }
}

/**
 *@brief the recursive nearest neighbor search algorithm from Andrew Moore's thesis
 *@param tree the span we're searching
 *@param pt the point we're trying to find nearest to
 *@param maxDistSq the maximum possible distance a better solution can be
 *@param nearest the current nearest tree element
 *@param depth the current search depth
 *@return the squared distance between the nearest point and the query point
 */
template<class PAYLOAD>
double RecKDTree2D<PAYLOAD>::findNearest(const RecKDTree2D<PAYLOAD>::TreeWalker &tree,
                                         const RecPoint2D & pt,
                                         double maxDistSq,
                                         RecKDTree2D<PAYLOAD>::TreeElement * nearest,
                                         unsigned int depth) const
{
    bool inSmaller;
    double distSq;

    // depending on depth, we change axis for the split
    if ((depth & 1) == 0)
    {
        inSmaller = pt.x < tree.getElement().loc.x;
    }
    else
    {
        inSmaller = pt.y < tree.getElement().loc.y;
    }

    TreeWalker nearWalker(tree);
    TreeWalker farWalker(tree);
    bool haveNear = false;
    bool haveFar = false;

    // now find out which half of the tree the node is more likely in....
    if (inSmaller)
    {
        haveNear = tree.getSmaller(nearWalker);
        haveFar = tree.getLarger(farWalker);
    }
    else
    {
        haveNear = tree.getLarger(nearWalker);
        haveFar = tree.getSmaller(farWalker);
    }


    // now look there.
    if(haveNear)
    {
        distSq = findNearest(nearWalker, pt, maxDistSq, nearest, depth+1);
    } else {
        distSq = INFINITY;
    }

    maxDistSq = std::min(maxDistSq, distSq);

    // next we check to see if the other half of the space here, is within range of our test point...
    double hpDisSq;
    if ((depth & 1) == 0)
    {
        hpDisSq = (pt.x - tree.getElement().loc.x)*(pt.x - tree.getElement().loc.x);
    }
    else
    {
        hpDisSq = (pt.y - tree.getElement().loc.y)*(pt.y - tree.getElement().loc.y);
    }

    if (hpDisSq < maxDistSq)
    {
        // it is, oh joy.

        // now we're going to check to see if the pivot point is closer than what we've found so far.... if it is, we have
        // to check the other half
        double pivotDistSq = tree.getElement().loc.distanceSq(pt);

        // check to see if the pivot point is closer...
        if (pivotDistSq < distSq)
        {
            *nearest = tree.getElement();
            distSq = pivotDistSq;
            maxDistSq = pivotDistSq;
        }

        double tmpDistSq;
        // now check out the other tree....
        if(haveFar)
        {
            TreeElement tmpElem;
            tmpDistSq = findNearest(farWalker, pt, maxDistSq, &tmpElem, depth+1);
            if (tmpDistSq < distSq)
            {
                // it's closer, so update the returned distance squared and nearest...
                *nearest = tmpElem;
                distSq = tmpDistSq;
            }
        }
    }

    return distSq;
}


template<class PAYLOAD>
void RecKDTree2D<PAYLOAD>::getElements( PointPayloadPairVector &pairs ) const
{
    for( typename ElemVector::const_iterator it = elements_.begin(),
	   it2 = elements_.end();
	 it != it2; ++it )
    {
        pairs.push_back( std::make_pair( it->loc, it->payload ) );
    }

}


// TREEWALKER Implementation

/**\brief Initialization constructor, creates a TreeWalker that spans the whole of elements */
template <class PAYLOAD>
RecKDTree2D<PAYLOAD>::TreeWalker::TreeWalker(const RecKDTree2D<PAYLOAD>::ElemVector &elements):
    elements_(elements),
    from_(0),
    to_(elements.size())
{}

/**\brief Protected full constructor, used exclusively by getSmaller() and getLarger() */
template <class PAYLOAD>
RecKDTree2D<PAYLOAD>::TreeWalker::TreeWalker(const RecKDTree2D<PAYLOAD>::ElemVector &elements, const unsigned int from, const unsigned int to):
    elements_(elements),
    from_(from),
    to_(to)
{}

/**\brief Copy Constructor, necessary for Vectorness */
template <class PAYLOAD>
RecKDTree2D<PAYLOAD>::TreeWalker::TreeWalker(const RecKDTree2D<PAYLOAD>::TreeWalker &copy):
    elements_(copy.elements_),
    from_(copy.from_),
    to_(copy.to_)
{}

/**\brief Element accessor */
template <class PAYLOAD>
const typename RecKDTree2D<PAYLOAD>::TreeElement & RecKDTree2D<PAYLOAD>::TreeWalker::getElement() const
{
    return elements_[getElementIndex()];
}

/**\brief Assignment operator, necessary for Vectorness
 *
 * \note This does not do anything with elements_, as it is a const ref that is difficult to perturb.  This may cause bizarre bugs if you do silly things
 */
template <class PAYLOAD>
const typename RecKDTree2D<PAYLOAD>::TreeWalker & RecKDTree2D<PAYLOAD>::TreeWalker::operator=(const RecKDTree2D<PAYLOAD>::TreeWalker &other)
{
    from_ = other.from_;
    to_ = other.to_;
    return *this;
}

/**\brief Populate a TreeWalker representing the left (smaller) half
 * \param out The new TreeWalker (out by reference)
 * \return boolean success (false implies no left half)
 */
template <class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::TreeWalker::getSmaller(TreeWalker &out) const
{
    if((from_ + 1) >= to_)
    {
        // we need a separation bigger than 1 to have a smaller branch
        return false;
    } else {
        out = TreeWalker(elements_,from_,(from_ + to_) >> 1);
        return true;
    }
}

/**\brief Populate a TreeWalker representing the right (larger) half
 * \param out The new TreeWalker (out by reference)
 * \return boolean success (false implies no right half)
 */
template <class PAYLOAD>
bool RecKDTree2D<PAYLOAD>::TreeWalker::getLarger(TreeWalker &out) const
{
    if((from_ + 2) >= to_)
    {
        // we need a separation bigger than 2 to have a larger branch
        return false;
    } else {
        out = TreeWalker(elements_,1 + ((from_ + to_) >> 1),to_);
        return true;
    }
}

#endif //ifndef _RECKDTREE2D_IMPL_H_
