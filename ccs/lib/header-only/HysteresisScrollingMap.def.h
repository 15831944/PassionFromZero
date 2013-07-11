
#ifndef _HYSTERESISSCROLLINGMAP_DEF_H_
#define _HYSTERESISSCROLLINGMAP_DEF_H_

#ifndef _HYSTERESISSCROLLINGMAP_H_
#error "This file should only be included from HysteresisScrollingMap.h"
#endif

/**
 * @brief default constructor allows for declaration of iterator
 */
template <class TT>
inline HysteresisScrollingMap<TT>::iterator::iterator(void)
    :dataIter_(),
    timeIter_()
{
}

template <class TT>
inline HysteresisScrollingMap<TT>::iterator::iterator(const iterator & other)
{
    *this = other;
}

template <class TT>
inline HysteresisScrollingMap<TT>::iterator::iterator(const IndexRange & range,
                                            HysteresisScrollingMap<TT> &map)
    :ScrollingMap<TT>::iterator(range, map),
    timeIter_(range, map.timeMap_)
{
}

template <class TT>
inline HysteresisScrollingMap<TT>::iterator::iterator(const typename ScrollingMap<TT>::iterator & data, const typename ScrollingMap<boost::posix_time::ptime>::iterator & time, HysteresisScrollingMap<TT> & map)
    :map_(&map),
    dataIter_(data),
    timeIter_(time)
{
}

template <class TT>
inline typename HysteresisScrollingMap<TT>::iterator &
HysteresisScrollingMap<TT>::iterator::operator++(void)
{
    dataIter_++;
    timeIter_++;
    return *this;
}

template <class TT>
inline typename HysteresisScrollingMap<TT>::iterator
HysteresisScrollingMap<TT>::iterator::operator++(int)
{
    iterator ret(*this);
    ++dataIter_;
    ++timeIter_;
    return ret;
}

template <class TT>
inline typename HysteresisScrollingMap<TT>::iterator &
HysteresisScrollingMap<TT>::iterator::operator=(HysteresisScrollingMap<TT>::iterator const & other)
{
    map_ = other.map_;
    dataIter_= other.dataIter_;
    timeIter_ = other.timeIter_;
    return *this;
}

template <class TT>
inline bool HysteresisScrollingMap<TT>::iterator::operator==(HysteresisScrollingMap<TT>::iterator const &other) const
{
    bool status(dataIter_ == other.dataIter_);
    status = status && (timeIter_ == other.timeIter_);
    return status;
}

template <class TT>
inline bool HysteresisScrollingMap<TT>::iterator::operator!=(HysteresisScrollingMap<TT>::iterator const &other) const
{
    return !(*this==other);
}

template <class TT>
inline bool HysteresisScrollingMap<TT>::iterator::isValid()
{
    bool status(true);
    status = isTimeValid() && isSpaceValid();
    return status;
}

/**
 * @brief generates a lineiterator that will step through the map from start to end inclusively
 *
 * @param start the location of the first cell for the iterator to touch
 * @param end the location of the last cell the iterator will touch
 */
template <class TT>
typename HysteresisScrollingMap<TT>::LineIterator
HysteresisScrollingMap<TT>::beginLine(const RecPoint2D & start,
                   const RecPoint2D & end)
{
    typename ScrollingMap<TT>::LineIterator data(dataMap_.beginLine(start, end));
    typename ScrollingMap<boost::posix_time::ptime>::LineIterator time(timeMap_.beginLine(start, end));
    return LineIterator(data, time, *this);
}

template <class TT>
inline HysteresisScrollingMap<TT>::LineIterator::LineIterator(const typename ScrollingMap<TT>::LineIterator & data, const typename ScrollingMap<boost::posix_time::ptime>::LineIterator & time, HysteresisScrollingMap<TT> & map)
    :map_(&map),
    dataIter_(data),
    timeIter_(time)
{
}

/**
 * @brief generates an end LineIterator that can be used to check if a LineIterator has come to the end of a line
 */
template <class TT>
typename HysteresisScrollingMap<TT>::LineIterator
HysteresisScrollingMap<TT>::endLine(void)
{
     LineIterator ret;
     return ret;
}

/**
 * @brief constructs a line iterator
 *
 * @param range the iterator will step from range.rs,range.cs to range.re,range.ce, inclusively
 * @param map the map that this iterator will operate on
 */
template <class TT>
inline HysteresisScrollingMap<TT>::LineIterator::LineIterator(const IndexRange &range, HysteresisScrollingMap<TT> & map)
    :map_(&map),
    dataIter_(range, map.dataMap_),
    timeIter_(range, map.timeMap_)
{
}

/**
 * @brief returns true if the iterator is within its map
 * it is possible for the iterator to step out of the map if the end points are not on the map.
 */
template <class TT>
inline bool HysteresisScrollingMap<TT>::LineIterator::isValid(void)
{
    return isTimeValid() && isSpaceValid();
}

/**
 * @brief copy constructor
 */
template <class TT>
inline HysteresisScrollingMap<TT>::LineIterator::LineIterator(const LineIterator & other)
    :map_(other.map_),
    dataIter_(other.dataIter_),
    timeIter_(other.timeIter_)
{
}

/**
 * @brief void constructor, which effectively creates a LineIterator end case
 */
template <class TT>
inline HysteresisScrollingMap<TT>::LineIterator::LineIterator()
    :map_(),
    dataIter_(),
    timeIter_()
{
}

/**
 * @brief pre-increment operator
 */
template <class TT>
inline typename HysteresisScrollingMap<TT>::LineIterator &
HysteresisScrollingMap<TT>::LineIterator::operator++()
{
    ++dataIter_;
    ++timeIter_;
    return *this;
}

/**
 * @brief post-increment operator
 */
template <class TT>
inline typename HysteresisScrollingMap<TT>::LineIterator HysteresisScrollingMap<TT>::LineIterator::operator++(int)
{
    LineIterator ret(*this);
    ++dataIter_;
    ++timeIter_;
    return ret;
}

/**
 * @brief assignment operator
 */
template <class TT>
typename HysteresisScrollingMap<TT>::LineIterator & HysteresisScrollingMap<TT>::LineIterator::operator=(LineIterator const &other)
{
    map_ = other.map_;
    dataIter_ = other.dataIter_;
    timeIter_ = other.timeIter_;
    return *this;
}

/**
 * @brief equality operator
 */
template <class TT>
inline bool HysteresisScrollingMap<TT>::LineIterator::operator==(LineIterator const & other) const

{
    return (dataIter_ == other.dataIter_) && (timeIter_ == other.timeIter_);
}
#endif

