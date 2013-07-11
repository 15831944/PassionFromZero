/**
 * @file map.def.h
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date: 9/19/2006
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _MAP_DEF_H_
#define _MAP_DEF_H_

#ifndef _MAP_H_
#error Should only be included from Map.h
#endif

/**
 * @brief constructs a line iterator
 *
 * @param range the iterator will step from range.rs,range.cs to range.re,range.ce, inclusively
 * @param map the map that this iterator will operate on
 */
template <class TT>
inline Map<TT>::LineIterator::LineIterator(const IndexRange &range, const Map<TT> & map): range_(range)
{

    curCell_ = map.cells_+range_.cs+range_.rs*map.numCols_;
    offEnd_= false;
    if (range_.rs==range_.re && range_.cs==range_.ce)
        almostOffEnd_ = true;
    else
        almostOffEnd_ = false;
    row_ = range_.rs;
    col_ = range_.cs;
    numRows_ = map.numRows_;
    numCols_ = map.numCols_;
/*    std::cout << "**********************" << std::endl;
    std::cout << "**********************" << std::endl;
        
    std::cout << "range start:" << range_.rs << ", " << range_.cs 
              << " end: " << range_.re << ", " << range_.ce
              << std::endl;
*/
    //bressenham line setup
    // code borrowed from: http://www.cs.unc.edu/~mcmillan/comp136/Lecture6/Lines.html
    dr_ =  range_.re - range_.rs;
    dc_ = range_.ce - range_.cs;

    if (dr_ < 0) 
    {
        dr_ = -dr_;  stepRow_ = -1; 
    } 
    else 
    { 
        stepRow_ = 1; 
    }

    if (dc_ < 0) 
    { 
        dc_ = -dc_;  stepCol_ = -1; 
    } 
    else 
    { 
        stepCol_ = 1; 
    }

    dr_ <<=1;
    dc_ <<=1;
    if (dc_ > dr_)
        fraction_ = dr_ - (dc_ >> 1);
    else 
        fraction_ = dc_ - (dr_ >> 1);
}

/**
 * @brief returns true if the iterator is within its map
 * it is possible for the iterator to step out of the map if the end points are not on the map.
 */
template <class TT>
inline bool Map<TT>::LineIterator::isValid(void)
{
    return (row_>=0 && col_>=0 && row_<numRows_ && col_<numCols_);
}

/**
 * @brief copy constructor
 */
template <class TT>
inline Map<TT>::LineIterator::LineIterator(const LineIterator & other) 
{
    (*this) = other;
}

/**
 * @brief void constructor, which effectively creates a LineIterator end case
 */
template <class TT>
inline Map<TT>::LineIterator::LineIterator(void)
{
    offEnd_ = true;
    almostOffEnd_ =true;
    curCell_ = NULL;
    row_=0;
    col_ = 0;
    numRows_ = 0;
    numCols_ = 0;
    dr_ = 0;
    dc_ = 0;
    fraction_=0;
    stepRow_=0;
    stepCol_ =0;
}


/**
 * @brief pre-increment operator
 */
template <class TT>
inline typename Map<TT>::LineIterator &
Map<TT>::LineIterator::operator++()
{
    if (offEnd_)
        return *this;
    
/*    std::cout << "dr_: " << dr_
              << " dc_: " << dc_
              << " fraction: " << fraction_
              << " row_: " << row_
              << " col_: " << col_
              << " stepRow: " << stepRow_
              << " stepCol: "<< stepCol_ 
              << " almostOffEnd: " << almostOffEnd_ 
              << std::endl;
*/
    if (dc_ > dr_)
    {
        if (fraction_ >=0)
        {
            row_ += stepRow_;
            curCell_ += (numCols_*stepRow_);
            fraction_ -= dc_;
        }
        col_ += stepCol_;
        curCell_ += stepCol_;
        fraction_ += dr_;

        // we add almostOffEnd to allow us to actually touch the end points coming from above or below.
        // if we were almostOffEnd last time, we will be offEnd this time.
        if (almostOffEnd_)
            offEnd_ = true;

        if (col_ == range_.ce)
            almostOffEnd_ = true;


        
    }
    else
    {
        if (fraction_ >= 0)
        {
            col_ += stepCol_;
            curCell_ += stepCol_;
            fraction_ -= dr_;
        }
        row_ += stepRow_;
        curCell_ += (numCols_*stepRow_);
        fraction_ += dc_;

        // we add almostOffEnd to allow us to actually touch the end points coming from above or below.
        // if we were almostOffEnd last time, we will be offEnd this time.
        if (almostOffEnd_)
            offEnd_ = true;

        if (row_ == range_.re)
            almostOffEnd_ = true;
    }
    return *this;
}

/**
 * @brief post-increment operator
 */
template <class TT>
inline typename Map<TT>::LineIterator Map<TT>::LineIterator::operator++(int)
{
    LineIterator ret(*this);
    ++(*this);
    return ret;
}

/**
 * @brief assignment operator
 */
template <class TT>
typename Map<TT>::LineIterator & Map<TT>::LineIterator::operator=(LineIterator const &other)
{
    curCell_ = other.curCell_;
    range_= other.range_;
    offEnd_ = other.offEnd_;
    almostOffEnd_ = other.almostOffEnd_;
    row_ = other.row_;
    col_ = other.col_;
    numRows_ = other.numRows_;
    numCols_ = other.numCols_;
    dr_ = other.dr_;
    dc_ = other.dc_;
    fraction_ = other.fraction_;
    stepRow_ = other.stepRow_;
    stepCol_ = other.stepCol_;
    return *this;
}

/**
 * @brief equality operator
 */
template <class TT>
inline bool Map<TT>::LineIterator::operator==(LineIterator const & other) const

{
    if (offEnd_==true && other.offEnd_==true)
        return true;
    return other.curCell_ == curCell_;
}






/**
 * @brief private constructor that is used by Map
 */
template <class TT>
inline Map<TT>::iterator::iterator(const IndexRange & range, const Map<TT> & map): range_(range) 
{
    curCell_ = map.cells_ + range_.cs + range_.rs*map.numCols_;
    row_ = range_.rs; col_ = range_.cs;
    offEnd_ = false;
    columnStep_ = map.numCols_-(range_.ce-range_.cs);
}

/**
 * @brief copy constructor
 */
template <class TT>
inline Map<TT>::iterator::iterator(const Map<TT>::iterator & other):
    curCell_(other.curCell_),range_(other.range_), offEnd_(other.offEnd_),
    row_(other.row_),col_(other.col_),columnStep_(other.columnStep_)
{
}

/**
 * @brief this generates a Map<TT>::iterator that can be used to test when another iterator has run its course
 */
template <class TT>
inline Map<TT>::iterator::iterator(void)
{
    offEnd_ = true;
    curCell_ = NULL;
    row_=0;
    col_ = 0;
    columnStep_ = 0;
}

/**
 * @brief the pre-increment operator
 */
template <class TT>
inline typename Map<TT>::iterator &
Map<TT>::iterator::operator++()
{
    // if we're off the end of the iterator, then we don't step further
    if (offEnd_)
    {
        return *this;
    }

    // we increment the column and the underlying vector iterator
    col_++;
    ++curCell_;
    // now check to see if we're past the column end
    if (col_==range_.ce)
    {
        // we are, move to the next row
        row_++;
        /// check if we're past the last row
        if (row_==range_.re)
        {
            // we are, mark that so
            offEnd_=true;
        }
        else
        {
            // we now need to move to the corect column and index
            col_=range_.cs;
            curCell_+=columnStep_;
        }
    }
    return *this;
}

/**
 * @brief post-increment operator
 */
template <class TT>
inline typename Map<TT>::iterator
Map<TT>::iterator::operator++(int)
{
    iterator ret(*this);
    ++(*this);
    return ret;
}

/**
 * @brief assignment operator
 */
template <class TT>
inline typename Map<TT>::iterator &
Map<TT>::iterator::operator=(Map<TT>::iterator const & other) 
{
    curCell_=other.curCell_;
    range_=other.range_;
    offEnd_= other.offEnd_;
    row_=other.row_;
    col_=other.col_;
    columnStep_=other.columnStep_;
    return *this;
}

/**
 * @brief equality operator
 */
template <class TT>
inline bool Map<TT>::iterator::operator==(Map<TT>::iterator const &other) const
{
    if (other.offEnd_==true && offEnd_==true) 
        return true;
    return other.curCell_ == curCell_;
}

/**
 * @brief inequality operator
 */
template <class TT>
inline bool Map<TT>::iterator::operator!=(Map<TT>::iterator const &other) const
{   
    return !(*this==other);
}

#endif //ifndef _MAP_DEF_H_
