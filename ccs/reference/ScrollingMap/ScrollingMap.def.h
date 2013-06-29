/**
 * @file ScrollingMap.def.h
 * @author: Chris Urmson (curmson@ri.cmu.edu)
 * @date:
 *
 * @attention Copyright (c) 2006
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _SCROLLINGMAP_DEF_H_
#define _SCROLLINGMAP_DEF_H_

#ifndef _SCROLLINGMAP_H_
#error Should only be included from ScrollingMap.h
#endif

template <class TT>
ScrollingMap<TT>::Cell::Cell(void)
{
    row=-1;         // the row and column of each cell is set to -1.  Therefore, each cell will be invalid when first accessed.
    column =-1;
}

template <class TT>
ScrollingMap<TT>::Cell::Cell(const ScrollingMap<TT>::Cell & otherCell)
    : row(otherCell.row),
      column(otherCell.column),
      value(otherCell.value)
{
}

template <class TT>
inline void ScrollingMap<TT>::Cell::operator=(const ScrollingMap<TT>::Cell & otherCell)
{
    if (this==&otherCell)
        return;
    row = otherCell.row;
    column = otherCell.column;
    value =otherCell.value;
}

/**
 * @brief default constructor allows for declaration of iterator
 */
template <class TT>
inline ScrollingMap<TT>::iterator::iterator(void)
{
    map_=NULL;
    offEnd_ = true;
    row_=0;
    col_=0;
    range_=IndexRange();
    cell_=NULL;
    lastCellInRow_=NULL;
}

template <class TT>
inline ScrollingMap<TT>::iterator::iterator(const iterator & other)
{
    *this = other;
}

template <class TT>
inline ScrollingMap<TT>::iterator::iterator(const IndexRange & range,
                                            ScrollingMap<TT> &map):
    range_(range)
{
    map_ = &map;
    row_ = range.rs;
    col_ = range.cs;
    cell_ = &map_->index(row_, col_);
    lastCellInRow_ = computeLastCellInRow(cell_);
    offEnd_ = false;
}

template <class TT>
inline typename ScrollingMap<TT>::iterator &
ScrollingMap<TT>::iterator::operator++(void)
{
    if (offEnd_)
        return *this;
    ++cell_;
    ++col_;
    if (col_>range_.ce)
    {
        row_++;
        if (row_>range_.re)
        {
            offEnd_ = true;
            cell_ = NULL;
            lastCellInRow_ = NULL;
        }
        else
        {
            col_ = range_.cs;
            cell_ = &map_->index(row_, col_);
            lastCellInRow_ = computeLastCellInRow(cell_);
        }
    }
    else if (cell_ > lastCellInRow_)
    {
        cell_ = lastCellInRow_ - (map_->dimension_ - 1);
    }

    return *this;
}

template <class TT>
inline typename ScrollingMap<TT>::iterator
ScrollingMap<TT>::iterator::operator++(int)
{
    iterator ret(*this);
    ++(*this);
    return ret;
}

template <class TT>
inline typename ScrollingMap<TT>::iterator &
ScrollingMap<TT>::iterator::operator=(ScrollingMap<TT>::iterator const & other)
{
    range_ = other.range_;
    offEnd_ = other.offEnd_;
    row_ = other.row_;
    col_ = other.col_;
    map_ = other.map_;
    cell_ = other.cell_;
    lastCellInRow_ = other.lastCellInRow_;
    return *this;
}

template <class TT>
inline bool ScrollingMap<TT>::iterator::operator==(ScrollingMap<TT>::iterator const &other) const
{
    if (other.offEnd_==true && offEnd_==true)
        return true;
    return (other.row_ == row_ &&
            other.col_ == col_ &&
            other.map_ == map_);
}

template <class TT>
inline bool ScrollingMap<TT>::iterator::operator!=(ScrollingMap<TT>::iterator const &other) const
{
    return !(*this==other);
}

/**
 * @brief Return a pair representing the current value/validity
 */
template <class TT>
inline std::pair<TT,bool> ScrollingMap<TT>::iterator::operator*() const
{
    return std::make_pair( cell_->value, cell_->row==row_ && cell_->column==col_ );  
}

//template <class TT>
//inline TT & ScrollingMap<TT>::iterator::operator*() const
//{
//    return map_->index(row_,col_).value;
//};

//template <class TT>
//inline TT * ScrollingMap<TT>::iterator::operator->() const
//{
//    return &(map_->index(row_,col_).value);
//};

/**
 * @brief Sets the current value
 */
template <class TT>
inline void ScrollingMap<TT>::iterator::set( const TT& val )
{
    if (cell_->row==row_ && cell_->column==col_) 
    { 
        cell_->value = val; 
    } 
    else 
    {
        map_->setByRowCol( row_, col_, val );
    }
}

/**
 * @brief reports if the data pointed to in the map is valid
 */
template <class TT>
inline bool ScrollingMap<TT>::iterator::isValid(void) const
{

    typename ScrollingMap<TT>::Cell & cell = map_->index(row_,col_);
    return (cell.row==row_ && cell.column==col_);
}

/**
 * @brief Compute last cell in the row cell_ points to in cells_ local coordinates
 */
template <class TT>
typename ScrollingMap<TT>::Cell *ScrollingMap<TT>::iterator::computeLastCellInRow(const Cell * const cell_) const
{
    const unsigned int currentRow = (cell_ - map_->cells_) / map_->dimension_;
    const unsigned int lastColumn = map_->dimension_ - 1;

    return &map_->cells_[currentRow * map_->dimension_ + lastColumn];
}

/**
 * @brief generates a lineiterator that will step through the map from start to end inclusively
 *
 * @param start the location of the first cell for the iterator to touch
 * @param end the location of the last cell the iterator will touch
 */
template <class TT>
typename ScrollingMap<TT>::LineIterator
ScrollingMap<TT>::beginLine(const RecPoint2D & start,
                   const RecPoint2D & end)
{
    IndexRange range;
//    convertGlobalPointToIndices(start,range.rs,range.cs);
//    convertGlobalPointToIndices(end,range.re,range.ce);
    pointToRowCol(start,range.rs,range.cs);                     // use the scrollingMap method instead
    pointToRowCol(end,range.re,range.ce);                       // use the scrollingMap method instead
    return LineIterator(range,*this);
}

/**
 * @brief generates an end LineIterator that can be used to check if a LineIterator has come to the end of a line
 */
template <class TT>
typename ScrollingMap<TT>::LineIterator
ScrollingMap<TT>::endLine(void)
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
inline ScrollingMap<TT>::LineIterator::LineIterator(const IndexRange &range, ScrollingMap<TT> & map): range_(range), map_(&map)
{
    int index = map.globalRowColToIndex(range_.rs, range_.cs);                   // alternate way
    curCell_ = &(map.cells_[index]);
    offEnd_= false;
    if (range_.rs==range_.re && range_.cs==range_.ce)
        almostOffEnd_ = true;
    else
        almostOffEnd_ = false;
    row_ = range_.rs;
    col_ = range_.cs;
    numRows_ = map.dimension_;
    numCols_ = map.dimension_;

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
inline bool ScrollingMap<TT>::LineIterator::isValid(void)
{
    if ( curCell_ )
    {
        return (row_==curCell_->row && col_==curCell_->column);
    }
    return false;
}

/**
 * @brief copy constructor
 */
template <class TT>
inline ScrollingMap<TT>::LineIterator::LineIterator(const LineIterator & other)
{
    (*this) = other;
}

/**
 * @brief void constructor, which effectively creates a LineIterator end case
 */
template <class TT>
inline ScrollingMap<TT>::LineIterator::LineIterator(void)
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
    map_ = NULL;
}


/**
 * @brief pre-increment operator
 */
template <class TT>
inline typename ScrollingMap<TT>::LineIterator &
ScrollingMap<TT>::LineIterator::operator++()
{
    int index;
    if (offEnd_)
        return *this;

    if (dc_ > dr_)
    {
        if (fraction_ >=0)
        {
            row_ += stepRow_;
            index = map_->globalRowColToIndex( row_, col_ );
            curCell_ = &(map_->cells_[index]);
            fraction_ -= dc_;
        }
        col_ += stepCol_;
        index = map_->globalRowColToIndex( row_, col_ );
        curCell_ = &(map_->cells_[index]);
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
            index = map_->globalRowColToIndex( row_, col_ );
            curCell_ = &(map_->cells_[index]);
            fraction_ -= dr_;
        }
        row_ += stepRow_;
        index = map_->globalRowColToIndex( row_, col_ );
        curCell_ = &(map_->cells_[index]);

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
inline typename ScrollingMap<TT>::LineIterator ScrollingMap<TT>::LineIterator::operator++(int)
{
    LineIterator ret(*this);
    ++(*this);
    return ret;
}

/**
 * @brief assignment operator
 */
template <class TT>
typename ScrollingMap<TT>::LineIterator & ScrollingMap<TT>::LineIterator::operator=(LineIterator const &other)
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
    map_ = other.map_;
    return *this;
}

/**
 * @brief equality operator
 */
template <class TT>
inline bool ScrollingMap<TT>::LineIterator::operator==(LineIterator const & other) const

{
    if (offEnd_==true && other.offEnd_==true)
        return true;
    return other.curCell_ == curCell_;
}

/**
 * @brief clears the map from the location of this iterator.
 * @param clearValue the value to set any cell in the map that is not this value to
 */
template <class TT>
void ScrollingMap<TT>::LineIterator::clearConnectedFromHere(const TT & clearValue, unsigned int maxDepth)
{
    map_->clearRecurser(clearValue, row_, col_, 1, maxDepth);
	// also hit all of the 8 connected neighbors to blur around this point, this avoids bressenham line artifacts from the iterator
	// we have to do it here, rather than in the recurser, to allow us to have a stop condition in the recurser
    map_->clearRecurser(clearValue, row_ - 1, col_ + 1 ,1, maxDepth );
    map_->clearRecurser(clearValue, row_ - 1, col_     ,1, maxDepth );
    map_->clearRecurser(clearValue, row_ - 1, col_ - 1 ,1, maxDepth );

    map_->clearRecurser(clearValue, row_ + 1, col_ + 1 ,1, maxDepth );
    map_->clearRecurser(clearValue, row_ + 1, col_     ,1, maxDepth );
    map_->clearRecurser(clearValue, row_ + 1, col_ - 1 ,1, maxDepth );

    map_->clearRecurser(clearValue, row_ , col_ - 1    ,1, maxDepth );
    map_->clearRecurser(clearValue, row_ , col_ + 1    ,1, maxDepth );
}


/**
 * @brief recursively flood clears the map from the given row column, any value that is not clear will be cleared.
 * @param clearValue the value to use to clear the map
 * @param r the global row to start the flood clear from
 * @param c the global column to start the flood clear from
 */
template <class TT>
void ScrollingMap<TT>::clearRecurser(const TT & clearValue, int row, int col, 
                                     unsigned int curDepth, unsigned int maxDepth)
{
    Cell & cell = index(row,col);
    if ( cell.value != clearValue )
    {
        cell.value = clearValue;
        if (curDepth < maxDepth)
        {
            clearRecurser(clearValue, row - 1, col + 1, curDepth+1, maxDepth  );
            clearRecurser(clearValue, row - 1, col    , curDepth+1, maxDepth  );
            clearRecurser(clearValue, row - 1, col - 1, curDepth+1, maxDepth  );
            
            clearRecurser(clearValue, row + 1, col + 1, curDepth+1, maxDepth  );
            clearRecurser(clearValue, row + 1, col    , curDepth+1, maxDepth  );
            clearRecurser(clearValue, row + 1, col - 1, curDepth+1, maxDepth  );
            
            clearRecurser(clearValue, row    , col - 1, curDepth+1, maxDepth  );
            clearRecurser(clearValue, row    , col + 1, curDepth+1, maxDepth  );
        }
    }

}

#endif //ifndef _SCROLLINGMAP_DEF_H_
