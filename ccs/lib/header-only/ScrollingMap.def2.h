#ifndef _SCROLLINGMAP_DEF2_H_
#define _SCROLLINGMAP_DEF2_H_

#ifndef _SCROLLINGMAP_H_
#error Should only be included from ScrollingMap.h
#endif

/**
 * @brief default constructor creates a 50mx50m scrolling map with 0.25m cells
 */
template <class TT>
ScrollingMap<TT>::ScrollingMap(void)
    :
      cellSize_m_( 0.0 ),
      invCellSize_( 0.0 ),
      size_m_( 0.0 ),
      dimension_( 0 ),
      boundary_( ),
      cells_(NULL),
      timeStamp_( ),
      useShallowCopy_( false ),
      compressOnSerialize_( false ),
      RC_( false ),
      voidedValue_( ),
      uncompressBuffer_(NULL),
      uncompressBufferSize_(0)
{
    mapSetup(50,0.25);
    RecPoint2D point(0,0);
    boundary_.setMinPoint( point );   // set the bounding box to be a point at zero-zero since non-RC
    boundary_.setMaxPoint( point );   // scrolling map has no bounding box
}

/**
 * @brief creates a size_m x size_m map with cellSize_m cells
 */
template <class TT>
ScrollingMap<TT>::ScrollingMap(double size_m, double cellSize_m)
    :
      cellSize_m_( cellSize_m ),
      invCellSize_( 0.0 ),
      size_m_( size_m ),
      dimension_( 0 ),
      boundary_( ),
      cells_(NULL),
      timeStamp_( ),
      useShallowCopy_( false ),
      compressOnSerialize_( false ),
      RC_( false ),
      voidedValue_( ),
      uncompressBuffer_(NULL),
      uncompressBufferSize_(0)
{
    mapSetup(size_m,cellSize_m);
    RecPoint2D point(0,0);
    boundary_.setMinPoint( point );   // set the bounding box to be a point at zero-zero since non-RC
    boundary_.setMaxPoint( point );   // scrolling map has no bounding box
}


/**
 * @brief creates a size_m x size_m map with cellSize_m cells and sets up row-column scrolling if told to do so
 */
template <class TT>
ScrollingMap<TT>::ScrollingMap(double size_m, double cellSize_m, bool RC, TT voidedValue)
    :
      cellSize_m_( cellSize_m ),
      invCellSize_( ),
      size_m_( size_m ),
      dimension_( 0 ),
      boundary_( ),
      cells_(NULL),
      timeStamp_( ),
      useShallowCopy_( false ),
      compressOnSerialize_( false ),
      RC_( RC ),
      voidedValue_( voidedValue ),
      uncompressBuffer_(NULL),
      uncompressBufferSize_(0)
{
    mapSetup(size_m,cellSize_m,0,0,1);

    // initialize the map so that it the cells point to a cohesive rectangle.  Ths approach relies on the idea that
    // we can initialize each cell to an 'unknown' value and that this is just as valid as reading that the cell is
    // "invalid".  This method is only called during row-column scrolling mode because not doing so causes the
    // row-column shifting to fail in certain circumstances
    if( RC_==1 )
    {
        double range = size_m_ / 2;
        RecPoint2D min( -range, -range );  // set the bounding box
        RecPoint2D max( range, range );    // set the bounding box
        RecPoint2D vec = max - min;
        boundary_.setMinMaxPoints( min, max );
        this->set(max+vec,voidedValue_);
        this->set(min,voidedValue_);
        RecPoint2D point;
        for( point.x =  -range + 0.5 * cellSize_m_; point.x < range; point.x += cellSize_m_ )
        {
            for( point.y =  -range + 0.5 * cellSize_m_; point.y < range; point.y += cellSize_m_ )
            {
                (*this).set( point, voidedValue_ );
            }
        }
    }
    else // else we called this form of constructor but told it not to use row-column scrolling.  Set bb to 0,0
    {
        RecPoint2D point(0,0);
        boundary_.setMinPoint( point );   // set the bounding box to be a point at zero-zero since non-RC
        boundary_.setMaxPoint( point );   // scrolling map has no bounding box
    }
}


/**
 * @brief copy consructor
 */
template <class TT>
ScrollingMap<TT>::ScrollingMap(const ScrollingMap<TT> & otherMap)
    : cells_(NULL),
      uncompressBuffer_(NULL),
      uncompressBufferSize_(0)
{
    *this = otherMap;
}

/**
 * @brief virtual destructor
 */
template <class TT>
ScrollingMap<TT>::~ScrollingMap(void)
{
    if (cells_)
        delete [] cells_;

    if (uncompressBuffer_)
        delete [] uncompressBuffer_;
}

/**
 * @brief accessor for protected member cellSize_m_
 */
template <class TT>
inline double ScrollingMap<TT>::getCellSize(void)
{
    return cellSize_m_;
}

/**
 * @brief accessor for protected member size_m_
 */
template <class TT>
inline double ScrollingMap<TT>::getSize(void)
{
    return size_m_;
}


/**
 * @brief accessor for protected member dimension_
 */
template <class TT>
inline int ScrollingMap<TT>::getDimension(void)
{
    return dimension_;
}

/**
 * @brief gets the data stored at a global coordinate in the map
 *
 * @return a reference to the data in the map
 *
 * @param globalPt the global points of the cell in the map we'd like to get data from
 * @param isValid this flag is set to true if the cell in the map corresponds to the queried location, false otherwise
 *
 * care should be taken to always check the isValid flag.  Otherwise, the data being accessed may not actually be the data requested
 */
template <class TT>
TT & ScrollingMap<TT>::get(const RecPoint2D & globalPt, bool & isValid)
{
    long row, col;
    pointToRowCol(globalPt,row,col);
    return getByRowCol(row,col, isValid);
}

/**
 * @brief gets the data stored at the global row/col coordinate
 * @return a reference to the data in the map
 * @param row the global row of the cell of interest
 * @param col the global col of the cell of interest
 * @param isValid this flag is set to true if the cell in the map corresponds to the queried location, false otherwise
 *
 * only use this method if you really know the row/col of the cell in question, otherwise use the get operators.  Be
 *sure to check isValid.
 */
template <class TT>
TT & ScrollingMap<TT>::getByRowCol(int row, int col, bool &isValid)
{
    Cell & cell = index(row,col);
    // we know the data is valid if the map row and column are the same as our query location
    isValid =  (cell.row==row && cell.column ==col);
    return cell.value;
}


/**
 * @brief sets the value at the given global coordinate and returns a reference to the data
 *
 * @return a reference to the data that has just been set
 *
 * @param globaPt the global coordinate of data to be set
 * @param value the value we wish to set at the given location
 * writing to a some location will overwrite whatever old data was stored in that location.
 */
template <class TT>
TT & ScrollingMap<TT>::set(const RecPoint2D & globalPt, const TT & value)
{
    long row, col;
    pointToRowCol(globalPt,row,col);  //get global row and column
    return setByRowCol(row,col,value);
}

/**
 * @brief sets the value at the given global row/column coordinate and returns a reference to the data
 * @return a reference to the data that has just been set
 * @param row the global row of the point we want to set
 * @param col the global column of the point we want to set
 * @param value the value we want to set
 *
 * this method should only be used if you really know the row/ col of interest, otherwise use the set accessors
 */
template <class TT>
TT & ScrollingMap<TT>::setByRowCol(int row, int col, const TT & value)
{
    Cell & cell = index(row,col);     //get cell at that global row and column

    // if we're in row-column scrolling, check to see if this cell is valid.  If not, fix it.
    if( RC_ == 1)
    {
        bool oops = 0;
        // if these don't match, then we have a disagreement on the column position
        if( cell.column != col)
        {
            colShift( cell.column, col );
            oops = 1;
        }
        // if these don't match, then we have a disagreement on the row position
        if( cell.row != row)
        {
            rowShift( cell.row, row );
            oops = 1;
        }
        cell.value = value;
    }
    else{
        cell.row = row;
        cell.column = col;
        cell.value = value;
    }

    return cell.value;                //return the value in the cell, probably just a check
}

/**
 * @brief gets the data stored at a global coordinate in the map
 *
 * @return a reference to the data in the map
 *
 * @param globalPt the global points of the cell in the map we'd like to get data from
 * @param isValid this flag is set to true if the cell in the map corresponds to the queried location, false otherwise
 *
 * care should be taken to always check the isValid flag.  Otherwise, the data being accessed may not actually be the data requested
 */
template <class TT>
TT & ScrollingMap<TT>::get(const RecPoint3D & globalPt, bool & isValid)
{
    return get(RecPoint2D(globalPt.x, globalPt.y), isValid);
}


/**
 * @brief sets the value at the given global coordinate and returns a reference to the data
 *
 * @return a reference to the data that has just been set
 *
 * @param globaPt the global coordinate of data to be set
 * @param value the value we wish to set at the given location
 * writing to a some location will overwrite whatever old data was stored in that location.
 */
template <class TT>
TT & ScrollingMap<TT>::set(const RecPoint3D & globalPt, const TT & value)
{
    return set(RecPoint2D(globalPt.x, globalPt.y), value);
}

template <class TT>
void ScrollingMap<TT>::colShift( long oldCol, const long newCol )
{
    // All cells in old column are incorrect.  Inc through all rows and set cells in that column to new column.
    // Just run through rows 0 to (dimention-1) so we don't have to mod them.
    // Use the new column for indexing since old and new column would mod to the same value.
    //
    // If it happens that we move to a new column that leaves some space between our map and the new column, we need to
    // move other columns as well to ensure that the map stays as a continuous block.  Therefore, after the column is
    // moved, move back towards the rest of the map and check the columns until we find a correct one or we're moved as
    // many columns as there are in the map.  This last part is necessary in case the map moves to a new location where
    // there is no overlap with the previous map's location.
    //
    // ***NOTE***  Two large issues.  Due to the iteration logic, the first index calculation has to use row = 0, even
    // though the offending cell may be in a different row.  Secondly, because of this, the if statement that checks if
    // the column is correct needs to be skipped the first time through.  At initialization, the cell at -1, -1 has a
    // different column value than the other cells in the column.  Therefore, if the check is done on 'index', which
    // points to a different cell, it is possible for a correct col to come back.  Skipping this check is valid since
    // the calling function confirms that a shift is needed.

    int index = globalRowColToIndex( 0, newCol);
    bool stopLoop = 0;
    int sign = (newCol - oldCol)/abs(newCol - oldCol);      // direction of additional row/col, later we'll increment in the opp dir
    int newCol_ = newCol;                                   // this is where the cells should be, don't modify newCol.  See below
    int count = 0;

    // we we shift by more than the size of the map, we have zero valid cols, so our shifting will never stop unless we
    // cap the number of columns shifted by the number of columns in the map
    while( (stopLoop == 0)&&(count<dimension_) )
    {
        if( (cells_[index].column != newCol_)||(count==0) )  //if the columns don't match, fix this column
        {
            for( int ii = 0; ii< dimension_; ii++)      // move all cells in this column to the new one
            {
                cells_[ index ].column = newCol_;
                cells_[ index ].value = voidedValue_;
                index += dimension_;                    // since we increment the row, just increment by dimension_
            }
            newCol_ -= sign;                            // increment the column where the cells should be pointing
            index = globalRowColToIndex( 0, newCol_);   // get the index for this new column at the zero row
            count++;
        }
        else
        {
            stopLoop = 1;
        }
    }

    // Now, shift the bounding box.  newCol has not been modified, so it points to the leading edge of the map.
    RecPoint2D max = boundary_.getMaxPoint();
    RecPoint2D min = boundary_.getMinPoint();
    // since we just moved a column, change the bounding box location to the new column values
    // dont forget to add a cellSize where appropriate, lest ye be have a fencepost problem
    if( sign > 0 )
    {
        max.y = (newCol+1)*cellSize_m_;                         // y-location of leading column
        min.y = (newCol - (dimension_ - 1) )*cellSize_m_;   // trailing column of map:  back dimension_ - 1 columns
    }
    else
    {
        min.y = (newCol)*cellSize_m_;                         // y-location of trailing column
        max.y = (newCol + (dimension_) )*cellSize_m_;   // leading column of map:  forward (dimension_ - 1) columns
    }
    boundary_.setMinMaxPoints(min,max);
}

template <class TT>
void ScrollingMap<TT>::rowShift( long oldRow, const long newRow )
{
    // All cells in old row are incorrect.  Inc through all columns and cells in that row to new value.
    // Just run through columns 0 to (dimention-1) so we don't have to mod them
    // Use the newRow for indexing since new and old row would mod to the same value.
    // ****NOTE**** See note above for comment on index using col = 0 and the count = 0 condition.

    // index = row*dimension_ + c, so start at col = 0 and just increment by 1 to more to new column
    int index = globalRowColToIndex( newRow, 0);
    bool stopLoop = 0;
    int sign = (newRow - oldRow)/abs(newRow - oldRow);  // direction of the additional col, later we'll be incrementing in the opp dir
    int newRow_ = newRow;               // this is were the cells should be pointing.  Don't modify newRow.  See below.
    int count = 0;

    // we we shift by more than the size of the map, we have zero valid rows, so our shifting will never stop unless we
    // cap the number of rows shifted by the number of rows in the map
    while( (stopLoop == 0)&&(count < dimension_) )
    {
        if( (cells_[index].row != newRow_)||(count==0) )
        {
            for( int ii = 0; ii< dimension_; ii++ )
            {
                cells_[ index ].row = newRow_;
                cells_[ index ].value = voidedValue_;
                index++;                                // increment the column, so just increment the index by one
            }
            newRow_ -= sign;                            // increment the row where the cells should be pointing
            index = globalRowColToIndex( newRow_, 0);   // get the index for this new row at the zero column
            count++;
        }
        else
        {
            stopLoop = 1;
        }
    }

    // Now, shift the bounding box.  newRow has not been modified, so it points to the leading edge of the map.
    RecPoint2D max = boundary_.getMaxPoint();
    RecPoint2D min = boundary_.getMinPoint();
    // since we just moved a row, change the bounding box location to the new column values
    if( sign > 0 )
    {
        max.x = (newRow + 1)*cellSize_m_;                         // x-location of leading row
        min.x = (newRow - (dimension_ - 1) )*cellSize_m_;   // trailing row of map-  back dimension_ - 1 columns
    }
    else
    {
        min.x = newRow*cellSize_m_;                         // x-location of trailing row
        max.x = (newRow + (dimension_) )*cellSize_m_;   // leading row of map- forward (dimension_ - 1) columns
    }
    boundary_.setMinMaxPoints(min,max);
}


template <class TT>
inline long ScrollingMap<TT>::mod(long i, long range) const
{
    long tmp = i%range;
    return (tmp<0?tmp+=range:tmp);
}

/**
 * @brief returns a reference to the cell in the map given the global index progvided
 *
 * @param row the global index that will have a modulus operator applied to it before index our physical array
  * @param col the global index that will have a modulus operator applied to it before index our physical array
 */
template <class TT>
typename ScrollingMap<TT>::Cell & ScrollingMap<TT>::index(long row, long col)
{
    int r,c;
    r = mod(row,dimension_);
    c = mod(col,dimension_);
    return cells_[r*dimension_+c];
}

/**
 * @brief converts a global point to a global row and column index
 */
template <class TT>
void ScrollingMap<TT>::pointToRowCol(const RecPoint2D & globalPt, long &row, long &col) const
{
    row = static_cast<int>(floor(globalPt.x*invCellSize_));
    col = static_cast<int>(floor(globalPt.y*invCellSize_));
}

/**
 * @brief converts a row,col to a global point
 */
template <class TT>
RecPoint2D ScrollingMap<TT>::rowColToGlobalPoint(long row, long col) const
{
    RecPoint2D global((row * cellSize_m_) + (cellSize_m_ / 2.0), (col * cellSize_m_) + (cellSize_m_ / 2.0));
    return global;
}

/**
 * @brief converts a global point to a global row and column index
 */
template <class TT>
void ScrollingMap<TT>::pointToRowCol(const RecPoint3D & globalPt, long &row, long &col) const
{
    row = static_cast<int>(floor(globalPt.x*invCellSize_));
    col = static_cast<int>(floor(globalPt.y*invCellSize_));
}

/**
 * @brief converts a row,col to a global row and column index
 */
template <class TT>
int ScrollingMap<TT>::globalRowColToIndex(long row, long col) const
{
    long index;
    long tempRow = mod( row, dimension_ );
    long tempCol = mod( col, dimension_ );
    index = tempRow*dimension_ + tempCol;
    return index;
}

/**
 * @brief Given a global point, converts it to the global point representing the center of the appropriate cell.
 */
template <class TT>
void ScrollingMap<TT>::getCellCenter(const RecPoint2D & globalPt, RecPoint2D & centerPoint)
{
    long row(0), col(0);
    pointToRowCol(globalPt, row, col);
    centerPoint = rowColToGlobalPoint(row, col);
}

/**
 * @brief Given a global point, converts it to the global point representing the center of the appropriate cell.
 */
template <class TT>
void ScrollingMap<TT>::getCellCenter(const RecPoint3D & globalPt, RecPoint3D & centerPoint)
{
    long row(0), col(0);
    pointToRowCol(globalPt, row, col);
    centerPoint = rowColToGlobalPoint(row, col);
}

/**
 * @brief assignment operator
 */
template <class TT>
void ScrollingMap<TT>::operator=(const ScrollingMap<TT> & otherMap) {

    // we're copying onto ourselves, let's not do anything
    if (this == &otherMap)
        return;

    const bool needAllocation = (otherMap.dimension_ != dimension_ || otherMap.RC_ != RC_);

    boundary_ = otherMap.boundary_;
    voidedValue_ = otherMap.voidedValue_;
    RC_ = otherMap.RC_;
    timeStamp_ = otherMap.timeStamp_;

    mapSetup(otherMap.size_m_,
             otherMap.cellSize_m_,
             otherMap.useShallowCopy_,
             otherMap.compressOnSerialize_,
             needAllocation
            );

    if (useShallowCopy_)
    {
        memcpy(static_cast<void*>(cells_),static_cast<void*>(otherMap.cells_),sizeof(Cell)*dimension_*dimension_);
    }
    else
    {
        int numCells = dimension_*dimension_;
        for (int i=0;i<numCells;i++)
        {
            cells_[i] = otherMap.cells_[i];
        }
    }
}


/**
 * @brief creates an iterator that will step over the region [min,max) points (note: max specifically not included)
 */
template <class TT>
typename ScrollingMap<TT>::iterator
ScrollingMap<TT>::begin(const RecPoint2D & min,
                        const RecPoint2D & max)
{
    IndexRange range;

    // we subtract 1 percent of the cellSize from max to handle the open-ended inclusion
    // this allows begin(getBounds()) to exactly iterate over the whole map
    RecPoint2D epsilon(0.01 * cellSize_m_,0.01 * cellSize_m_);

    pointToRowCol(min,range.rs,range.cs);
    pointToRowCol(max - epsilon,range.re,range.ce);
    range.normalize();
    iterator ret(range,*this);
    return ret;
}

/**
 * @brief returns an iterator to step over the given rectangular region
 */
template <class TT>
typename ScrollingMap<TT>::iterator
ScrollingMap<TT>::begin(const RecAxisAlignedBox2D & box)
{
    return begin(box.getMinPoint(),box.getMaxPoint());
}

/**
 * @brief returns an iterator to step over the entire map as-is
 */
template <class TT>
typename ScrollingMap<TT>::iterator
ScrollingMap<TT>::begin()
{
    return begin(getBounds());
}

/**
 * @brief returns an iterator that can be used to test against when iterating over regions
 */
template <class TT>
typename ScrollingMap<TT>::iterator
ScrollingMap<TT>::end(void)
{
    iterator ret;
    return ret;
}

/**
 * @brief this goes through and clears values in the map.  It will not cause the values to be invalid, just set to the
 * voidedValue if the map is not allowed to use shallow copy.  If it is shallow copyable, we will void all of the cells too.
 */
template <class TT>
void ScrollingMap<TT>::clear(void)
{
    if (useShallowCopy_)
    {
        memset( cells_, 0, sizeof(Cell)*dimension_*dimension_);
    }
    else
    {
        for (iterator it= begin(); it!= end(); ++it)
        {
            it.set(voidedValue_);
        }
    }
}

/**
 * Sets two cells equal to 0, at the corners of the map, such that the center of the map should be focused on globalPt.
 */
template <class TT>
void ScrollingMap<TT>::centerAt(const RecPoint2D & globalPt)
{
    RecPoint2D max(globalPt + RecPoint2D(size_m_/2.0, size_m_/2.0));
    RecPoint2D min(globalPt - RecPoint2D(size_m_/2.0, size_m_/2.0));

    this->set(min, TT());
    this->set(max, TT());
}

/**
 * @brief this sets up the map size and allocates memory
 */
template <class TT>
void ScrollingMap<TT>::mapSetup(double size_m,
                                double cellSize_m,
                                bool useShallowCopy,
                                bool compressOnSerialize,
                                bool needAllocation
                               )
{
    dimension_ = (int)floor(size_m/cellSize_m +0.5);
    size_m_ = size_m;
    cellSize_m_ = cellSize_m;
    invCellSize_ = 1.0/cellSize_m;

    if (needAllocation) {
        if (cells_)
            delete [] cells_;

        cells_ = new Cell[dimension_*dimension_];
        // this next line just makes sure that cell at -1,-1 is also invalid
        // the constructor for cell, sets row and column to -1,-1 which
        // effectively marks every other cell in the map as invalid
        if(!RC_)
            index(-1,-1).column = 0;
    }

    useShallowCopy_ = useShallowCopy;
    compressOnSerialize_ = compressOnSerialize;
}



/**
 * @brief tha map serialization save function
 */
/*
template <class TT>
template <class Archive>
void ScrollingMap<TT>::save(Archive &ar, unsigned int version) const
{
    ar << cellSize_m_ <<size_m_
       <<  timeStamp_
       << useShallowCopy_ << compressOnSerialize_ << RC_ << voidedValue_ << boundary_;

    // archive versioning confirmation.  This can be removed once we are all settled
    static int once = 0;
    if(!once)
    {
        once = 1;
        std::cerr<<"SAVE ScrollingMap with element size "<<sizeof(TT)
                 <<" and version "<<version<<" (compression = "
                 <<(compressOnSerialize_ ? "ON )" : "OFF )")<<std::endl;
    }

    // version 1 (and after) cuts down on bandwidth by eliminating the cell row & column serialization and just sending the body
    // for ScrollingByteMaps, this means we get to save 11/12 bytes per cell, which is a 92% reduction in net/disk bandwidth
    // note: this is only guaranteed to work for RC scrolling maps
    if(RC_ && version >= 1)
    {
        if(useShallowCopy_)
        {
            // shallow copy piles things into a vector and serializes the vector en-masse
            std::vector<TT> vec;
            vec.reserve(dimension_ * dimension_);
            for (int i=0; i< dimension_ * dimension_; i++)
            {
                vec.push_back(cells_[i].value);
            }
            // the usage of &vec[0] assumes that the vector class will maintain data contiguity
            if(version >= 2 && compressOnSerialize_)
            {
                const unsigned long uncompressedMapSize = sizeof(TT[1])*dimension_*dimension_;
                const unsigned long neededCompressBufferSize = compressBound(uncompressedMapSize);
                unsigned char *compressBuffer = new unsigned char[neededCompressBufferSize];
                // now we do the compression
                unsigned long compressedDataSize = neededCompressBufferSize;
                const int zerror = compress(compressBuffer,
                                            &compressedDataSize,
                                            reinterpret_cast<unsigned char* >(&vec[0]),
                                            uncompressedMapSize
                    );

                if (zerror != Z_OK)
                    std::cerr << "compression error!" << std::endl;

                // now we serialize it
                ar << compressedDataSize;
                ar << boost::serialization::make_binary_object(static_cast<void *>(compressBuffer),
                                                               compressedDataSize);
                delete [] compressBuffer;
            } else {
                ar << boost::serialization::make_binary_object(static_cast<void *>(&vec[0]),dimension_*dimension_*sizeof(TT[1]));
            }
        } else {
            // normal copy serializes the contents one at a time
            // this is safer, but slower (by ~5x for ScrollingByteMaps
            for (int ii=0; ii< dimension_ * dimension_; ii++)
            {
                ar << const_cast<const Cell*>(cells_)[ii].value;
            }
        }

        // note: compression support is not yet implemented

    } else {

        // old format, serializing the whole Cell
        if (!compressOnSerialize_)
        {
            if (useShallowCopy_)
            {
                ar << boost::serialization::make_binary_object(static_cast<void *>(cells_),dimension_*dimension_*sizeof(Cell));
            }
            else
            {
                for (int i=0; i< dimension_ * dimension_; i++)
                {
                    ar << const_cast<const Cell*>(cells_)[i];
                }
            }
        }
        else
        {
            const unsigned long uncompressedMapSize = sizeof(Cell)*dimension_*dimension_;
            const unsigned long neededCompressBufferSize = compressBound(uncompressedMapSize);
            unsigned char *compressBuffer = new unsigned char[neededCompressBufferSize];

            // now we do the compression
            unsigned long compressedDataSize = neededCompressBufferSize;
            const int zerror = compress(compressBuffer,
                                        &compressedDataSize,
                                        reinterpret_cast<unsigned char* >(cells_),
                                        uncompressedMapSize
                );

            if (zerror != Z_OK)
                std::cerr << "compression error!" << std::endl;

            // now we serialize it
            ar << compressedDataSize;
            ar << boost::serialization::make_binary_object(static_cast<void *>(compressBuffer),
                                                           compressedDataSize);

            delete [] compressBuffer;
        }
    }
}
*/


/**
 * @brief the map serialization load function
 */
/*
template <class TT>
template < class Archive>
void ScrollingMap<TT>::load(Archive &ar, unsigned int version)
{
    const double oldSize_m = size_m_;
    const double oldCellSize_m = cellSize_m_;
    const bool oldRC = RC_;

    ar >> cellSize_m_  >> size_m_
       >> timeStamp_
       >> useShallowCopy_ >> compressOnSerialize_ >> RC_ >> voidedValue_ >> boundary_;

    mapSetup(size_m_,cellSize_m_,useShallowCopy_, compressOnSerialize_,
             (cellSize_m_ != oldCellSize_m) || (oldSize_m != size_m_) || RC_ != oldRC);

    static int once = 0;
    if(!once)
    {
        once = 1;
        std::cerr<<"LOAD ScrollingMap with element size "<<sizeof(TT)
                 <<" and version "<<version<<" (compression = "
                 <<(compressOnSerialize_ ? "ON )" : "OFF )")<<std::endl;
    }

    // again, versions at and after 1 serialize the data only
    // note: this is only guaranteed toi work for RC scrolling maps
    if(RC_ && version >= 1)
    {

        // poke max, then min to guarantee that the cells are aligned
        // this might be made a little smarter, as it could conceivably
        // waste an O(dimension^2) shift in the case of whole-map differences
        // but.. those differences should happen infrequently enough for us to not care

        RecVector2D halfCell(0.5 * cellSize_m_, 0.5 * cellSize_m_);
        RecPoint2D pt = boundary_.getMaxPoint() - halfCell;
        this->set(pt,voidedValue_);
        pt = boundary_.getMinPoint() + halfCell;
        this->set(pt,voidedValue_);

        if(useShallowCopy_)
        {
            std::vector<TT> vec;
            vec.reserve(dimension_ * dimension_);
            if(version >= 2 && compressOnSerialize_)
            {
                unsigned long uncompressedMapSize = sizeof(TT[1])*dimension_*dimension_;
                unsigned long compressedDataSize;
                ar >> compressedDataSize;

                if (compressedDataSize > uncompressBufferSize_)
                {
                    if (uncompressBuffer_)
                        delete [] uncompressBuffer_;
                    // we don't bother allocating just to the size needed, we allocate to the maxium data size
                    // for this map size- the assumption is we will probably be sending several maps this size
                    // and we don't want to reallocate, just because the first map we get is efficiently compressed
                    uncompressBufferSize_ = compressBound(uncompressedMapSize);
                    uncompressBuffer_ = new unsigned char[uncompressBufferSize_];
                }

                ar >> boost::serialization::make_binary_object(static_cast<void *>(uncompressBuffer_),
                                                               compressedDataSize);

                const int zerror = uncompress(reinterpret_cast<unsigned char*>(&vec[0]),
                                              &uncompressedMapSize,
                                              uncompressBuffer_,
                                              compressedDataSize
                    );

                if (zerror != Z_OK)
                    std::cerr << "uncompression error!" << std::endl;
            } else {
                // again, the usage of &vec[0] is a little tricky
                ar >> boost::serialization::make_binary_object(static_cast<void *>(&vec[0]),dimension_*dimension_*sizeof(TT[1]));
            }
            // push the values into the cells
            for (int i=0; i< dimension_ * dimension_; i++)
            {
                cells_[i].value = vec[i];
            }
        } else {
            // push the archive directly into the cells
            for (int i=0; i< dimension_ * dimension_; i++)
            {
                ar >> cells_[i].value;
            }
        }
    } else {
        if (!compressOnSerialize_)
        {
            if (useShallowCopy_)
            {
                ar >> boost::serialization::make_binary_object(static_cast<void *>(cells_),dimension_*dimension_*sizeof(Cell));
            }
            else
            {
                for (int i=0; i< dimension_ * dimension_; i++)
                    ar >> cells_[i];
            }
        }
        else
        {
            unsigned long uncompressedMapSize = sizeof(Cell)*dimension_*dimension_;
            unsigned long compressedDataSize;

            ar >> compressedDataSize;

            if (compressedDataSize > uncompressBufferSize_)
            {
                if (uncompressBuffer_)
                    delete [] uncompressBuffer_;

                // we don't bother allocating just to the size needed, we allocate to the maxium data size
                // for this map size- the assumption is we will probably be sending several maps this size
                // and we don't want to reallocate, just because the first map we get is efficiently compressed
                uncompressBufferSize_ = compressBound(uncompressedMapSize);
                uncompressBuffer_ = new unsigned char[uncompressBufferSize_];
            }

            ar >> boost::serialization::make_binary_object(static_cast<void *>(uncompressBuffer_),
                                                           compressedDataSize);

            const int zerror = uncompress(reinterpret_cast<unsigned char*>(cells_),
                                          &uncompressedMapSize,
                                          uncompressBuffer_,
                                          compressedDataSize
                );

            if (zerror != Z_OK)
                std::cerr << "uncompression error!" << std::endl;
        }
    }
}
*/



/**
 * @brief a method that returns a filtered map, where each cell in the destination map contains the nth largest value of
 * its neighbors in the srcMap.  This is a generalized median filter.
 */
template <class TT>
void filterNth(unsigned int nth, ScrollingMap<TT> & destMap, ScrollingMap<TT> &srcMap)
{

    if (nth > 8)
        return;

    int dimension = srcMap.dimension_;
    if (dimension==0)
        return;

    typename ScrollingMap<TT>::Cell * src = srcMap.cells_;


    destMap = srcMap;

    typename ScrollingMap<TT>::Cell * dst = destMap.cells_;

    int  x, y;
    TT t;

#define MINMAX(a,b) \
  {if (a> b) {t=a;a=b;b=t;}}

    // from openCV with some minor tweaks to make it templatized and work with our cell structures
    for( y = 0; y < dimension; y++, dst += dimension )
    {
        const typename ScrollingMap<TT>::Cell* src0 = src + dimension*(y-1);
        const typename ScrollingMap<TT>::Cell* src1 = src0 + dimension;
        const typename ScrollingMap<TT>::Cell* src2 = src1 + dimension;
        if( y == 0 )
                src0 = src1;
            else if( y == dimension - 1 )
                src2 = src1;

            for( x = 0; x < 2; x++ )
            {
                int x0 = x < 1 ? x : dimension - 3 + x;
                int x2 = x < 1 ? x + 1 : dimension - 2 + x;
                int x1 = x < 1 ? x0 : x2;

                TT p[8];
                p[0] = src0[x0].value; p[1] = src0[x1].value; p[2] = src0[x2].value;
                p[3] = src1[x0].value; p[4] = src1[x1].value; p[5] = src1[x2].value;
                p[6] = src2[x0].value; p[7] = src2[x1].value; p[8] = src2[x2].value;

                sort(&p[0], &p[8]);
                dst[x1].value = p[nth];

            }
            for( x = 1; x < dimension - 1; x++ )
            {
                TT p[8];
                p[0] = src0[x-1].value; p[1] = src0[x].value; p[2] = src0[x+1].value;
                p[3] = src1[x-1].value; p[4] = src1[x].value; p[5] = src1[x+1].value;
                p[6] = src2[x-1].value; p[7] = src2[x].value; p[8] = src2[x+1].value;


                sort(&p[0], &p[8]);
                dst[x].value = p[nth];

            }
        }
}

/**
 * @brief edt of binary image using squared distance
 *
 * @param bandwidth: bandwidth of exp function
 * @param MAX_SQUARED_DISTANCE: maximum squared distance between points and obstacles
 * Note: can only used by Byte map
 */
template <class TT>
void ScrollingMap<TT>::edtBinaryMap(double bandwidth, double maxSquaredDistance) {
    int dim = getDimension();
    double *map = new double[dim*dim];
    unsigned char OBSTACLE = UCHAR_MAX;
    bool isValid = false;
    RecPoint2D index;
    RecAxisAlignedBox2D bounds = getBounds();
    double cellSize = getCellSize();
    int x=0,y=0;
    for (index.x = bounds.getMinPoint().x; index.x <bounds.getMaxPoint().x; index.x+=cellSize)
    {
        y=0;
        for (index.y = bounds.getMinPoint().y; index.y < bounds.getMaxPoint().y; index.y+=cellSize)
        {
            if(get(index,isValid) == OBSTACLE)
                map[y*dim+x] = 0;
            else
                map[y*dim+x] = maxSquaredDistance;
            ++y;
        }
        ++x;
    }
    edt2D(map,dim,dim,maxSquaredDistance);
    x=0;
    y=0;
    for (index.x = bounds.getMinPoint().x; index.x <bounds.getMaxPoint().x; index.x+=cellSize)
    {
        y=0;
        for (index.y = bounds.getMinPoint().y; index.y < bounds.getMaxPoint().y; index.y+=cellSize)
        {
            set(index,(unsigned char)( UCHAR_MAX * exp( -1/bandwidth * map[y*dim+x] ) + 0.5));
            ++y;
        }
        ++x;
    }
    delete []map;
}

/* edt of 2d function using squared distance */
template <class TT>
void ScrollingMap<TT>::edt2D(double *map, int width, int height, double maxSquaredDistance) {
    double *f = new double[std::max(width,height)];
    // transform along columns
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            f[y] = map[y*width+x];
        }
        double *d = edt1D(f, height, maxSquaredDistance);
        for (int y = 0; y < height; y++) {
            map[y*width+x] = d[y];
        }
        delete [] d;
    }
    // transform along rows
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            f[x] = map[y*width+x];
        }
        double *d = edt1D(f, width, maxSquaredDistance);
        for (int x = 0; x < width; x++) {
            map[y*width+x] = d[x];
        }
        delete [] d;
    }
    delete f;
}

/* edt of 1d function using squared distance */
template <class TT>
double * ScrollingMap<TT>::edt1D(double *f, int n, double maxSquaredDistance) {
    double *d = new double[n];
    int *v = new int[n];
    double *z = new double[n+1];
    int k = 0;
    v[0] = 0;
    z[0] = -maxSquaredDistance;
    z[1] = +maxSquaredDistance;
    for (int q = 1; q <= n-1; q++) {
        double s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
        while (s <= z[k]) {
            k--;
            s  = ((f[q]+square(q))-(f[v[k]]+square(v[k])))/(2*q-2*v[k]);
        }
        k++;
        v[k] = q;
        z[k] = s;
        z[k+1] = +maxSquaredDistance;
    }

    k = 0;
    for (int q = 0; q <= n-1; q++) {
        while (z[k+1] < q) k++;
        d[q] = square(q-v[k]) + f[v[k]];
    }
    delete [] v;
    delete [] z;
    return d;
}

/*
// the long version of boost class versioning for templates
namespace boost {
    namespace serialization {
        template<class TT>
        struct version< ScrollingMap<TT> >
        {
            BOOST_STATIC_CONSTANT(unsigned int, value = 2);
        };
    } // namespace serialization
} // namespace boost
*/

#endif //ifndef _SCROLLINGMAP_DEF2_H_
