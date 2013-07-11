
#ifndef _MAP_DEF2_H_
#define _MAP_DEF2_H_

#ifndef _MAP_H_
#error Should only be included from Map.h
#endif

/**
 * @brief converts map relative eastings to a column index
 */
template <class TT>
inline int Map<TT>::eastingsToCol(double mapE_m) const
{
    return static_cast<int>(floor((mapE_m+widthBy2_m_)*invCellSize_));
};

/**
 * @brief converts map relative northings to a row index
 */
template <class TT>
inline int Map<TT>::northingsToRow(double mapN_m) const
{
    return static_cast<int>(floor((mapN_m+lengthBy2_m_)*invCellSize_));
};

/**
 * @brief converts a column index to map relative eastings
 */
template <class TT>
inline double Map<TT>::colToEastings(int c) const
{
    return -widthBy2_m_ + c*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief converts a column position to map relative eastings
 */
template <class TT>
inline double Map<TT>::colToEastings(double c) const
{
    return -widthBy2_m_ + c*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief converts a row index to map relative northings
 */

template <class TT>
inline double Map<TT>:: rowToNorthings(int r) const
{
    return -lengthBy2_m_ + r*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief converts a row position to map relative northings
 */

template <class TT>
inline double Map<TT>:: rowToNorthings(double r) const
{
    return -lengthBy2_m_ + r*cellSize_m_ + cellSize_m_/2;
};

/**
 * @brief sets the map origin to a point near the passed origin that is a multiple of the specified cell size
 */
template <class TT>
inline void Map<TT>::setOrigin(const RecPoint2D & ori)
{
    int xCellCnt  = static_cast<int>(ori.x/cellSize_m_+0.5);
    int yCellCnt = static_cast<int>(ori.y/cellSize_m_+0.5);
    origin_.x = xCellCnt*cellSize_m_;
    origin_.y = yCellCnt*cellSize_m_;
};



/**
 * @brief generates a lineiterator that will step through the map from start to end inclusively
 *
 * @param start the location of the first cell for the iterator to touch
 * @param end the location of the last cell the iterator will touch
 */
template <class TT>
typename Map<TT>::LineIterator
Map<TT>::beginLine(const RecPoint2D & start,
                   const RecPoint2D & end) const
{
    IndexRange range;
    convertGlobalPointToIndices(start,range.rs,range.cs);
    convertGlobalPointToIndices(end,range.re,range.ce);
    return LineIterator(range,*this);
}

/**
 * @brief generates an end LineIterator that can be used to check if a LineIterator has come to the end of a line
 */
template <class TT>
typename Map<TT>::LineIterator
Map<TT>::endLine(void) const
{
     LineIterator ret;
     return ret;
}

/**
 * @brief creates an iterator that will step over the entire map
 */
template <class TT>
typename Map<TT>::iterator
Map<TT>::begin(void) const
{
    IndexRange range;
    range.rs = 0; range.cs = 0;
    range.re = numRows_; range.ce = numCols_;
    return iterator(range,*this);
};


/**
 * @brief returns an iterator that will step over the cells in the defined rectangular region
 *
 * @return an iterator that will step over the passed rectangular region
 *
 * @param min the most south westerly point of the map we wish to iterate over
 * @param max the most north easterly point of the map we with to iterate over
 *
 * this system will return an iterator that steps over the rectangular region defined by min and max.
 * The region is clipped to the map dimensions.
 */
template <class TT>
typename Map<TT>::iterator Map<TT>::begin(const RecPoint2D &min,
                                          const RecPoint2D &max) const
{
    IndexRange range;
    convertGlobalPointToIndices(min,range.rs,range.cs);
    convertGlobalPointToIndices(max,range.re,range.ce);
    range.normalize();
    range.rs = std::min(std::max(0L,range.rs),numRows_);
    range.re = std::min(std::max(0L,range.re),numRows_);
    range.cs = std::min(std::max(0L,range.cs),numCols_);
    range.ce = std::min(std::max(0L,range.ce),numCols_);

    return iterator(range,*this);
}

/**
 * @brief returns an iterator that will step over the cells in the defined rectangular region
 */
template <class TT>
typename Map<TT>::iterator Map<TT>::begin(const RecAxisAlignedBox2D & box) const
{
    return begin(box.getMinPoint(),box.getMaxPoint());
}

/**
 * @brief returns an iterator that defines the end of an iterated region
 */
template <class TT>
typename Map<TT>::iterator Map<TT>::end() const
{
    iterator end;
    return end;
}

/**
 * @brief calculates the row and column for a passed global point
 */
template <class TT>
void Map<TT>::convertGlobalPointToIndices(const RecPoint2D & globalPt,long &row, long &col) const
{
    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
}

/**
 * @brief calculates the global point position for given row and column indices
 */
template <class TT>
void Map<TT>::convertIndicesToGlobalPoint(const int &row, const int &col, double &x, double &y) const
{
    x = rowToNorthings(row) + origin_.x;
    y = colToEastings(col) + origin_.y;
}

/**
 * @brief calculates the global point position for given row and column position (not necessarily integer indices)
 */
template <class TT>
void Map<TT>::convertIndicesToGlobalPoint(const double &row, const double &col, double &x, double &y) const
{
    x = rowToNorthings(row) + origin_.x;
    y = colToEastings(col) + origin_.y;
}

/**
 * @brief checks wether the passed point is within the bound of the map
 */
template <class TT>
bool  Map<TT>::inBounds(const RecPoint2D & globalPt) const
{
    int row,col;
    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
    return ((col >= 0 && col <numCols_) && (row >= 0 && row < numRows_));
}

/**
 * @brief returns a reference to the content of the cell at the globalPt.
 *
 * @return a reference to the cell
 *
 * @param globalPt the location we want to get a cell for
 *
 * if the point is out side of the map, theis function will fail an assertion so it is important to check inBounds
 * before calling operator()
 */
template <class TT>
TT & Map<TT>::operator()(const RecPoint2D & globalPt)
{
    int row, col;
    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}

/**
 * @brief returns a const reference to the contetn of the cell at globalPt
 */
template <class TT>
const TT & Map<TT>::operator()(const RecPoint2D & globalPt) const
{
    int row, col;

    col = eastingsToCol(globalPt.y-origin_.y);
    row = northingsToRow(globalPt.x-origin_.x);
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}

/**
 * @brief returns a reference to the contents of the cell at (row,col)
 */
template <class TT>
TT & Map<TT>::operator()(int row, int col)
{
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}

/**
 * @brief returns a const reference to the contents of the cell at (row,col)
 */
template <class TT>
const TT & Map<TT>::operator()(int row, int col) const
{
    BOOST_ASSERT(col >= 0 && col <numCols_);
    BOOST_ASSERT(row >= 0 && row < numRows_);
    return cells_[col+row*numCols_];
}


template <class TT>
Map<TT>::Map(void)
{
    mapSetup(RecPoint2D(0,0),0,0,0);
}

/**
 * @brief constructs a map with it's center at the passed orign, and with given width, length and cell size.
 */
template <class TT>
Map<TT>::Map(const RecPoint2D &origin, double length_m, double width_m, double cellSize_m)
{
    mapSetup(origin,length_m,width_m,cellSize_m);
}

/**
 * @brief assignment operator
 * if useShallowCopy is set, a memcopy is performed rather than an element by element copy.
 */
template <class TT>
void Map<TT>::operator=(const Map<TT> & otherMap)
{
    if (this==&otherMap)
        return;

    bool needReAlloc= (otherMap.numRows_!=numRows_) || (otherMap.numCols_!=numCols_);



    if (NULL!=cells_ || !needReAlloc)
        delete[] cells_;
    mapSetup(otherMap.origin_, otherMap.length_m_,
             otherMap.width_m_,otherMap.cellSize_m_, needReAlloc);

    if (useShallowCopy_)
    {
        memcpy(static_cast<void*>(cells_),static_cast<void*>(otherMap.cells_),sizeof(TT)*numRows_*numCols_);
    }
    else
    {
        int numCells = numRows_*numCols_;
        for (int i=0;i<numCells;i++)
        {
            cells_[i] = otherMap.cells_[i];
        }
    }
}

/**
 * @brief copy constructor
 */
template <class TT>
Map<TT>::Map(const Map<TT> & otherMap)
{
    if (this==&otherMap)
        return;

    mapSetup(otherMap.origin_, otherMap.length_m_, otherMap.width_m_,otherMap.cellSize_m_);

    if (useShallowCopy_)
    {
        memcpy(static_cast<void*>(cells_),static_cast<void*>(otherMap.cells_),sizeof(TT)*numRows_*numCols_);
    }
    else
    {
        int numCells = numRows_*numCols_;
        for (int i=0;i<numCells;i++)
        {
            cells_[i] = otherMap.cells_[i];
        }
    }
}

/**
 * @brief virtual destructor
 */
template <class TT>
Map<TT>::~Map(void)
{
    delete[] cells_;
}

/**
 * @brief sets up the map and allocates memory
 */
template <class TT>
void Map<TT>::mapSetup(const RecPoint2D & origin, double length_m, double width_m, double cellSize_m, bool useShallowCopy, bool compressOnSerialize, bool needAllocation)
{
    useShallowCopy_ = useShallowCopy;
    compressOnSerialize_ = compressOnSerialize;

    if (cellSize_m ==0) {
        numRows_ = 0;
        numCols_ = 0;
        cells_ = NULL;
        cellSize_m_ =0.0;
        invCellSize_ = 0;
        width_m_=0.0;
        widthBy2_m_ = 0.0;
        length_m_ =0.0;
        lengthBy2_m_= 0.0;
  } else {
      numRows_ = (int)floor(length_m/cellSize_m +0.5);
      numCols_ = (int)floor(width_m/cellSize_m + 0.5);
      if (needAllocation)
          cells_ = new TT[numRows_*numCols_];

      cellSize_m_ = cellSize_m;
      invCellSize_ = 1/cellSize_m_;
      width_m_ = numCols_*cellSize_m_;
      widthBy2_m_ = width_m_/2.0;
      length_m_ = numRows_ *cellSize_m_;
      lengthBy2_m_ = length_m_/2.0;
  }

    setOrigin(origin);
}

/**
 * @brief returns the bounds of the map
 */
template <class TT>
RecAxisAlignedBox2D  Map<TT>::getBounds(void) const
{
    RecAxisAlignedBox2D  ret(origin_.x-widthBy2_m_,origin_.x+widthBy2_m_,
                             origin_.y-lengthBy2_m_,origin_.y+lengthBy2_m_);
    return ret;
}


/**
 * @brief fills all cells of the map with the given value
 */
template <class TT>
void Map<TT>::fill(const TT & value)
{
    for (iterator it= begin(); it!=end();++it)
        *it =value;
}

/**
 * @brief fills the passed axis aligned bounding box
 */
template <class TT>
void Map<TT>::fill(const RecAxisAlignedBox2D & box, const TT & value)
{
    for (iterator it=begin(box); it!=end(); ++it)
        *it = value;
}


/**
 * @brief calculates areas where data may have changed.
 *
 * @param otherMap to which this map is compared.
 *
 * @param changeMap the map is filled with true or false indicating if the region has changed in otherMap compared to
 * this map
 *
 * Any place the otherMap either differs from this map or any area where othermap doesn't overlap with this map will be
 * marked as changed in changeMap
 */
template <class TT>
void Map<TT>::calculateChangeMap(const Map<TT> & otherMap, Map<bool> & changeMap)
{
    // anywhere the othermap overlaps the change map, there could be change
    changeMap.fill(otherMap.getBounds(),true);


    //calculate the area that is common to both maps and the change map
    RecAxisAlignedBox2D  overlapRegion = changeMap.getBounds().
        overlapArea(
            otherMap.getBounds()
            ).overlapArea(getBounds());

    // now iterate over the the three maps, setting the changed flag to true anywhere the maps differ
    iterator myIt, otherIt;
    Map<bool>::iterator changeIt;
    myIt = begin(overlapRegion);
    otherIt = otherMap.begin(overlapRegion);
    changeIt = changeMap.begin(overlapRegion);

    BOOST_ASSERT(changeMap.getCellSize()==getCellSize() &&
                 otherMap.getCellSize() == getCellSize() &&
                 "maps must have the same cellsizes");

    for (/**/;myIt!=end();++myIt, ++otherIt, ++changeIt)
        *changeIt = *myIt!=*otherIt;

}

template<class TT>
void Map<TT>::toIplImage(IplImage*& image)
{
  // deallocate if the image is the wrong size - note that this assumes that
  // the image is either allocated and set up, but set up for a different sized
  // map, or it is not allocated at all.  Behavior is undefined if just the
  // header is passed in.
  if(image &&
     (image->width != numRows_ || image->height != numCols_)
     )
    {
      cvReleaseImage(&image);
    }

  // if the image is null (released from above or never allocated), then
  // allocate the header and the image
  if(!image)
    {
      CvSize sze = cvSize(numRows_, numCols_);

      image = cvCreateImage(sze, IPL_DEPTH_8U, 1);
    }

  cvSetImageData(image, static_cast<void*>(cells_), numCols_);
}



/**
 * @brief tha map serialization save function
 */
/*
template <class TT>
template <class Archive>
void Map<TT>::save(Archive &ar, unsigned int version) const
{
    static unsigned char *compressBuffer=NULL; // used to keep around a compression buffer that is only allocated once.
    static unsigned long compressBufferSize=0;


    ar << cellSize_m_ << width_m_
       << length_m_ << origin_ << timeStamp_
       << useShallowCopy_ << compressOnSerialize_;

    if (!compressOnSerialize_)
    {
        if (useShallowCopy_)
        {
            ar << boost::serialization::make_binary_object(static_cast<void *>(cells_),numRows_*numCols_*sizeof(TT));
        }
        else
        {
            for (int i=0; i< numRows_ * numCols_; i++)
                ar << const_cast<const TT *>(cells_)[i];
        }
    }
    else
    {
        unsigned long uncompressedMapSize = sizeof(TT)*numCols_*numRows_;
        unsigned long neededCompressBufferSize = compressBound(uncompressedMapSize);
        // check if we need to make our buffer bigger
        if (neededCompressBufferSize> compressBufferSize)
        {
            if (compressBuffer)
                delete[] compressBuffer;
            compressBuffer = new unsigned char[neededCompressBufferSize];
            compressBufferSize = neededCompressBufferSize;
        }

        // now we do the compression
        unsigned long compressedDataSize = compressBufferSize;
        int zerror;
        zerror = compress(compressBuffer, &compressedDataSize,
                          reinterpret_cast<unsigned char* >(cells_), uncompressedMapSize);
        if (zerror!=Z_OK)
            std::cerr << "compression error!" << std::endl;
        ar << compressedDataSize;
        ar << boost::serialization::make_binary_object(static_cast<void *>(compressBuffer),
                                                       compressedDataSize);

    }

}
*/


/**
 * @brief the map serialization load function
 */
/*
template <class TT>
template < class Archive>
void Map<TT>::load(Archive &ar, unsigned int version)
{
    static unsigned char * uncompressBuffer =NULL;
    static unsigned long uncompressBufferSize =0;

    double oldCellSize, oldWidth, oldLength;
    oldCellSize= cellSize_m_; oldWidth = width_m_; oldLength = length_m_;


    ar >> cellSize_m_  >> width_m_
       >> length_m_ >> origin_ >> timeStamp_
       >> useShallowCopy_ >> compressOnSerialize_;


    mapSetup(origin_,length_m_,width_m_,cellSize_m_,useShallowCopy_, compressOnSerialize_,
             !(oldCellSize==cellSize_m_ && oldWidth ==width_m_ && oldLength ==length_m_));

    if (!compressOnSerialize_)
    {
        if (useShallowCopy_)
        {
            ar >> boost::serialization::make_binary_object(static_cast<void *>(cells_),numRows_*numCols_*sizeof(TT));
        }
        else
        {
        for (int i=0; i< numRows_ * numCols_; i++)
            ar >> cells_[i];
        }
    }
    else
    {
        unsigned long compressedDataSize;
        unsigned long uncompressedMapSize = sizeof(TT)*numRows_*numCols_;
        int zerror;
        ar >> compressedDataSize;
        if (compressedDataSize > uncompressBufferSize)
        {
            if (uncompressBuffer)
                delete[] uncompressBuffer;
            // we don't bother allocating just to the size needed, we allocate to the maxium data size
            // for this map size- the assumption is we will probably be sending several maps this size
            // and we don't want to reallocate, just because the first map we get is efficiently compressed
            uncompressBuffer = new unsigned char[compressBound(uncompressedMapSize)];
        }
        ar >> boost::serialization::make_binary_object(static_cast<void *>(uncompressBuffer),
                                                       compressedDataSize);
        zerror =uncompress(reinterpret_cast<unsigned char*>(cells_),&uncompressedMapSize,
                           uncompressBuffer,compressedDataSize);
        if (zerror!=Z_OK)
            std::cerr << "uncompression error!" << std::endl;

    }

}
*/




#endif //ifndef _MAP_DEF2_H_
