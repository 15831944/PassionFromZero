
#ifndef _SCROLLINGMAP_H_
#define _SCROLLINGMAP_H_


#include "../RecGeometry.h"
#include "Map.h"
#include <iostream>
#include <iterator>

//#include <boost/serialization/version.hpp>
//#include <boost/serialization/split_member.hpp>
//#include <boost/serialization/binary_object.hpp>
//#include <boost/serialization/vector.hpp>

//#include <zlib.h>

template <class TT>
class ScrollingMap;

template <class TT> void filterNth(unsigned int n, ScrollingMap<TT> & destMap, ScrollingMap<TT> & srcMap);

/**
 * @brief a new type of scrolling 2D map
 * @ingroup perceptionTypesGroup
 */
template <class TT>
class ScrollingMap
{
public:
    // a new constructor to signify that we're doing row-column scrolling
    ScrollingMap(double size_m, double cellSize_m, bool RC, TT voidedValue);

    // old constructors
    ScrollingMap(double size_m, double cellSize_m);
    ScrollingMap(const ScrollingMap& otherMap);
    ScrollingMap(void);

    virtual ~ScrollingMap(void);
    void operator=(const ScrollingMap& other);

    void setUseShallowCopy(void) {useShallowCopy_=true;};
    void setCompressOnSerialize(void) {compressOnSerialize_ = true;};

    TT & get(const RecPoint3D & globalPt, bool &isValid);
    TT & get(const RecPoint2D & globalPt, bool &isValid);
    TT & getByRowCol(int row, int col, bool &isValid);

    TT & set(const RecPoint3D & globalPt, const TT & value);
    TT & set(const RecPoint2D & globalPt, const TT & value);
    TT & setByRowCol(int row, int col, const TT & value);

    double getCellSize(void);
    double getSize(void);
    int getDimension(void);
    void centerAt(const RecPoint2D & globalPt);
    inline RecAxisAlignedBox2D getBounds() const {return boundary_;};

    template<class T>inline T square(const T &x) { return x*x;}
    double *edt1D(double *f, int n, double maxSquaredDistance);
    void edt2D(double *map,int width, int height, double maxSquaredDistance);
    void edtBinaryMap(double bandwidth, double maxSquaredDistance);
    //    inline TT operator*() const {return curCell_->value;};

protected:
    /**
     * @brief the container type for data in our map
     */
    class Cell
    {
    public:
        Cell(void);
        Cell(const Cell & otherCell);
        long row;
        long column;
        TT value;
        void operator=(const Cell & otherCell);
    /*
    private:
        friend class boost::serialization::access;
        template <class Archive>
        void serialize(Archive &ar, unsigned int version)
            {
                ar & row & column & value;
            };
    */
    };

  public:
    inline void convertGlobalPointToIndices(const RecPoint2D & globalPt, long &row,  long &col) {
        pointToRowCol(globalPt, row, col);}

    //    void convertGlobalPointToIndices(const RecPoint3D & globalPt, long &row,  long &col) const;

    void getCellCenter(const RecPoint2D & globalPt, RecPoint2D & centerPoint);
    void getCellCenter(const RecPoint3D & globalPt, RecPoint3D & centerPoint);

    class LineIterator:
        public std::iterator<std::forward_iterator_tag, TT>
    {
      public:
        LineIterator & operator++(); ///<prefix increment, steps along line
        LineIterator operator++(int);///<postfix increment, steps along line
        LineIterator & operator=(LineIterator const &other); ///<assignment operator
        bool operator==(LineIterator const &other) const;///< equality operator
        bool operator!=(LineIterator const &other) const {return !((*this)==other);} ///<inequality operator
        LineIterator(const LineIterator &other); ///< copy constructor
        LineIterator(void); ///< creates a end of line iterator
        inline int getRow() const {return row_;};
        inline int getCol() const {return col_;};
        // NOTE:  The * operator no longer returns a reference, just the value. See LineInterator::set below
        inline std::pair<TT,bool> operator*() const
            { return ( std::make_pair( curCell_->value, row_==curCell_->row && col_==curCell_->column ) ); }
        // NOTE:  The * operator no longer returns a pointer, just the value. See LineInterator::set below
        //inline TT operator->() const {return curCell_->value;};  NOW ITS DEAD
        bool isValid(void); ///< reports true if the current cell is valid
        // This is the new method for setting the value in a cell using a line iterator.  This method not only sets the
        // the value, but it also directs the cell to be at the location of the line iterator.
        inline void set( const TT& val ) { map_->setByRowCol(row_,col_, val); };

        inline RecPoint2D globalCoord() { return map_->rowColToGlobalPoint( row_, col_ ); }

        void clearConnectedFromHere(const TT & clearValue, unsigned int maxDepth = 0xFFFFFFFF);

      private:
        friend class ScrollingMap;
        LineIterator(const IndexRange &range, ScrollingMap<TT> & map);
        Cell *curCell_; ///< the current cell we're point to
        IndexRange range_; ///< the iterator will travel from rs,cs to re,ce
        bool offEnd_; ///< indicates the iterator has run its course
        bool almostOffEnd_; ///< a fix to make the line drawing inclusive of end points
        int row_,col_; ///< row and column index
        long numRows_; ///< number of rows in the source map
        long numCols_; ///< number of columns in the source map
        int dr_,dc_; ///< step directions for the iterator
        int fraction_; ///< ratio of row/column motion required
        int stepRow_, stepCol_; ///< the step direction for moving through the map
        ScrollingMap<TT> * map_; ///< a pointer to the refernce map
    };
    LineIterator beginLine(const RecPoint2D& start, const RecPoint2D& end);
    LineIterator endLine(void );

    class iterator: public std::iterator<std::forward_iterator_tag, TT>
    {
    public:
        iterator & operator++(); ///< pre increment operator
        iterator operator++(int); ///< post increment operator
        iterator & operator=(iterator const &other); ///< assignment operator
        bool operator==(iterator const &other) const;
        bool operator!=(iterator const &other) const;

        inline std::pair<TT,bool> operator*() const;

        iterator(const iterator & other);
        iterator(void); ///< this constructors sole purpose is to set the off end
        bool isValid(void) const;
        inline void set( const TT& val );
        inline int getRow() const {return row_;};
        inline int getCol() const {return col_;};
        inline RecPoint2D getCenterPoint() const {return map_->rowColToGlobalPoint( row_, col_ );}

        inline std::pair<TT,bool> getNeighbor(int rOffset, int cOffset) const
            {
                int rIndex, cIndex;
                rIndex = row_ + rOffset;
                cIndex = col_ + cOffset;
                Cell &cell = map_->index( rIndex, cIndex);
                return ( std::make_pair( cell.value, cell.row==rIndex && cell.column==cIndex ) );
            }

        inline void setNeighbor( int rOffset, int cOffset, const TT& val )
            {
                map_->setByRowCol(row_ + rOffset, col_ + cOffset);
            }

    protected:
        friend class ScrollingMap;
        iterator(const IndexRange & range, ScrollingMap<TT> & map);

        IndexRange range_; ///< the range over which the iterator is moving
        ScrollingMap<TT> * map_; ///< a pointer to the refernce map
        bool offEnd_;///< means the iterator is off the end of the range
        int row_,col_; ///< current row and column
        Cell *cell_, *lastCellInRow_;

        Cell *computeLastCellInRow(const Cell * const cell_) const;
    };
    iterator begin();
    iterator begin(const RecPoint2D & min, const RecPoint2D & max);
    iterator begin(const RecAxisAlignedBox2D & box);
    iterator end(void);

    friend void filterNth<>(unsigned int n, ScrollingMap<TT> & destMap, ScrollingMap<TT> & srcMap);
    void clear(void);

protected:
    double cellSize_m_; ///<the width/length of a cell in the map
    double invCellSize_; ///<1/cellSize_m_;
    double size_m_; ///< the width/length of the map
    int dimension_; ///< the number of rows/columns in the map
    RecAxisAlignedBox2D boundary_;

    Cell * cells_;
    ptime timeStamp_;
    bool useShallowCopy_;
    bool compressOnSerialize_;

    bool RC_;
    TT voidedValue_;
    void colShift( long oldCol, const long newCol );
    void rowShift( long oldRow, const long newRow );

    void mapSetup(double size_m, double cellSize_m,
                  bool useShallowCopy=false, bool compressOnSerialize=false,
                  bool needAllocation= true);

    Cell & index(long row, long col);
    void pointToRowCol(const RecPoint2D & globalPt, long &row,long &col) const;
    void pointToRowCol(const RecPoint3D & globalPt, long &row,long &col) const;
    int globalRowColToIndex(long row,long col) const;
    RecPoint2D rowColToGlobalPoint( long row, long col ) const;
    long mod(long i, long range) const;
    void clearRecurser(const TT & clearValue, int row, int col,
                       unsigned int curDepth, unsigned int maxDepth);

//    friend class boost::serialization::access;

//    template <class Archive>
//    void load(Archive & ar, unsigned int version);

    unsigned char *uncompressBuffer_;
    unsigned long uncompressBufferSize_;

//    template <class Archive>
//    void save(Archive & ar, unsigned int version) const;

//    BOOST_SERIALIZATION_SPLIT_MEMBER()

};

typedef ScrollingMap<unsigned char> ScrollingByteMap;
typedef ScrollingMap<double> ScrollingDoubleMap;
typedef ScrollingMap<short> ScrollingShortMap;

#include "ScrollingMap.def.h"
#include "ScrollingMap.def2.h"

#endif //ifndef _SCROLLINGMAP_H_
