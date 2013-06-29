/**
 * @file    ScrollingMapUpdateLog.h
 * @author  Christopher Baker
 * @date    07/17/2008
 *
 * @attention Copyright (c) 2008
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#ifndef _SCROLLINGMAPUPDATELOG_H_
#define _SCROLLINGMAPUPDATELOG_H__

#include <task/Task.h>
#include <task/LogChannel.h>

/**\brief Read and Regurgitate older ScrollingByteMap logs.
 *
 * The ScrollingByteMap is responsible for some of the largest log files in the bunch,
 * and the version 1 serialization format had a bunch of redundant information that could
 * be discarded, saving something like 92% of disk space.
 *
 * Beyond that, they are not compressed by default to save CPU, but they generally compress
 * quite well (FusedStaticMaps, for instance, typically compress by 97% or more).  Moreover,
 * in playback situations, decompression is very fast, and the time spent is usually recovered
 * and more by having to move around 97% less data as message buffers.  This, combined with the
 * the a 5-minute log requiring 23MB compressed, as opposed to 844MB raw, makes compression an
 * attractive post-processing option
 */
class ScrollingMapUpdateLog: public task::Task
{
  public:
    ScrollingMapUpdateLog( const std::string& taskName );
    virtual ~ScrollingMapUpdateLog( );

    virtual bool initialize( void );
    virtual bool executive( void );
    virtual void cleanup( void );

  protected:
    task::LogChannel *input_;
    task::LogChannel *output_;
    bool enableCompression_;
};

#endif
