/**
 * @file    ScrollingMapUpdateLog.cc
 * @author  Christopher Baker
 * @date    07/17/2008
 *
 * @attention Copyright (c) 2008
 * @attention Carnegie Mellon University
 * @attention All rights reserved.
 */
#include "ScrollingMapUpdateLog.h"
#include "ScrollingMap.h"
#include <task/ChannelFactory.h>
#include <task/SerializationUtilities.h>

using namespace std;
using namespace task;

Task *Task::get(void)
{
    static ScrollingMapUpdateLog task(string("ScrollingMapUpdateLog"));
    return &task;
}

ScrollingMapUpdateLog::ScrollingMapUpdateLog(const string& taskName)
    : Task(taskName),
      input_(NULL),
      output_(NULL),
      enableCompression_(false)
{
}

ScrollingMapUpdateLog::~ScrollingMapUpdateLog()
{
}

bool ScrollingMapUpdateLog::initialize(void)
{
    logger_.log_info("initialize");

    // Open input_ channel
    {
        ConfigSection channelConfig;
        const ConfigSection::GetStatus status = taskParams_.get(string("channelToRead"), channelConfig);

        if (status == ConfigSection::GS_OK)
        {
            Channel *inputChannel = ChannelFactory::createChannel(channelConfig, task::Read);

            if (inputChannel == NULL)
            {
                logger_.log_fatal("channelToExcerptFrom failed to be created");

                return false;
            }
            else if ((input_ = dynamic_cast<LogChannel *>(inputChannel)) == NULL)
            {
                logger_.log_fatal("channelToExcerptFrom not a LogChannel");

                ChannelFactory::releaseChannel(inputChannel);

                return false;
            }
            else if (input_->connect() != task::CS_Success)
            {
                logger_.log_fatal("channelToExcerptFrom failed to connect");

                ChannelFactory::releaseChannel(inputChannel);
                input_ = NULL;

                return false;
            }
        } 
        else
        {
            logger_.log_fatal("Invalid channelToExcerptFrom specified");

            return false;
        }
    }

    {
        ConfigSection channelConfig;
        const ConfigSection::GetStatus status = taskParams_.get(string("channelToWrite"), channelConfig);

        if (status == ConfigSection::GS_OK) 
        {
            Channel *outputChannel = ChannelFactory::createChannel(channelConfig, task::Write);

            if (outputChannel == NULL)
            {
                logger_.log_fatal("channelToExcerptTo failed to be created");

                return false;
            }
            else if ((output_ = dynamic_cast<LogChannel *>(outputChannel)) == NULL)
            {
                logger_.log_fatal("channelToExcerptTo not a LogChannel");

                ChannelFactory::releaseChannel(outputChannel);

                return false;
            }
            else if (output_->connect() != task::CS_Success)
            {
                logger_.log_fatal("channelToExcerptTo failed to connect");

                ChannelFactory::releaseChannel(outputChannel);

                return false;
            }
        } 
        else
        {
            logger_.log_fatal("Invalid channelToExcerptFrom specified");

            return false;
        }
    }

    {
        const ConfigSection::GetStatus status = taskParams_.get(string("enableCompression"), enableCompression_);

        if (status != ConfigSection::GS_OK)
        {
            logger_.log_fatal("Invalid enableCompression specified");

            return false;
        }
    }

    return true;
}

bool ScrollingMapUpdateLog::executive(void)
{
    ChannelStatus readStatus = CS_NewData;
    ChannelStatus writeStatus = CS_Success;
    boost::posix_time::ptime begin,end,startConversion=getSystemTime();
    input_->getLogExtents(begin,end);

    const double totalLogTime_s = 0.001 * (double)(end-begin).total_milliseconds();

    logger_.log_notice("Input log covers %.3lfs",totalLogTime_s);

    while ((readStatus == CS_NewData) && (writeStatus == CS_Success))
    {
        string msgBuf,outBuf;
        boost::posix_time::ptime timeOfReceipt,now=getSystemTime();

        readStatus = input_->read(msgBuf, timeOfReceipt);
        if (readStatus == CS_NewData)
        {
            ScrollingByteMap bb;
            restoreType(bb,msgBuf);
            if(enableCompression_)
            {
                bb.setCompressOnSerialize();
            }
            serializeType(outBuf,bb);
            writeStatus = output_->write(outBuf, timeOfReceipt);
        }

        const double currentLogTime_s = 0.001 * (double)(timeOfReceipt-begin).total_milliseconds() + 0.001;
        const double currentWorkTime_s = 0.001 * (double)(now-startConversion).total_milliseconds() + 0.001;
        const double conversionRatio = currentLogTime_s / currentWorkTime_s;
        const double ETA_s = (totalLogTime_s - currentLogTime_s) / conversionRatio;

        fprintf(stderr,"\r - Processing Log: at %.3lf / %.3lf ( ratio = %.2lf, ETA = %.0lfs )         \r",
                currentLogTime_s,totalLogTime_s,conversionRatio, ETA_s);
        fflush(stderr);
    }

    return false;
}

void ScrollingMapUpdateLog::cleanup( 
    void 
    )
{
    logger_.log_info("cleanup");

    if (output_)
    {
        Channel *tmp(output_);

        output_->disconnect();

        ChannelFactory::releaseChannel(tmp);
        output_ = NULL;
    }

    if (input_)
    {
        Channel *tmp(input_);

        input_->disconnect();

        ChannelFactory::releaseChannel(tmp);
        input_ = NULL;
    }

    return;
}

