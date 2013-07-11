
#ifndef _TIMESTAMP_H_
#define _TIMESTAMP_H_
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/gregorian/greg_serialize.hpp>
#include <boost/date_time/posix_time/time_serialize.hpp>

/**
 * we use the boost ptime as our time inteface.  It is very powerful.  Consult
 http://www.boost.org/doc/html/date_time/posix_time.html as a good reference.
 */
using boost::posix_time::ptime;
using boost::posix_time::time_duration;
using boost::posix_time::hours;
using boost::posix_time::minutes;
using boost::posix_time::seconds;
using boost::posix_time::milliseconds;
using boost::posix_time::microseconds;


/**
 * @brief calculate the time elapsed between t1 and t2.  i.e. if t2 occured after t1, the result is positive.
 *
 * @return returns the delta in time between t1 and t2, in fractional seconds with the calculation performed to microsecond resolution.
 */
inline long double elapsedTimeInSeconds(const ptime & t1, const ptime &t2) 
{
    time_duration tmp = t2-t1;
    long numSeconds = tmp.total_seconds();
    time_duration remainder = tmp - seconds(numSeconds);

    return (long double)( numSeconds + remainder.total_microseconds()/(1000000.0));
}

inline long double convertToSeconds(const ptime & t1)
{
    const static ptime epoch(boost::gregorian::date(1970,1,1));
    return elapsedTimeInSeconds(epoch,t1);
}

inline ptime convertToptime(long double t)
{
    const static ptime epoch(boost::gregorian::date(1970,1,1));
    long sec = static_cast<long>(t);
    long double remainder= (t-sec);
    long usec = static_cast<long>((remainder)*1000000);

    return (epoch + seconds(sec)+microseconds(usec));
}

/**
 * @brief helper function to allow easy to use duration generation
 */
inline time_duration secondsToDuration(double secs)
{
    time_duration tmp = seconds(static_cast<long>(secs));
    long usecs = static_cast<long>((secs-static_cast<long>(secs))*1000000 + 0.5);
    return tmp + microseconds(usecs);
}


/**
 * @brief this function converts a given time duration to a double representation of the seconds, 
with microsecond accuracy
 */
inline double toSeconds(const time_duration &td)
{
    long secs = td.total_seconds();
    time_duration fracSeconds = td-seconds(secs);
    return static_cast<double>(secs + fracSeconds.total_microseconds()/1000000.0);

}

#endif //ifndef _TIMESTAMP_H_
