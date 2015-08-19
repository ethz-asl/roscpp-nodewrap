/******************************************************************************
 * Copyright (C) 2014 by Ralf Kaestner                                        *
 * ralf.kaestner@gmail.com                                                    *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the               *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/

/** \file LatencyStatistics.h
  * \brief Header file providing the LatencyStatistics class interface
  */

#ifndef ROSCPP_NODEWRAP_LATENCY_STATISTICS_H
#define ROSCPP_NODEWRAP_LATENCY_STATISTICS_H

#include <utility>

#include <roscpp_nodewrap/statistics/Statistics.h>

#include <roscpp_nodewrap_msgs/LatencyEstimates.h>

namespace nodewrap {
  /** \brief ROS latency statistics
    * 
    * This class provides latency statistics for a cyclic event.
    */
  class LatencyStatistics :
    public Statistics<double> {
  public:
    /** \brief Declaration of the latency statistics estimates type
      */
    class Estimates :
      public Statistics<double>::Estimates {
    public:
      /** \brief Default constructor
        */
      Estimates();
      
      /** \brief Copy constructor
        * 
        * \param[in] src The source latency statistics estimates which
        *   are being copied to these latency statistics estimates.
        */
      Estimates(const Estimates& src);
          
      /** \brief Convert these latency statistics estimates to a ROS
        *    message
        * 
        * \param[in,out] message The message to convert these estimates to.
        */
      template <typename M> void toMessage(M& message) const;
      
      /** \brief Times of the last events considered in the estimates
        */
      std::pair<ros::Time, ros::Time> timesOfLastEvents;
    };
    
    /** \brief Default constructor
      *
      * \param[in] windowSize The window size for the rolling estimates
      *   of the statistics.
      * \param[in] nameOfVariates The name of the variates.
      * \param[in] unitOfVariates The unit of the variates.
      */
    LatencyStatistics(size_t rollingWindowSize = 0, const std::string&
      nameOfVariates = "latency", const std::string& unitOfVariates = "s");
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source latency statistics which are being
      *   copied to these latency statistics.
      */
    LatencyStatistics(const LatencyStatistics& src);
    
    /** \brief Destructor
      */
    ~LatencyStatistics();
    
    /** \brief Retrieve the times of the last events
      *
      * \return The times of the last events.
      */
    const std::pair<ros::Time, ros::Time>& getTimesOfLastEvents() const;
    
    /** \brief Update these latency statistics with the time of occurrence
      *   of two events
      * 
      * \param[in] timeOfFirstEvent The time of occurrence of the first
      *   event.
      * \param[in] timeOfSecondEvent The time of occurrence of the second
      *   event.
      */
    inline void events(const ros::Time& timeOfFirstEvent, const ros::Time&
        timeOfSecondEvent = ros::Time::now()) {
      if (rollingWindowSize) {
        double secondsBetweenEvents =
          (timeOfSecondEvent-timeOfFirstEvent).toSec();
       
        accumulator(secondsBetweenEvents);
      }
      
      timesOfLastEvents.first = timeOfFirstEvent;
      timesOfLastEvents.second = timeOfSecondEvent;
    };
    
    /** \brief Extract the estimates of these latency statistics
      *
      * \param[in,out] estimates The extracted current estimates of
      *   these statistics.
      */
    void extract(Estimates& estimates) const;
    
    /** \brief Clear these latency statistics
      */
    void clear();
    
  protected:
    /** \brief The time of the last event
      */
    std::pair<ros::Time, ros::Time> timesOfLastEvents;
  };
};

#include <roscpp_nodewrap/statistics/LatencyStatistics.tpp>

#endif
