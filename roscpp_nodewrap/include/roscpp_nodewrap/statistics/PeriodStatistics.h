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

/** \file PeriodStatistics.h
  * \brief Header file providing the PeriodStatistics class interface
  */

#ifndef ROSCPP_NODEWRAP_PERIOD_STATISTICS_H
#define ROSCPP_NODEWRAP_PERIOD_STATISTICS_H

#include <ros/ros.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>

#include <roscpp_nodewrap/statistics/CyclicEventStatistics.h>

#include <roscpp_nodewrap_msgs/PeriodEstimates.h>

namespace nodewrap {
  /** \brief ROS period statistics
    * 
    * This class provides period statistics for a cyclic event.
    */
  class PeriodStatistics :
    public CyclicEventStatistics {
  public:
    /** \brief Declaration of the estimates type
      */
    class Estimates {
    public:
      /** \brief Default constructor
        */
      Estimates();
      
      /** \brief Copy constructor
        * 
        * \param[in] src The source period statistics estimates which
        *   are being copied to these period statistics estimates.
        */
      Estimates(const Estimates& src);
          
      /** \brief Convert these period statistics estimates to a ROS
        *   message
        * 
        * \param[in,out] msg The message to convert these estimates to.
        */
      void toMessage(PeriodEstimates& msg) const;
      
      /** \brief Time of the last event considered in the estimates
        */
      ros::Time timeOfLastEvent;
      
      /** \brief The number of samples accumulated to compute the non-rolling
        *   estimates
        */
      size_t numSamples;
      
      /** \brief The overall minimum period
        */
      ros::Duration min;
      
      /** \brief The overall maximum period
        */
      ros::Duration max;
      
      /** \brief The number of samples accumulated to compute the rolling
        *   estimates
        */
      size_t numRollingSamples;
      
      /** \brief The rolling mean period
        */
      ros::Duration rollingMean;
      
      /** \brief The rolling period variance
        */
      ros::Duration rollingVariance;
    };
    
    /** \brief Default constructor
      *
      * \param[in] rollingWindowSize The window size for the rolling
      *   estimates of the statistics.
      */
    PeriodStatistics(size_t rollingWindowSize = 100);
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source period statistics which are being
      *   copied to these period statistics.
      */
    PeriodStatistics(const PeriodStatistics& src);
    
    /** \brief Destructor
      */
    ~PeriodStatistics();
    
    /** \brief Set these statistics' rolling window size
      *
      * \param[in] rollingWindowSize The window size for the rolling
      *   estimates of the statistics. 
      * 
      * \note Calling this method will lead to the statistics being
      *   cleared.
      */
    void setRollingWindowSize(size_t rollingWindowSize);
    
    /** \brief Retrieve these statistics' rolling window size
      *
      * \return The window size for the rolling estimates of these
      *   statistics. 
      */
    size_t getRollingWindowSize() const;
    
    /** \brief Access these statistics' number of non-rolling samples
      *
      * \return The number of samples accumulated to compute the non-rolling
      *   estimates of these statistics.
      */
    size_t getNumSamples() const;
    
    /** \brief Access these statistics' number of rolling samples
      *
      * \return The number of samples accumulated to compute the rolling
      *   estimates of these statistics, generally less or equal than the
      *   defined window size.
      */
    size_t getNumRollingSamples() const;
    
    /** \brief Access these statistics' minimum period
      *
      * \return The overall minimum period of the statistics.
      */
    ros::Duration getMin() const;
    
    /** \brief Access these statistics' maximum period
      *
      * \return The overall maximum period of the statistics.
      */
    ros::Duration getMax() const;
    
    /** \brief Access these statistics' rolling mean period
      *
      * \return The rolling mean period of the statistics within the
      *   specified window.
      */
    ros::Duration getRollingMean() const;
    
    /** \brief Access these statistics' rolling period variance
      *
      * \return The period variance of the statistics within the
      *   specified window.
      */
    ros::Duration getRollingVariance() const;
    
    /** \brief Retrieve the current estimates of these statistics
      *
      * \return The current estimates of these statistics
      */
    Estimates getEstimates() const;
    
    /** \brief Signal to these period statistics that an event has
     *    occurred.
      * 
      * \param[in] timeOfEvent The time of the event.
      * 
      * \see CyclicEventStatistics::event
      */
    inline void event(const ros::Time& timeOfEvent = ros::Time::now()) {
      if (rollingWindowSize) {
        double secondsSinceLastEvent = getTimeSinceLastEvent(
          timeOfEvent).toSec();
        
        if (secondsSinceLastEvent > 0.0)
          accumulator(secondsSinceLastEvent);
      }
      
      CyclicEventStatistics::event(timeOfEvent);
    };
    
    /** \brief Clear these period statistics
      */
    void clear();
    
  protected:
    /** \brief Forward declaration of the accumulator type
      */ 
    typedef boost::accumulators::accumulator_set<double,
      boost::accumulators::stats<
        boost::accumulators::tag::min,
        boost::accumulators::tag::max,
        boost::accumulators::tag::rolling_mean,
        boost::accumulators::tag::variance>
      > Accumulator;
    
    /** \brief Accumulator for these period statistics
      */
    Accumulator accumulator;
    
    /** \brief Rolling window size of these period statistics
      */
    size_t rollingWindowSize;
  };
};

#endif
