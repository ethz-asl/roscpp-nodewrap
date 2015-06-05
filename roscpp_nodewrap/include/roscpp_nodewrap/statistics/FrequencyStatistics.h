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

/** \file FrequencyStatistics.h
  * \brief Header file providing the FrequencyStatistics class interface
  */

#ifndef ROSCPP_NODEWRAP_FREQUENCY_STATISTICS_H
#define ROSCPP_NODEWRAP_FREQUENCY_STATISTICS_H

#include <ros/ros.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/rolling_variance.hpp>

#include <roscpp_nodewrap/statistics/CyclicEventStatistics.h>

#include <roscpp_nodewrap_msgs/FrequencyEstimates.h>

namespace nodewrap {
  /** \brief ROS frequency statistics
    * 
    * This class provides frequency statistics for a cyclic event.
    */
  class FrequencyStatistics :
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
        * \param[in] src The source frequency statistics estimates which
        *   are being copied to these frequency statistics estimates.
        */
      Estimates(const Estimates& src);
          
      /** \brief Convert these frequency statistics estimates to a ROS
        *   message
        * 
        * \param[in,out] msg The message to convert these estimates to.
        */
      void toMessage(FrequencyEstimates& msg) const;
      
      /** \brief Time of the last event considered in the estimates
        */
      ros::Time timeOfLastEvent;
    
      /** \brief The number of samples accumulated to compute the non-rolling
        *   estimates
        */
      size_t numSamples;
      
      /** \brief The overall minimum frequency
        */
      double min;
      
      /** \brief The overall maximum frequency
        */
      double max;
      
      /** \brief The number of samples accumulated to compute the rolling
        *   estimates
        */
      size_t numRollingSamples;
      
      /** \brief The rolling mean frequency
        */
      double rollingMean;
      
      /** \brief The rolling frequency variance
        */
      double rollingVariance;
    };
    
    /** \brief Default constructor
      *
      * \param[in] windowSize The window size for the rolling estimates
      *   of the statistics.
      */
    FrequencyStatistics(size_t rollingWindowSize = 100);
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source frequency statistics which are being
      *   copied to these frequency statistics.
      */
    FrequencyStatistics(const FrequencyStatistics& src);
    
    /** \brief Destructor
      */
    ~FrequencyStatistics();
    
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
    
    /** \brief Access these statistics' minimum frequency
      *
      * \return The overall minimum frequency of the statistics.
      */
    double getMin() const;
    
    /** \brief Access these statistics' maximum frequency
      *
      * \return The overall maximum frequency of the statistics.
      */
    double getMax() const;
    
    /** \brief Access these statistics' rolling mean frequency
      *
      * \return The rolling mean frequency of the statistics within the
      *   specified window.
      */
    double getRollingMean() const;
    
    /** \brief Access these statistics' rolling frequency variance
      *
      * \return The rolling frequency variance of the statistics within the
      *   specified window.
      */
    double getRollingVariance() const;
    
    /** \brief Retrieve the current estimates of these statistics
      *
      * \return The current estimates of these statistics
      */
    Estimates getEstimates() const;
    
    /** \brief Signal to these frequency statistics that an event has
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
          accumulator(1.0/secondsSinceLastEvent);
      }
      
      CyclicEventStatistics::event(timeOfEvent);
    };
    
    /** \brief Clear these frequency statistics
      */
    void clear();
    
  protected:
    /** \brief Forward declaration of the accumulator type
      */ 
    typedef boost::accumulators::accumulator_set<double,
      boost::accumulators::stats<
        boost::accumulators::tag::count,
        boost::accumulators::tag::min,
        boost::accumulators::tag::max,
        boost::accumulators::tag::rolling_mean,
        boost::accumulators::tag::rolling_variance>
      > Accumulator;
    
    /** \brief Accumulator for these frequency statistics
      */
    Accumulator accumulator;
    
    /** \brief Rolling window size of these frequency statistics
      */
    size_t rollingWindowSize;
  };
};

#endif
