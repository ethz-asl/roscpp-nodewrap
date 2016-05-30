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

/** \file Statistics.h
  * \brief Header file providing the Statistics class interface
  */

#ifndef ROSCPP_NODEWRAP_STATISTICS_H
#define ROSCPP_NODEWRAP_STATISTICS_H

#include <ros/ros.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/rolling_variance.hpp>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS statistics
    * 
    * This class provides the templated basis of all statistics.
    */
  template <typename T> class Statistics {
  public:
    /** \brief Forward declaration of the variate type
      */
    typedef T Variate;
    
    /** \brief Declaration of the statistics estimates type
      */
    class Estimates {
    public:
      /** \brief Default constructor
        */
      Estimates();
      
      /** \brief Copy constructor
        * 
        * \param[in] src The source statistics estimates which are being
        *   copied to these statistics estimates.
        */
      Estimates(const Estimates& src);
          
      /** \brief Convert these statistics estimates to a ROS message
        * 
        * \param[in,out] message The message to convert these estimates to.
        */
      template <typename M> void toMessage(M& message) const;
      
      /** \brief The number of samples accumulated to compute the non-rolling
        *   estimates
        */
      size_t numSamples;
      
      /** \brief The overall minimum sample
        */
      T min;
      
      /** \brief The overall maximum sample
        */
      T max;
      
      /** \brief The number of samples accumulated to compute the rolling
        *   estimates
        */
      size_t numRollingSamples;
      
      /** \brief The rolling mean estimate
        */
      T rollingMean;
      
      /** \brief The rolling variance estimate
        */
      T rollingVariance;
    };
    
    /** \brief Default constructor
      * 
      * \param[in] rollingWindowSize The size of the window for computing
      *   the rolling estimates.
      * \param[in] nameOfVariates The name of the variates.
      * \param[in] unitOfVariates The unit of the variates.
      */
    Statistics(size_t rollingWindowSize = 0, const std::string&
      nameOfVariates = std::string(), const std::string& unitOfVariates =
      std::string());
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source statistics which are being copied to
      *   these statistics.
      */
    Statistics(const Statistics<T>& src);
    
    /** \brief Destructor
      */
    virtual ~Statistics();
    
    /** \brief Set the name of the variates of these statistics
      *
      * \param[in] nameOfVariates The name of the variates. 
      */
    void getNameOfVariates(const std::string& nameOfVariates);
    
    /** \brief Retrieve the name of the variates of these statistics
      *
      * \return The name of the variates. 
      */
    const std::string& getNameOfVariates() const;
    
    /** \brief Set the unit of the variates of these statistics
      *
      * \param[in] unitOfVariates The unit of the variates. 
      */
    void getUnitOfVariates(const std::string& unitOfVariates);
    
    /** \brief Retrieve the unit of the variates of these statistics
      *
      * \return The unit of the variates. 
      */
    const std::string& getUnitOfVariates() const;
    
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
    
    /** \brief Access these statistics' minimum sample
      *
      * \return The overall minimum sample of the statistics.
      */
    T getMin() const;
    
    /** \brief Access these statistics' maximum sample
      *
      * \return The overall maximum sample of the statistics.
      */
    T getMax() const;
    
    /** \brief Access these statistics' rolling mean estimate
      *
      * \return The rolling mean estimate of the statistics within the
      *   specified window.
      */
    T getRollingMean() const;
    
    /** \brief Access these statistics' rolling variance estimate
      *
      * \return The rolling variance estimate of the statistics within
      *   the specified window.
      */
    T getRollingVariance() const;
    
    /** \brief Update these statistics
      * 
      * \param[in] sample The sample used to update the statistics.
      */
    virtual void update(const T& sample);
    
    /** \brief Extract the estimates of these statistics
      *
      * \param[in,out] estimates The extracted current estimates of
      *   these statistics.
      */
    void extract(Estimates& estimates) const;
    
    /** \brief Clear these statistics
      */
    virtual void clear();
    
  protected:
    /** \brief Forward declaration of the accumulator type
      */ 
    typedef boost::accumulators::accumulator_set<T,
      boost::accumulators::stats<
        boost::accumulators::tag::count,
        boost::accumulators::tag::min,
        boost::accumulators::tag::max,
        boost::accumulators::tag::rolling_mean,
        boost::accumulators::tag::rolling_variance>
      > Accumulator;
    
    /** \brief The accumulator for these statistics
      */
    Accumulator accumulator;
    
    /** \brief The rolling window size of these statistics
      */
    size_t rollingWindowSize;
    
    /** \brief The name of the variates of these statistics
      */
    std::string nameOfVariates;
    
    /** \brief The unit of the variates of these statistics
      */
    std::string unitOfVariates;
  };
};

#include <roscpp_nodewrap/statistics/Statistics.tpp>

#endif
