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

/** \file CyclicEventStatistics.h
  * \brief Header file providing the CyclicEventStatistics class interface
  */

#ifndef ROSCPP_NODEWRAP_CYCLIC_EVENT_STATISTICS_H
#define ROSCPP_NODEWRAP_CYCLIC_EVENT_STATISTICS_H

#include <roscpp_nodewrap/statistics/Statistics.h>

namespace nodewrap {
  /** \brief ROS cyclic event statistics
    * 
    * This class provides the templated basis of all statistics for
    * evaluating the timing of a cyclic event.
    */
  template <typename T> class CyclicEventStatistics :
    public Statistics<T> {
  public:
    /** \brief Declaration of the cyclic event statistics estimates type
      */
    class Estimates :
      public Statistics<T>::Estimates {
    public:
      /** \brief Default constructor
        */
      Estimates();
      
      /** \brief Copy constructor
        * 
        * \param[in] src The source cyclic event statistics estimates which
        *   are being copied to these cyclic event statistics estimates.
        */
      Estimates(const Estimates& src);
          
      /** \brief Convert these cyclic event statistics estimates to a ROS
        *    message
        * 
        * \param[in,out] message The message to convert these estimates to.
        */
      template <typename M> void toMessage(M& message) const;
      
      /** \brief Time of the last event considered in the estimates
        */
      ros::Time timeOfLastEvent;
    };
    
    /** \brief Default constructor
      * 
      * \param[in] rollingWindowSize The size of the window for computing
      *   the rolling estimates.
      * \param[in] nameOfVariates The name of the variates.
      * \param[in] unitOfVariates The unit of the variates.
      */
    CyclicEventStatistics(size_t rollingWindowSize = 0, const std::string&
      nameOfVariates = std::string(), const std::string& unitOfVariates = 
      std::string());
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source cyclic event statistics which are being
      *   copied to these cyclic event statistics.
      */
    CyclicEventStatistics(const CyclicEventStatistics<T>& src);
    
    /** \brief Destructor
      */
    ~CyclicEventStatistics();
    
    /** \brief Retrieve the time of the last event
      *
      * \return The time of the last event.
      */
    const ros::Time& getTimeOfLastEvent() const;
    
    /** \brief Retrieve the time since the last event
      *
      * \param[in] now The assumed current time used to compute the time
      *   since the last event.
      * 
      * \return The time since the last event.
      */
    ros::Duration getTimeSinceLastEvent(const ros::Time&
        now = ros::Time::now()) const;    
    
    /** \brief Update these cyclic event statistics with the time of
      *   occurrence of an event.
      * 
      * \param[in] timeOfEvent The time of the event.
      */
    void event(const ros::Time& timeOfEvent = ros::Time::now());
    
    /** \brief Extract the estimates of these cyclic event statistics
      *
      * \param[in,out] estimates The extracted current estimates of
      *   these statistics.
      */
    void extract(Estimates& estimates) const;
    
    /** \brief Clear these cyclic event statistics
      */
    void clear();
    
  protected:
    /** \brief The time of the last event
      */
    ros::Time timeOfLastEvent;
  };
};

#include <roscpp_nodewrap/statistics/CyclicEventStatistics.tpp>

#endif
