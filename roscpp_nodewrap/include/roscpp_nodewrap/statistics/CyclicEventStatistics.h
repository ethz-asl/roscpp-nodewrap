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

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS cyclic event statistics
    * 
    * This class provides the basis of all statistics for a cyclic event.
    */
  class CyclicEventStatistics {
  public:
    /** \brief Default constructor
      */
    CyclicEventStatistics();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source cyclic event statistics which are being
      *   copied to these cyclic event statistics.
      */
    CyclicEventStatistics(const CyclicEventStatistics& src);
    
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
    inline ros::Duration getTimeSinceLastEvent(const ros::Time&
        now = ros::Time::now()) const {
      if (!timeOfLastEvent.isZero())
        return now-timeOfLastEvent;
      else
        return ros::Duration();
    };    
    
    /** \brief Signal to these cyclic event statistics that an event
      *   has occurred.
      * 
      * \param[in] timeOfEvent The time of the event.
      */
    inline void event(const ros::Time& timeOfEvent = ros::Time::now()) {
      timeOfLastEvent = timeOfEvent;
    };
    
    /** \brief Clear these cyclic event statistics
      */
    void clear();
    
  protected:
    /** \brief Time of the last event of these cyclic event statistics
      */
    ros::Time timeOfLastEvent;
  };
};

#endif
