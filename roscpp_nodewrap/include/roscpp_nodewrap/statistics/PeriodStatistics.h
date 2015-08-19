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

#include <roscpp_nodewrap/statistics/CyclicEventStatistics.h>

#include <roscpp_nodewrap_msgs/PeriodEstimates.h>

namespace nodewrap {
  /** \brief ROS period statistics
    * 
    * This class provides period statistics for a cyclic event.
    */
  class PeriodStatistics :
    public CyclicEventStatistics<double> {
  public:
    /** \brief Default constructor
      *
      * \param[in] rollingWindowSize The window size for the rolling
      *   estimates of the statistics.
      * \param[in] nameOfVariates The name of the variates.
      * \param[in] unitOfVariates The unit of the variates.
      */
    PeriodStatistics(size_t rollingWindowSize = 0, const std::string&
      nameOfVariates = "period", const std::string& unitOfVariates = "s");
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source period statistics which are being
      *   copied to these period statistics.
      */
    PeriodStatistics(const PeriodStatistics& src);
    
    /** \brief Destructor
      */
    ~PeriodStatistics();
    
    /** \brief Update these period statistics with the time of occurrence
      *   of an event
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
      
      CyclicEventStatistics<double>::event(timeOfEvent);
    };
  };
};

#endif
