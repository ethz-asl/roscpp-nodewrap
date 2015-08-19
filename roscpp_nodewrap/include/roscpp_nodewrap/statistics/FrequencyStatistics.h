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

#include <roscpp_nodewrap/statistics/CyclicEventStatistics.h>

#include <roscpp_nodewrap_msgs/FrequencyEstimates.h>

namespace nodewrap {
  /** \brief ROS frequency statistics
    * 
    * This class provides frequency statistics for a cyclic event.
    */
  class FrequencyStatistics :
    public CyclicEventStatistics<double> {
  public:
    /** \brief Default constructor
      *
      * \param[in] windowSize The window size for the rolling estimates
      *   of the statistics.
      * \param[in] nameOfVariates The name of the variates.
      * \param[in] unitOfVariates The unit of the variates.
      */
    FrequencyStatistics(size_t rollingWindowSize = 0, const std::string&
      nameOfVariates = "frequency", const std::string& unitOfVariates = "Hz");
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source frequency statistics which are being
      *   copied to these frequency statistics.
      */
    FrequencyStatistics(const FrequencyStatistics& src);
    
    /** \brief Destructor
      */
    ~FrequencyStatistics();
    
    /** \brief Update these frequency statistics with the time of
      *   occurrence of an event
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
      
      CyclicEventStatistics<double>::event(timeOfEvent);
    };
  };
};

#endif
