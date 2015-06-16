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

/** \file FrequencyTaskOptions.h
  * \brief Header file providing the FrequencyTaskOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_FREQUENCY_TASK_OPTIONS_H
#define ROSCPP_NODEWRAP_FREQUENCY_TASK_OPTIONS_H

#include <roscpp_nodewrap/statistics/FrequencyStatistics.h>

#include <roscpp_nodewrap/diagnostics/StatisticsTaskOptions.h>

namespace nodewrap {
  /** \brief ROS frequency task options
    * 
    * This class encapsulates all options available for creating a
    * diagnostic task to monitor the frequency of an event.
    */
  class FrequencyTaskOptions :
    public StatisticsTaskOptions<FrequencyStatistics> {
  public:
    /** \brief Default constructor
      */
    FrequencyTaskOptions();
    
    /** \brief The window duration of the task's statistics
      *
      * The window duration and the expected frequency determine the
      * maximum number of rolling samples of the task's statistics.
      */
    ros::Duration windowDuration;
  };
};

#endif
