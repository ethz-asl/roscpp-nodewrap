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

#include <roscpp_nodewrap/diagnostics/DiagnosticTaskOptions.h>

namespace nodewrap {
  /** \brief ROS frequency task options
    * 
    * This class encapsulates all options available for creating a
    * diagnostic task to monitor the frequency of a cyclic event.
    */
  class FrequencyTaskOptions :
    public DiagnosticTaskOptions {
  public:
    /** \brief Default constructor
      */
    FrequencyTaskOptions();
    
    /** \brief The window of the frequency task's statistics
      *
      * The window size of the statistics is determined by the defined
      * window duration and the expected mean frequency. 
      */
    ros::Duration window;
    
    /** \brief The expected frequency to be diagnosed in [Hz]
      */
    double expected;
    
    /** \brief The warning tolerance of the diagnosed mean frequency
      * 
      * The tolerance bounds are represented by a factor in [0, 1] which
      * is symmetrically interpreted as the ratio of the expected frequency
      * and the diagnosed mean frequency (and vice versa).
      */
    double warnMeanTolerance;
    
    /** \brief The error tolerance of the diagnosed mean frequency
      * 
      * The tolerance bounds are represented by a factor in [0, 1] which
      * is symmetrically interpreted as the ratio of the expected frequency
      * and the diagnosed mean frequency (and vice versa).
      */
    double errorMeanTolerance;
    
    /** \brief The warning tolerance of the diagnosed frequency standard
      *   deviation
      * 
      * The tolerance bounds are represented by a factor in [0, 1] which
      * is symmetrically interpreted as the ratio of the expected frequency
      * and the frequencies obtained by symmetrically offsetting the
      * diagnosed mean frequency by the diagnosed frequency variance
      * (and vice versa).
      */
    double warnStandardDeviationTolerance;
    
    /** \brief The error tolerance of the diagnosed frequency standard
      *   deviation
      * 
      * The tolerance bounds are represented by a factor in [0, 1] which
      * is symmetrically interpreted as the ratio of the expected frequency
      * and the frequencies obtained by symmetrically offsetting the
      * diagnosed mean frequency by the diagnosed frequency variance
      * (and vice versa).
      */
    double errorStandardDeviationTolerance;
  };
};

#endif
