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

/** \file StatisticsTaskOptions.h
  * \brief Header file providing the StatisticsTaskOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_STATISTICS_TASK_OPTIONS_H
#define ROSCPP_NODEWRAP_STATISTICS_TASK_OPTIONS_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTaskOptions.h>

namespace nodewrap {
  /** \brief ROS statistics task options
    * 
    * This class encapsulates all options available for creating a
    * diagnostic task to monitor the estimates of some statistics.
    */
  template <class S> class StatisticsTaskOptions :
    public DiagnosticTaskOptions {
  public:
    /** \brief Default constructor
      */
    StatisticsTaskOptions();
    
    /** \brief The expected value to be diagnosed
      */
    typename S::Variate expected;
    
    /** \brief The window size of the task's statistics
      *
      * The window size determines the maximum number of rolling samples
      * of the task's statistics.
      */
    size_t windowSize;
    
    /** \brief The warning tolerance of the diagnosed mean
      * 
      * The tolerance bounds are represented by a factor in [0, 1] which
      * is symmetrically interpreted as the ratio of the expected value
      * and the deviation of the diagnosed mean from the expected value
      * (and vice versa).
      */
    double warnMeanTolerance;
    
    /** \brief The error tolerance of the diagnosed mean
      * 
      * The tolerance bounds are represented by a factor in [0, 1] which
      * is symmetrically interpreted as the ratio of the expected value
      * and the deviation of the diagnosed mean from the expected value
      * (and vice versa).
      */
    double errorMeanTolerance;
    
    /** \brief The warning tolerance of the diagnosed standard deviation
      * 
      * The tolerance bound is represented by a factor in [0, 1] which
      * is interpreted as the ratio of the expected VALUE and the
      * diagnosed standard deviation.
      */
    double warnStandardDeviationTolerance;
    
    /** \brief The error tolerance of the diagnosed standard deviation
      * 
      * The tolerance bound is represented by a factor in [0, 1] which
      * is interpreted as the ratio of the expected value and the
      * diagnosed standard deviation.
      */
    double errorStandardDeviationTolerance;
  };
};

#include <roscpp_nodewrap/diagnostics/StatisticsTaskOptions.tpp>

#endif
