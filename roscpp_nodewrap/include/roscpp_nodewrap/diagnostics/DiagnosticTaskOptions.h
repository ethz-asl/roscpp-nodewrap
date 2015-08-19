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

/** \file DiagnosticTaskOptions.h
  * \brief Header file providing the DiagnosticTaskOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_DIAGNOSTIC_TASK_OPTIONS_H
#define ROSCPP_NODEWRAP_DIAGNOSTIC_TASK_OPTIONS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS diagnostic task options
    * 
    * This class encapsulates common options available for creating
    * diagnostic tasks.
    */
  class DiagnosticTaskOptions {
  public:
    /** \brief Default constructor
      */
    DiagnosticTaskOptions();
    
    /** \brief The namespace of the diagnostic task options
      */
    std::string ns;
  };
};

#endif
