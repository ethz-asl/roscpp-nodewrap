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

/** \file SubscriberStatusTaskOptions.h
  * \brief Header file providing the SubscriberStatusTaskOptions class
  *   interface
  */

#ifndef ROSCPP_NODEWRAP_SUBSCRIBER_STATUS_TASK_OPTIONS_H
#define ROSCPP_NODEWRAP_SUBSCRIBER_STATUS_TASK_OPTIONS_H

#include <roscpp_nodewrap/diagnostics/DiagnosticTaskOptions.h>

namespace nodewrap {
  /** \brief ROS subscriber status task options
    * 
    * This class encapsulates all options available for creating a
    * diagnostic subscriber status task.
    */
  class SubscriberStatusTaskOptions :
    public DiagnosticTaskOptions {
  public:
    /** \brief Default constructor
      */
    SubscriberStatusTaskOptions();
    
    /** \brief The minimum number of publishers of the diagnosed
      *   subscriber
      */ 
    size_t minNumPublishers;
  };
};

#endif
