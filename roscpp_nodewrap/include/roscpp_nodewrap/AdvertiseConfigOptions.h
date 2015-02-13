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

/** \file AdvertiseConfigOptions.h
  * \brief Header file providing the AdvertiseConfigOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_ADVERTISE_CONFIG_OPTIONS_H
#define ROSCPP_NODEWRAP_ADVERTISE_CONFIG_OPTIONS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS advertise configuration service options
    * 
    * This class encapsulates all options available for creating a
    * configuration service server.
    */
  class AdvertiseConfigOptions {
  public:
    /** \brief Default constructor
      */
    AdvertiseConfigOptions();
    
    /** \brief The name of the configuration service the configuration
     *    service server advertises
      */ 
    std::string service;
    
    /** \brief The callback queue to be used by the configuration service
      *   server
      */ 
    ros::CallbackQueueInterface* callbackQueue;
    
    /** \brief A shared pointer to an object to track for the configuration
      *   service server callbacks
      */ 
    ros::VoidConstPtr trackedObject;    
  };
};

#endif
