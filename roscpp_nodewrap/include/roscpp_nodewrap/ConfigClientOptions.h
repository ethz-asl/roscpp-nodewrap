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

/** \file ConfigClientOptions.h
  * \brief Header file providing the ConfigClientOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_CONFIG_CLIENT_OPTIONS_H
#define ROSCPP_NODEWRAP_CONFIG_CLIENT_OPTIONS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS configuration service client options
    * 
    * This class encapsulates all options available for creating a
    * configuration service client.
    */
  class ConfigClientOptions {
  public:
    /** \brief Default constructor
      */
    ConfigClientOptions();
    
    /** \brief The name of the configuration service the configuration
      *   service client connects to
      */ 
    std::string service;
    
    /** \brief Extra key/value pairs to add to the connection header of
      *   the configuration service client
      */ 
    ros::M_string header;
    
    /** \brief Whether or not the connections of the configuration service
      *   client should persist
      */ 
    bool persistent;
  };
};

#endif
