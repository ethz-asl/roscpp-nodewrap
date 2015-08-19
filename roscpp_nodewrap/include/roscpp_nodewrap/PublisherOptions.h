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

/** \file PublisherOptions.h
  * \brief Header file providing the PublisherOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_PUBLISHER_OPTIONS_H
#define ROSCPP_NODEWRAP_PUBLISHER_OPTIONS_H

#include <roscpp_nodewrap/diagnostics/FrequencyTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/LatencyTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/PublisherStatusTaskOptions.h>

namespace nodewrap {
  /** \brief ROS publisher wrapper options
    * 
    * This class encapsulates all options available for creating a ROS
    * publisher wrapper.
    */
  class PublisherOptions :
    public ros::AdvertiseOptions {
  public:
    /** \brief Default constructor
      */
    PublisherOptions();

    /** \brief Copy constructor
      */
    PublisherOptions(const PublisherOptions& src);
    
    /** \brief Copy constructor (ROS-compatible version)
      */
    PublisherOptions(const ros::AdvertiseOptions& src);
    
    /** \brief The namespace of the publisher options
      */
    std::string ns;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   status of the publisher
      */ 
    PublisherStatusTaskOptions statusTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   message publishing frequency of the publisher
      */ 
    FrequencyTaskOptions publishingFrequencyTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   message timestamp frequency of the publisher
      */ 
    FrequencyTaskOptions messageStampFrequencyTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   message latency of the publisher
      */ 
    LatencyTaskOptions messageLatencyTaskOptions;
  };
};

#endif
