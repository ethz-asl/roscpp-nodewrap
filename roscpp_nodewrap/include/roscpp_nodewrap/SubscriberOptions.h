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

/** \file SubscriberOptions.h
  * \brief Header file providing the SubscriberOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_SUBSCRIBER_OPTIONS_H
#define ROSCPP_NODEWRAP_SUBSCRIBER_OPTIONS_H

#include <roscpp_nodewrap/diagnostics/FrequencyTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/LatencyTaskOptions.h>
#include <roscpp_nodewrap/diagnostics/SubscriberStatusTaskOptions.h>

namespace nodewrap {
  /** \brief ROS subscriber wrapper options
    * 
    * This class encapsulates all options available for creating a ROS
    * subscriber wrapper.
    */
  class SubscriberOptions :
    public ros::SubscribeOptions {
  public:
    /** \brief Default constructor
      */
    SubscriberOptions();

    /** \brief Copy constructor
      */
    SubscriberOptions(const SubscriberOptions& src);
    
    /** \brief Copy constructor (ROS-compatible version)
      */
    SubscriberOptions(const ros::SubscribeOptions& src);
    
    /** \brief Initialize the subscriber options
      */
    template <class M> void init(
      const std::string& topic, uint32_t queue_size, const
      boost::function<void(const boost::shared_ptr<M const>&)>& callback);
    
    /** \brief The namespace of the subscriber options
      */
    std::string ns;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   status of the subscriber
      */ 
    SubscriberStatusTaskOptions statusTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   message processing frequency of the subscriber
      */ 
    FrequencyTaskOptions processingFrequencyTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   message timestamp frequency of the subscriber
      */ 
    FrequencyTaskOptions messageStampFrequencyTaskOptions;
    
    /** \brief The options of the diagnostic task for monitoring the
      *   message latency of the subscriber
      */ 
    LatencyTaskOptions messageLatencyTaskOptions;
  };
};

#include <roscpp_nodewrap/SubscriberOptions.tpp>

#endif
