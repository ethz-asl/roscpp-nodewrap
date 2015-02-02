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

/** \file ParamClientOptions.h
  * \brief Header file providing the ParamClientOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_CLIENT_OPTIONS_H
#define ROSCPP_NODEWRAP_PARAM_CLIENT_OPTIONS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/ParamCallbackHelper.h>

namespace nodewrap {
  /** \brief ROS parameter client options
    * 
    * This class encapsulates all options available for creating a parameter
    * service client.
    */
  
  class ParamClientOptions {
  public:
    /** \brief Default constructor
      */
    ParamClientOptions();
    
    void init(const std::string& key, const XmlRpc::XmlRpcValue& value,
      const boost::function<bool(const XmlRpc::XmlRpcValue&)>& callback,
      bool cached = true);

    template <typename P> void init(const std::string& key, const P& value,
      const boost::function<void(const P&)>& callback, bool cached = true);
    
    std::string key;
    XmlRpc::XmlRpcValue value;
    bool cached;
    
    ParamCallbackHelperPtr helper;
    
    ros::CallbackQueueInterface* callbackQueue;
    
    ros::VoidConstPtr trackedObject;
  };
};

#include <roscpp_nodewrap/ParamClientOptions.tpp>

#endif
