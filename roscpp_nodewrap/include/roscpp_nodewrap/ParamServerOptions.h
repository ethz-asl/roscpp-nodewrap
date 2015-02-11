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

/** \file ParamServerOptions.h
  * \brief Header file providing the ParamServerOptions class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_SERVER_OPTIONS_H
#define ROSCPP_NODEWRAP_PARAM_SERVER_OPTIONS_H

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_abstract.hpp>

#include <ros/ros.h>

#include <roscpp_nodewrap/Param.h>
#include <roscpp_nodewrap/ParamSpec.h>
#include <roscpp_nodewrap/ParamType.h>

namespace nodewrap {
  /** \brief ROS parameter service server options
    * 
    * This class encapsulates all options available for creating a parameter
    * service server.
    */
  class ParamServerOptions {
  public:
    /** \brief Default constructor
      */
    ParamServerOptions();
    
    /** \brief Templated initializer for this parameter service server options,
      *   based on the parameter type and the service request/response types
      */ 
    template <typename T, class MGetReq, class MGetRes, class MSetReq,
      class MSetRes> void init(const std::string& service, const std::string&
      name, const boost::function<bool(const XmlRpc::XmlRpcValue&, T&)>&
      fromXmlRpcValue, const boost::function<bool(const T&,
      XmlRpc::XmlRpcValue&)>& toXmlRpcValue, const boost::function<bool(const
      MSetReq&, T&)>& fromRequest, const boost::function<bool(const T&,
      MGetRes&)>& toResponse, bool cached = true);
    
    /** \brief Templated initializer for this parameter service server options,
      *   based on the parameter type and the service types
      */ 
    template <typename T, class GetService, class SetService> void init(
      const std::string& service, const std::string& name, const
      boost::function<bool(const XmlRpc::XmlRpcValue&, T&)>& fromXmlRpcValue,
      const boost::function<bool(const T&, XmlRpc::XmlRpcValue&)>&
      toXmlRpcValue, const boost::function<bool(const typename
      SetService::Request&, T&)>& fromRequest, const boost::function<bool(const
      T&, typename GetService::Response&)>& toResponse, bool cached = true);
    
    /** \brief Templated initializer for this parameter service server options,
      *   based on the parameter specification
      */ 
    template <class Spec> void init(const std::string& service, const
      std::string& name, const typename Spec::FromXmlRpcValue& fromXmlRpcValue,
      const typename Spec::ToXmlRpcValue& toXmlRpcValue, const typename
      Spec::FromRequest& fromRequest, const typename Spec::ToResponse&
      toResponse, bool cached = true);
    
    /** \brief Templated initializer for this parameter service server
      *   options, based on the parameter type
      */ 
    template <typename T> void init(const std::string& service, const
      std::string& name, bool cached = true, typename boost::disable_if<
      typename boost::is_abstract<Param<T> > >::type* dummy = 0);
    
    /** \brief The name of the parameter service the parameter service
      *   server advertises
      */ 
    std::string service;
    
    /** \brief The ROS name of the parameter under which it can be accessed
      *   through rosparam
      */ 
    std::string name;
    
    /** \brief The strong type of the parameter
      */ 
    ParamType type;
    
    /** \brief True, if the cached parameter value shall be provided by the
      *   parameter service server
      */     
    bool cached;

    /** \brief Helper object used for creating the parameter service server
      */
    ParamServiceHelperPtr helper;
    
    /** \brief The callback queue to be used by the parameter service server
      */ 
    ros::CallbackQueueInterface* callbackQueue;
    
    /** \brief A shared pointer to an object to track for the parameter
      *   service server callbacks
      */ 
    ros::VoidConstPtr trackedObject;    
  };
};

#include <roscpp_nodewrap/ParamServerOptions.tpp>

#endif
