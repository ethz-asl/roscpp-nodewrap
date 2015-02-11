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

#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/is_abstract.hpp>

#include <ros/ros.h>

#include <roscpp_nodewrap/Param.h>
#include <roscpp_nodewrap/ParamSpec.h>
#include <roscpp_nodewrap/ParamType.h>

namespace nodewrap {
  /** \brief ROS parameter service client options
    * 
    * This class encapsulates all options available for creating a parameter
    * service client.
    */
  class ParamClientOptions {
  public:
    /** \brief Default constructor
      */
    ParamClientOptions();
    
    /** \brief Templated initializer for this parameter service client options,
      *   based on the parameter type and the service request/response types
      */ 
    template <typename T, class MGetReq, class MGetRes, class MSetReq,
      class MSetRes> void init(const std::string& service, const
      boost::function<bool(const MGetRes&, T&)>& fromResponse, const
      boost::function<bool(const T&, MSetReq&)>& toReqeust, bool
      persistent = false);
    
    /** \brief Templated initializer for this parameter service client options,
      *   based on the parameter type and the service types
      */ 
    template <typename T, class GetService, class SetService> void init(
      const std::string& service, const boost::function<bool(const typename
      GetService::Response&, T&)>& fromResponse, const
      boost::function<bool(const T&, typename SetService::Request&)>&
      toRequest, bool persistent = false);
    
    /** \brief Templated initializer for this parameter service client options,
      *   based on the parameter specification
      */ 
    template <class Spec> void init(const std::string& service, const typename
      Spec::FromResponse& fromResponse, const typename Spec::ToRequest&
      toRequest, bool persistent = false);
    
    /** \brief Templated initializer for this parameter service client
      *   options, based on the parameter type
      */ 
    template <typename T> void init(const std::string& service,
      bool persistent = false, typename boost::disable_if<typename
      boost::is_abstract<Param<T> > >::type* dummy = 0);
    
    /** \brief The name of the parameter service the parameter service
      *   client connects to
      */ 
    std::string service;
    
    /** \brief The strong type of the parameter
      */ 
    ParamType type;
    
    /** \brief Helper object used for creating the parameter service client
      */
    ParamServiceHelperPtr helper;
    
    /** \brief Extra key/value pairs to add to the connection header of
      *   the parameter service client
      */ 
    ros::M_string header;
    
    /** \brief Whether or not the connections of the parameter service
      *   client should persist
      */ 
    bool persistent;
  };
};

#include <roscpp_nodewrap/ParamClientOptions.tpp>

#endif
