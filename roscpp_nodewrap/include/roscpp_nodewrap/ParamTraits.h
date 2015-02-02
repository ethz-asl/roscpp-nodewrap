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

/** \file ParamTraits.h
  * \brief Header file providing the ParamTraits class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_TRAITS_H
#define ROSCPP_NODEWRAP_PARAM_TRAITS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/SetParamValueXml.h>
#include <roscpp_nodewrap/GetParamValueXml.h>
#include <roscpp_nodewrap/SetParamValueString.h>
#include <roscpp_nodewrap/GetParamValueString.h>
#include <roscpp_nodewrap/SetParamValueDouble.h>
#include <roscpp_nodewrap/GetParamValueDouble.h>
#include <roscpp_nodewrap/SetParamValueInt.h>
#include <roscpp_nodewrap/GetParamValueInt.h>
#include <roscpp_nodewrap/SetParamValueBool.h>
#include <roscpp_nodewrap/GetParamValueBool.h>

#include <std_srvs/Empty.h>

namespace nodewrap {
  using namespace roscpp_nodewrap;
  
  /** \brief ROS parameter traits
    * 
    * This class provides commonly used traits for different parameter
    * types.
    */
  
  template <typename P> struct ParamTraits {
    enum { XmlRpcValueType = XmlRpc::XmlRpcValue::TypeInvalid };
    
    typedef std_srvs::Empty SetParamValue;
    typedef std_srvs::Empty GetParamValue;
  };
  
  template <> struct ParamTraits<XmlRpc::XmlRpcValue> {
    enum { XmlRpcValueType = XmlRpc::XmlRpcValue::TypeInvalid };
    
    typedef SetParamValueXml SetParamValue;
    typedef GetParamValueXml GetParamValue;
  };
  
  template <> struct ParamTraits<std::string> {
    enum { XmlRpcValueType = XmlRpc::XmlRpcValue::TypeString };
    
    typedef SetParamValueString SetParamValue;
    typedef GetParamValueString GetParamValue;
  };

  template <> struct ParamTraits<double> {
    enum { XmlRpcValueType = XmlRpc::XmlRpcValue::TypeDouble };
    
    typedef SetParamValueDouble SetParamValue;
    typedef GetParamValueDouble GetParamValue;
  };
  
  template <> struct ParamTraits<int> {
    enum { XmlRpcValueType = XmlRpc::XmlRpcValue::TypeInt };
    
    typedef SetParamValueInt SetParamValue;
    typedef GetParamValueInt GetParamValue;
  };
  
  template <> struct ParamTraits<bool> {
    enum { XmlRpcValueType = XmlRpc::XmlRpcValue::TypeBoolean };
    
    typedef SetParamValueBool SetParamValue;
    typedef GetParamValueBool GetParamValue;
  };
  
  template <typename P> inline bool paramToValue(const ros::NodeHandle&
    nodeHandle, const std::string& key, P& value);
  template <typename P> inline void valueToParam(const ros::NodeHandle&
    nodeHandle, const P& value, const std::string& key);
  
  template <typename P> inline void messageToValue(const typename
    ParamTraits<P>::SetParamValue::Request& message, P& value);
  template <typename P> inline void valueToMessage(const P& value,
    typename ParamTraits<P>::GetParamValue::Response& message);
  
  template <> inline void messageToValue<XmlRpc::XmlRpcValue>(const
    SetParamValueXml::Request& message, XmlRpc::XmlRpcValue& value);
  template <> inline void valueToMessage<XmlRpc::XmlRpcValue>(const
    XmlRpc::XmlRpcValue& value, GetParamValueXml::Response& message);
  
  template <typename P> inline void valueToXmlRpcValue(const P& value,
    XmlRpc::XmlRpcValue& xmlRpcValue);
};

#include <roscpp_nodewrap/ParamTraits.tpp>

#endif
