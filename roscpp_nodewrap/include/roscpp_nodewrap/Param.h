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

/** \file Param.h
  * \brief Header file providing the Param class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_H
#define ROSCPP_NODEWRAP_PARAM_H

#include <ros/ros.h>

#include <roscpp_nodewrap/ParamSpec.h>
#include <roscpp_nodewrap/ParamTraits.h>

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
  
  /** \brief ROS templated parameter class wrapper (default implementation)
    * 
    * This class provides type definitions and conversion functions for
    * standard parameter value types through template specialization.
    * To support other, custom parameter value types, you may provide
    * your own specializations.
    * 
    * By default, that is, for any unsupported parameter value type, the
    * templated parameter class wrapper is abstract.
    */
  template <typename T, class Enable = void> class Param {
  public:
    /** \brief A purely virtual method rendering this class abstract
      */ 
    virtual void unsupported() = 0;
  };
  
  /** \brief ROS templated parameter class wrapper (XML/RPC value
    *   specialization)
    */
  template <> class Param<XmlRpc::XmlRpcValue> {
  public:
    typedef typename ParamSpecS<XmlRpc::XmlRpcValue, GetParamValueXml,
      SetParamValueXml>::ToParamSpec Spec;
      
    static bool fromXmlRpcValue(const XmlRpc::XmlRpcValue& xmlRpcValue,
      XmlRpc::XmlRpcValue& value);
    static bool toXmlRpcValue(const XmlRpc::XmlRpcValue& value,
      XmlRpc::XmlRpcValue& xmlRpcValue);
    static bool fromRequest(const SetParamValueXml::Request& request,
      XmlRpc::XmlRpcValue& value);
    static bool toRequest(const XmlRpc::XmlRpcValue& value,
      SetParamValueXml::Request& request);
    static bool fromResponse(const GetParamValueXml::Response& response,
      XmlRpc::XmlRpcValue& value);
    static bool toResponse(const XmlRpc::XmlRpcValue& value,
      GetParamValueXml::Response& response);
  };
  
  /** \brief ROS templated parameter class wrapper (string value
    *   specialization)
    */
  template <> class Param<std::string> {
  public:
    typedef typename ParamSpecS<std::string, GetParamValueString,
      SetParamValueString>::ToParamSpec Spec;
      
    static bool fromXmlRpcValue(const XmlRpc::XmlRpcValue& xmlRpcValue,
      std::string& value);
    static bool toXmlRpcValue(const std::string& value,
      XmlRpc::XmlRpcValue& xmlRpcValue);
    static bool fromRequest(const SetParamValueString::Request& request,
      std::string& value);
    static bool toRequest(const std::string& value,
      SetParamValueString::Request& request);
    static bool fromResponse(const GetParamValueString::Response& response,
      std::string& value);
    static bool toResponse(const std::string& value,
      GetParamValueString::Response& response);
  };

  /** \brief ROS templated parameter class wrapper (floating-point-type
    *   value specialization)
    */
  template <typename T> class Param<T, typename
    boost::enable_if<typename ParamTraits::IsFloat<T> >::type> {
  public:
    typedef typename ParamSpecS<T, GetParamValueDouble,
      SetParamValueDouble>::ToParamSpec Spec;
      
    inline static bool fromXmlRpcValue(const XmlRpc::XmlRpcValue&
      xmlRpcValue, T& value);
    inline static bool toXmlRpcValue(const T& value, XmlRpc::XmlRpcValue&
      xmlRpcValue);
    inline static bool fromRequest(const SetParamValueDouble::Request&
      request, T& value);
    inline static bool toRequest(const T& value, SetParamValueDouble::Request&
      request);
    inline static bool fromResponse(const GetParamValueDouble::Response&
      response, T& value);
    inline static bool toResponse(const T& value,
      GetParamValueDouble::Response& response);
  };

  /** \brief ROS templated parameter class wrapper (integral-type
    *   value specialization)
    */
  template <typename T> class Param<T, typename
    boost::enable_if<typename ParamTraits::IsInteger<T> >::type> {
  public:
    typedef typename ParamSpecS<T, GetParamValueInt,
      SetParamValueInt>::ToParamSpec Spec;
      
    inline static bool fromXmlRpcValue(const XmlRpc::XmlRpcValue& xmlRpcValue,
      T& value);
    inline static bool toXmlRpcValue(const T& value, XmlRpc::XmlRpcValue&
      xmlRpcValue);
    inline static bool fromRequest(const SetParamValueInt::Request& request,
      T& value);
    inline static bool toRequest(const T& value, SetParamValueInt::Request&
      request);
    inline static bool fromResponse(const GetParamValueInt::Response&
      response, T& value);
    inline static bool toResponse(const T& value, GetParamValueInt::Response&
      response);
  };

  /** \brief ROS templated parameter class wrapper (boolean-type
    *   value specialization)
    */
  template <> class Param<bool> {
  public:
    typedef typename ParamSpecS<bool, GetParamValueBool,
      SetParamValueBool>::ToParamSpec Spec;
      
    static bool fromXmlRpcValue(const XmlRpc::XmlRpcValue& xmlRpcValue,
      bool& value);
    static bool toXmlRpcValue(const bool& value, XmlRpc::XmlRpcValue&
      xmlRpcValue);
    static bool fromRequest(const SetParamValueBool::Request& request,
      bool& value);
    static bool toRequest(const bool& value, SetParamValueBool::Request&
      request);
    static bool fromResponse(const GetParamValueBool::Response& response,
      bool& value);
    static bool toResponse(const bool& value, GetParamValueBool::Response&
      response);
  };
};

#include <roscpp_nodewrap/Param.tpp>

#endif
