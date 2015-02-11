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

/** \file ParamSpec.h
  * \brief Header file providing the ParamSpec class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_SPEC_H
#define ROSCPP_NODEWRAP_PARAM_SPEC_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS parameter specification, templated on the parameter type
    *   and message types
    * 
    * This class encapsulates parameter specifications which are required
    * to operate the parameter services based on the parameter type and the
    * message types.
    */
  template <typename T, class MGetReq, class MGetRes, class MSetReq,
    class MSetRes> class ParamSpec {
  public:
    /** \brief Definition of the parameter value type
      */
    typedef T Value;
    
    /** \brief Definition of the service request type for querying the
      *   parameter's value
      */
    typedef MGetReq GetValueServiceRequest;
    
    /** \brief Definition of the service response type for querying the
      *   parameter's value
      */
    typedef MGetRes GetValueServiceResponse;
    
    /** \brief Definition of the service request type for modifying the
      *   parameter's value
      */
    typedef MSetReq SetValueServiceRequest;
    
    /** \brief Definition of the service response type for modifying the
      *   parameter's value
      */
    typedef MSetRes SetValueServiceResponse;

    /** \brief Definition of the function type for assigning the parameter's
      *   value from an XML/RPC value
      */
    typedef boost::function<bool(const XmlRpc::XmlRpcValue&, T&)>
      FromXmlRpcValue;
      
    /** \brief Definition of the function type for assigning the parameter's
      *   value to an XML/RPC value
      */
    typedef boost::function<bool(const T&, XmlRpc::XmlRpcValue&)>
      ToXmlRpcValue;
      
    /** \brief Definition of the function type for assigning the parameter's
      *   value from a service request
      */
    typedef boost::function<bool(const MSetReq&, T&)> FromRequest;
    
    /** \brief Definition of the function type for assigning the parameter's
      *   value to a service request
      */
    typedef boost::function<bool(const T&, MSetReq&)> ToRequest;
    
    /** \brief Definition of the function type for assigning the parameter's
      *   value from a service response
      */
    typedef boost::function<bool(const MGetRes&, T&)> FromResponse;
    
    /** \brief Definition of the function type for assigning the parameter's
      *   value to a service response
      */
    typedef boost::function<bool(const T&, MGetRes&)> ToResponse;
  };
  
  /** \brief ROS parameter specification, templated on the parameter type
    *   and service types
    * 
    * This class encapsulates parameter specifications which are required
    * to operate the parameter services based on the parameter type and the
    * service types.
    */
  template <typename T, class SGet, class SSet> class ParamSpecS {
  public:
    /** \brief Definition of the parameter specification, templated on the
      *   parameter type and message types, which corresponds to this 
      *   parameter specification
      */
    typedef ParamSpec<T, typename SGet::Request, typename SGet::Response,
      typename SSet::Request, typename SSet::Response> ToParamSpec;
  };
};

#endif
