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

/** \file ParamCallbackHelper.h
  * \brief Header file providing the ParamCallbackHelper class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_CALLBACK_HELPER_H
#define ROSCPP_NODEWRAP_PARAM_CALLBACK_HELPER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS parameter callback helper
    * 
    * This class encapsulates the callback functions which are required
    * to operate the parameter services.
    */
  class ParamCallbackHelper {
  public:
    /** \brief Constructor
      */ 
    ParamCallbackHelper();
    
    /** \brief Destructor
      */ 
    virtual ~ParamCallbackHelper();
  };
  
  /** \brief ROS parameter callback helper (templated version)
    */    
  template <typename Spec> class ParamCallbackHelperT :
    public ParamCallbackHelper {
  public:
    /** \brief Definitions of the parameter value type derived from the
      *   parameter specifications
      */
    typedef typename Spec::Value Value;
    
    /** \brief Definition of the set/get service request/response types
      *   derived from the parameter specifications
      */
    typedef typename Spec::GetValueServiceRequest GetValueServiceRequest;
    typedef typename Spec::GetValueServiceResponse GetValueServiceResponse;
    typedef typename Spec::SetValueServiceRequest SetValueServiceRequest;
    typedef typename Spec::SetValueServiceResponse SetValueServiceReponse;
    
    /** \brief Definition of the callback function types derived from
      *   the parameter specifications
      */
    typedef typename Spec::FromXmlRpcValueCallback FromXmlRpcValueCallback;
    typedef typename Spec::ToXmlRpcValueCallback ToXmlRpcValueCallback;
    typedef typename Spec::FromRequestCallback FromRequestCallback;
    typedef typename Spec::ToResponseCallback ToResponseCallback;
    
    /** \brief Constructor
      */ 
    ParamCallbackHelperT(
      const FromXmlRpcValueCallback& fromXmlRpcValueCallback,
      const ToXmlRpcValueCallback& toXmlRpcValueCallback,
      const FromRequestCallback& fromRequestCallback,
      const ToResponseCallback& toResponseCallback);
    
  private:
    /** \brief The callback function for assigning the parameter's value
      *   from an XML/RPC value
      */
    FromXmlRpcValueCallback fromXmlRpcValueCallback;
    
    /** \brief The callback function for assigning the parameter's value
      *   to an XML/RPC value
      */
    ToXmlRpcValueCallback fromXmlRpcValueCallback;

    /** \brief The callback function for assigning the parameter's value
      *   from a service request
      */
    FromRequestCallback fromRequestCallback;
    
    /** \brief The callback function for assigning the parameter's value
      *   to a service response
      */
    ToResponseCallback toResponseCallback;
  };
};

#include <roscpp_nodewrap/ParamCallbackHelper.tpp>

#endif
