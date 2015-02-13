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

/** \file ParamServerCallbacks.h
  * \brief Header file providing the ParamServerCallbacks class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_SERVER_CALLBACKS_H
#define ROSCPP_NODEWRAP_PARAM_SERVER_CALLBACKS_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Parameter service server callbacks
    */
  class ParamServerCallbacks {
  protected:
    /** \brief Default constructor
      */ 
    ParamServerCallbacks();
  };
  
  /** \brief Parameter service server callbacks (templated version)
    */
  template <class Spec> class ParamServerCallbacksT :
    public ParamServerCallbacks {
  public:
    /** \brief Definition of the callback function type derived from the
      *   parameter specification for assigning the parameter's value from
      *   an XML/RPC value
      */
    typedef typename Spec::FromXmlRpcValueCallback FromXmlRpcValue;
    
    /** \brief Definition of the callback function type derived from the
      *   parameter specification for assigning the parameter's value to
      *   an XML/RPC value
      */
    typedef typename Spec::ToXmlRpcValueCallback ToXmlRpcValue;
  
    /** \brief Definition of the callback function type derived from the
      *   parameter specification for assigning the parameter's value from
      *   a service request
      */
    typedef typename Spec::FromRequestCallback FromRequest;
    
    /** \brief Definition of the callback function type derived from the
      *   parameter specification for assigning the parameter's value to
      *   a service response
      */
    typedef typename Spec::ToResponseCallback ToResponse;
    
    /** \brief Constructor, with full range of options
      */ 
    ParamServerCallbacksT(const FromXmlRpcValue& fromXmlRpcValue, const
      ToXmlRpcValue& toXmlRpcValue, const FromRequest& fromRequest, const
      ToResponse& toResponse);
    
    /** \brief Copy constructor
      */ 
    ParamServerCallbacksT(const ParamServerCallbacksT<Spec>& src);
    
    /** \brief The callback function for assigning the parameter's value
      *   from an XML/RPC value
      */
    FromXmlRpcValue fromXmlRpcValue;
    
    /** \brief The callback function for assigning the parameter's value
      *   to an XML/RPC value
      */
    ToXmlRpcValue toXmlRpcValue;
    
    /** \brief The callback function for assigning the parameter's value
      *   from a service request
      */
    FromRequest fromRequest;
    
    /** \brief The callback function for assigning the parameter's value
      *   to a service response
      */
    ToResponse toResponse;
  };
};

#include <roscpp_nodewrap/ParamServerCallbacks.tpp>

#endif
