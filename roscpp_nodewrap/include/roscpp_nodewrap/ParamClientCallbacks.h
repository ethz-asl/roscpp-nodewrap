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

/** \file ParamClientCallbacks.h
  * \brief Header file providing the ParamClientCallbacks class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_CLIENT_CALLBACKS_H
#define ROSCPP_NODEWRAP_PARAM_CLIENT_CALLBACKS_H

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief Parameter service client callbacks
    */
  class ParamClientCallbacks {
  protected:
    /** \brief Default constructor
      */ 
    ParamClientCallbacks();
  };
  
  /** \brief Parameter service client callbacks (templated version)
    */
  template <class Spec> class ParamClientCallbacksT :
    public ParamClientCallbacks {
  public:    
    /** \brief Definition of the callback function type derived from the
      *   parameter specification for assigning the parameter's value from
      *   a service response
      */
    typedef typename Spec::FromResponseCallback FromResponse;
    
    /** \brief Definition of the callback function type derived from the
      *   parameter specification for assigning the parameter's value to
      *   a service request
      */
    typedef typename Spec::ToRequestCallback ToRequest;
    
    /** \brief Constructor, with full range of options
      */ 
    ParamClientCallbacksT(const FromResponse& fromResponse, const ToRequest&
      toRequest);
    
    /** \brief Copy constructor
      */ 
    ParamClientCallbacksT(const ParamClientCallbacksT<Spec>& src);
    
    /** \brief The callback function for assigning the parameter's value
      *   from a service response
      */
    FromResponse fromResponse;
    
    /** \brief The callback function for assigning the parameter's value
      *   to a service request
      */
    ToRequest toRequest;
  };
};

#include <roscpp_nodewrap/ParamClientCallbacks.tpp>

#endif
