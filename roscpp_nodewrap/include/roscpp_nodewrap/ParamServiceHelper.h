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

/** \file ParamServiceHelper.h
  * \brief Header file providing the ParamServiceHelper class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_SERVICE_HELPER_H
#define ROSCPP_NODEWRAP_PARAM_SERVICE_HELPER_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>

namespace nodewrap {
  /** \brief ROS parameter service helper
    * 
    * This class provides helper functions which are required to create
    * the parameter services.
    */
  class ParamServiceHelper {
  friend class ConfigClient;
  friend class ConfigServer;
  friend class ParamClient;
  friend class ParamServer;
  public:
    /** \brief Constructor
      */ 
    ParamServiceHelper();
    
    /** \brief Destructor
      */ 
    virtual ~ParamServiceHelper();
    
  private:
    /** \brief Create a parameter service client (virtual declaration)
      */ 
    virtual ParamClient createClient(const ParamClientOptions& options,
      const NodeImplPtr& nodeImpl) = 0;
      
    /** \brief Create a parameter service server (virtual declaration)
      */ 
    virtual ParamServer createServer(const ParamServerOptions& options,
      const NodeImplPtr& nodeImpl) = 0;
  };
  
  /** \brief ROS parameter service helper (templated version)
    */    
  template <typename Spec> class ParamServiceHelperT :
    public ParamServiceHelper {
  public:
    /** \brief Definition of the parameter value type derived from the
      *   parameter specification
      */
    typedef typename Spec::Value Value;
    
    /** \brief Definition of the service request type derived from the
      *   parameter specification for querying the value of the parameter
      *   from the parameter service server
      */
    typedef typename Spec::GetValueServiceRequest GetValueServiceRequest;
    
    /** \brief Definition of the service response type derived from the
      *   parameter specification for querying the value of the parameter
      *   from the parameter service server
      */
    typedef typename Spec::GetValueServiceResponse GetValueServiceResponse;
    
    /** \brief Definition of the service request type derived from the
      *   parameter specification for modifying the value of the parameter
      *   through the parameter service server
      */
    typedef typename Spec::SetValueServiceRequest SetValueServiceRequest;
    
    /** \brief Definition of the service response type derived from the
      *   parameter specification for modifying the value of the parameter
      *   through the parameter service server
      */
    typedef typename Spec::SetValueServiceResponse SetValueServiceReponse;
    
    /** \brief Definition of the function type derived from the parameter
      *   specification for assigning the parameter's value from an XML/RPC
      *   value
      */
    typedef typename Spec::FromXmlRpcValue FromXmlRpcValue;
    
    /** \brief Definition of the function type derived from the parameter
      *   specification for assigning the parameter's value to an XML/RPC
      *   value
      */
    typedef typename Spec::ToXmlRpcValue ToXmlRpcValue;
    
    /** \brief Definition of the function type derived from the parameter
      *   specification for assigning the parameter's value from a service
      *   request
      */
    typedef typename Spec::FromRequest FromRequest;
    
    /** \brief Definition of the function type derived from the parameter
      *   specification for assigning the parameter's value to a service
      *   request
      */
    typedef typename Spec::ToRequest ToRequest;
    
    /** \brief Definition of the function type derived from the parameter
      *   specification for assigning the parameter's value from a service
      *   response
      */
    typedef typename Spec::FromResponse FromResponse;
    
    /** \brief Definition of the function type derived from the parameter
      *   specification for assigning the parameter's value to a service
      *   response
      */
    typedef typename Spec::ToResponse ToResponse;
    
    /** \brief Constructor (client version)
      */ 
    ParamServiceHelperT(const FromResponse& fromResponse, const ToRequest&
      toRequest);
    
    /** \brief Constructor (server version)
      */ 
    ParamServiceHelperT(const FromXmlRpcValue& fromXmlRpcValue, const
      ToXmlRpcValue& toXmlRpcValue, const FromRequest& fromRequest,
      const ToResponse& toResponse);
    
  private:
    /** \brief Create a parameter service client according to the parameter
      *   specification (implementation)
      */ 
    ParamClient createClient(const ParamClientOptions& options,
      const NodeImplPtr& nodeImpl);
    
    /** \brief Create a parameter service server according to the parameter
      *   specification (implementation)
      */ 
    ParamServer createServer(const ParamServerOptions& options,
      const NodeImplPtr& nodeImpl);
    
    /** \brief The function for assigning the parameter's value from an
      *   XML/RPC value
      */
    FromXmlRpcValue fromXmlRpcValue;
    
    /** \brief The function for assigning the parameter's value to an
      *   XML/RPC value
      */
    ToXmlRpcValue toXmlRpcValue;

    /** \brief The function for assigning the parameter's value from a
      *   service request
      */
    FromRequest fromRequest;
    
    /** \brief The function for assigning the parameter's value to a
      *   service request
      */
    ToRequest toRequest;
    
    /** \brief The function for assigning the parameter's value from a
      *   service response
      */
    FromResponse fromResponse;
    
    /** \brief The function for assigning the parameter's value to a
      *   service response
      */
    ToResponse toResponse;
  };
};

#include <roscpp_nodewrap/ParamServiceHelper.tpp>

#endif
