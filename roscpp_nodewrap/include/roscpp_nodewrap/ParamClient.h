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

/** \file ParamClient.h
  * \brief Header file providing the ParamClient class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_CLIENT_H
#define ROSCPP_NODEWRAP_PARAM_CLIENT_H

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/Forwards.h>
#include <roscpp_nodewrap/ParamType.h>

#include <roscpp_nodewrap/GetParamInfo.h>

namespace nodewrap {
  class NodeImpl;
  
  using namespace roscpp_nodewrap;
  
  /** \brief ROS parameter service client
    * 
    * This class provides access to a node's parameters through a ROS
    * service client interface.
    */
  class ParamClient {
  friend class ConfigClient;
  friend class NodeImpl;
  friend class ParamServiceHelper;
  template <class Spec> friend class ParamServiceHelperT;
  public:
    /** \brief Default constructor
      */
    ParamClient();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source parameter service client which is being
      *   copied to this parameter service client.
      */
    ParamClient(const ParamClient& src);
    
    /** \brief Destructor
      */
    ~ParamClient();
    
    /** \brief Access the name of the parameter service this parameter
      *   service client connects to
      */
    std::string getService() const;
    
    /** \brief Query the ROS name of the parameter advertised by the
      *   connected parameter service
      */
    std::string getParamName();
    
    /** \brief Modify the value of the parameter advertised by the
      *   connected parameter service
      */
    template <typename T> bool setParamValue(const T& value);
    
    /** \brief Query the value of the parameter advertised by the
      *   connected parameter service
      */
    template <typename T> bool getParamValue(T& value);
    
    /** \brief Query if this parameter service client is valid
      */
    bool isValid() const;
    
    /** \brief Query if this parameter service client uses persistent
      *   connections
      */
    bool isPersistent() const;
    
    /** \brief Query if the parameter service connected to by this
      *   parameter service client exist
      */
    bool exists() const;
    
    /** \brief Wait for the parameter service connected to by this
      *   parameter service client to become available
      */
    bool waitForExistence(ros::Duration timeout = ros::Duration(-1));
      
    /** \brief Perform shutdown of the parameter service client
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const ParamClient& paramClient) const {
      return (impl < paramClient.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const ParamClient& paramClient) const {
      return (impl == paramClient.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const ParamClient& paramClient) const {
      return (impl != paramClient.impl);
    };
    
  private:
    /** \brief ROS parameter service client implementation
      * 
      * This class provides the private implementation of the parameter
      * service client.
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const ParamClientOptions& options, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Query the ROS name of the parameter advertised by the
        *   connected parameter service (implementation)
        */
      std::string getParamName();
    
      /** \brief Query if this parameter service client is valid
        */
      bool isValid() const;
      
      /** \brief Create the service clients of this parameter service client
        */
      ros::ServiceClient client(const ros::ServiceClientOptions& options);
      
      /** \brief Disconnect from the parameter service connected to by this
        *   parameter service client
        */
      void disconnect();
      
      /** \brief Service client for querying the value of the parameter
        *   advertised by the connected parameter service
        */ 
      ros::ServiceClient getParamValueClient;
      
      /** \brief Service client for modifying the value of the parameter
        *   advertised by the connected parameter service
        */ 
      ros::ServiceClient setParamValueClient;
      
      /** \brief Service client for querying information about the parameter
        *   advertised by the connected parameter service
        */ 
      ros::ServiceClient getParamInfoClient;
      
      /** \brief The name of the parameter service connected to by this
        *   parameter service client
        */ 
      std::string service;
      
      /** \brief The type of the parameter advertised by the connected
        *   parameter service
        */ 
      ParamType type;
      
      /** \brief The parameter service client's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The node implementation owning this parameter service
        *   client
        */ 
      NodeImplPtr nodeImpl;
    };
    
    /** \brief ROS parameter service client implementation (templated version)
      * 
      * This class provides the private templated implementation of the
      * parameter service client.
      */
    template <class Spec> class ImplT :
      public Impl {
    public:
      /** \brief Definition of the parameter value type derived from the
        *   parameter specifications
        */
      typedef typename Spec::Value Value;
      
      /** \brief Definition of the service request type derived from the
        *   parameter specifications for querying the value of the parameter
        *   advertised by the connected parameter service
        */
      typedef typename Spec::GetValueServiceRequest GetValueServiceRequest;
      
      /** \brief Definition of the service response type derived from the
        *   parameter specifications for querying the value of the parameter
        *   advertised by the connected parameter service
        */
      typedef typename Spec::GetValueServiceResponse GetValueServiceResponse;
      
      /** \brief Definition of the service request type derived from the
        *   parameter specifications for modifying the value of the parameter
        *   advertised by the connected parameter service
        */
      typedef typename Spec::SetValueServiceRequest SetValueServiceRequest;
      
      /** \brief Definition of the service response type derived from the
        *   parameter specifications for modifying the value of the parameter
        *   advertised by the connected parameter service
        */
      typedef typename Spec::SetValueServiceResponse SetValueServiceResponse;
    
      /** \brief Definition of the function type derived from the parameter
        *   specification for assigning the parameter's value from a service
        *   response
        */
      typedef typename Spec::FromResponse FromResponse;
      
      /** \brief Definition of the function type derived from the parameter
        *   specification for assigning the parameter's value to a service
        *   request
        */
      typedef typename Spec::ToRequest ToRequest;
      
      /** \brief Constructor
        */
      ImplT(const FromResponse& fromResponse, const ToRequest& toRequest,
        const ParamClientOptions& options, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      virtual ~ImplT();
      
      /** \brief Query the value of the parameter advertised by the
        *   connected parameter service
        */ 
      bool getParamValue(Value& value);
      
      /** \brief Modify the value of the parameter advertised by the
        *   connected parameter service
        */
      bool setParamValue(const Value& value);
      
      /** \brief The function for assigning the value of the parameter
        *   advertised by the connected parameter service from a service
        *   response
        */
      FromResponse fromResponse;
      
      /** \brief The function for assigning the value of the parameter
        *   advertised by the connected parameter service to a service
        *   request
        */
      ToRequest toRequest;
    };
    
    /** \brief Declaration of the parameter service client implementation
      *   pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the parameter service client implementation
      *   weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;

    /** \brief The  parameter service client's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    ParamClient(const ParamClientOptions& options, const NodeImplPtr&
      nodeImpl);
  };        
};

#include <roscpp_nodewrap/ParamClient.tpp>

#endif
