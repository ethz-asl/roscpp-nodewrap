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

/** \file ParamServer.h
  * \brief Header file providing the ParamServer class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_SERVER_H
#define ROSCPP_NODEWRAP_PARAM_SERVER_H

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/Forwards.h>
#include <roscpp_nodewrap/ParamType.h>

#include <roscpp_nodewrap/GetParamInfo.h>

namespace nodewrap {
  class NodeImpl;
  
  using namespace roscpp_nodewrap;
  
  /** \brief ROS parameter service server
    * 
    * This class provides access to a node's parameters through a ROS
    * service server interface.
    */
  class ParamServer {
  friend class ConfigServer;
  friend class ParamServiceHelper;
  template <class Spec> friend class ParamServiceHelperT;
  public:
    /** \brief Default constructor
      */
    ParamServer();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source parameter service server which is being
      *   copied to this parameter service server.
      */
    ParamServer(const ParamServer& src);
    
    /** \brief Destructor
      */
    ~ParamServer();
    
    /** \brief Access the name of the parameter service advertised by this
      *   parameter service server
      */
    std::string getService() const;
    
    /** \brief Perform shutdown of the parameter service server
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const ParamServer& paramServer) const {
      return (impl < paramServer.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const ParamServer& paramServer) const {
      return (impl == paramServer.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const ParamServer& paramServer) const {
      return (impl != paramServer.impl);
    };
    
  private:
    /** \brief ROS parameter service server implementation
      * 
      * This class provides the private implementation of the parameter
      * service server.
      */
    class Impl {
    public:
      /** \brief Constructor
        */
      Impl(const ParamServerOptions& options, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Query the XML/RPC value of the parameter advertised by
        *   this parameter service server
        */ 
      bool getParamXmlRpcValue(XmlRpc::XmlRpcValue& value);
      
      /** \brief Modify the XML/RPC value of the parameter advertised by
        *   this parameter service server
        */ 
      bool setParamXmlRpcValue(const XmlRpc::XmlRpcValue& value);
      
      /** \brief Query if this parameter service server is valid
        */
      bool isValid() const;
      
      /** \brief Advertise the parameter service server's services
        */
      ros::ServiceServer advertise(const ros::AdvertiseServiceOptions&
        options);
      
      /** \brief Unadvertise the parameter service server's services
        */
      void unadvertise();
      
      /** \brief Service callback for querying information about the parameter
        *   advertised by this parameter service server
        */ 
      bool getParamInfoCallback(GetParamInfo::Request& request,
        GetParamInfo::Response& response);
      
      /** \brief Service server for querying the value of the parameter
        *   advertised by this parameter service server
        */ 
      ros::ServiceServer getParamValueServer;
      
      /** \brief Service server for modifying the value of the parameter
        *   advertised by this parameter service server
        */ 
      ros::ServiceServer setParamValueServer;
      
      /** \brief Service server for querying information about the parameter
        *   advertised by this parameter service server
        */ 
      ros::ServiceServer getParamInfoServer;
      
      /** \brief The name of the parameter service advertised by this
        *   parameter service server
        */ 
      std::string service;
      
      /** \brief The ROS name of the parameter advertised by this parameter
        *   service server
        */ 
      std::string name;
      
      /** \brief The type of the parameter advertised by this parameter
        *   service server
        */ 
      ParamType type;
      
      /** \brief True, if the cached parameter value is provided by this
        *   parameter service server
        */ 
      bool cached;
      
      /** \brief The parameter service server's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The node implementation owning this parameter service
        *   server
        */ 
      NodeImplPtr nodeImpl;
    };
    
    /** \brief ROS parameter service server implementation (templated version)
      * 
      * This class provides the private templated implementation of the
      * parameter service server.
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
        *   advertised by this parameter service server
        */
      typedef typename Spec::GetValueServiceRequest GetValueServiceRequest;
      
      /** \brief Definition of the service response type derived from the
        *   parameter specifications for querying the value of the parameter
        *   advertised by this parameter service server
        */
      typedef typename Spec::GetValueServiceResponse GetValueServiceResponse;
      
      /** \brief Definition of the service request type derived from the
        *   parameter specifications for modifying the value of the parameter
        *   advertised by this parameter service server
        */
      typedef typename Spec::SetValueServiceRequest SetValueServiceRequest;
      
      /** \brief Definition of the service response type derived from the
        *   parameter specifications for modifying the value of the parameter
        *   advertised by this parameter service server
        */
      typedef typename Spec::SetValueServiceResponse SetValueServiceResponse;
    
      /** \brief Definition of the function type for assigning the parameter's
        *   value from an XML/RPC value
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
        *   response
        */
      typedef typename Spec::ToResponse ToResponse;
      
      /** \brief Constructor
        */
      ImplT(const FromXmlRpcValue& fromXmlRpcValue, const ToXmlRpcValue&
        toXmlRpcValue, const FromRequest& fromRequest, const ToResponse&
        toResponse, const ParamServerOptions& options, const NodeImplPtr&
        nodeImpl);
      
      /** \brief Destructor
        */
      virtual ~ImplT();
      
      /** \brief Query the value of the parameter advertised by this
        *   parameter service server
        */ 
      bool getParamValue(Value& value);
      
      /** \brief Modify the value of the parameter advertised by this
        *   parameter service server
        */
      bool setParamValue(const Value& value);
      
      /** \brief Service callback for querying the value of the parameter
        *   advertised by this parameter service server
        */ 
      bool getParamValueCallback(GetValueServiceRequest& request, 
        GetValueServiceResponse& response);
      
      /** \brief Service callback for modifying the value of the parameter
        *   advertised by this parameter service server
        */ 
      bool setParamValueCallback(SetValueServiceRequest& request,
        SetValueServiceResponse& response);
      
      /** \brief The function for assigning the value of the parameter
        *   advertised by this parameter service server from an XML/RPC
        *   value
        */
      FromXmlRpcValue fromXmlRpcValue;
      
      /** \brief The function for assigning the value of the parameter
        *   advertised by this parameter service server to an XML/RPC
        *   value
        */
      ToXmlRpcValue toXmlRpcValue;
      
      /** \brief The function for assigning the value of the parameter
        *   advertised by this parameter service server from a service
        *   request
        */
      FromRequest fromRequest;
      
      /** \brief The function for assigning the value of the parameter
        *   advertised by this parameter service server to a service
        *   response
        */
      ToResponse toResponse;
    };
    
    /** \brief Declaration of the parameter service server implementation
      *   pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the parameter service server implementation
      *   weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;

    /** \brief The  parameter service server's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    ParamServer(const ParamServerOptions& options, const NodeImplPtr&
      nodeImpl);
  };        
};

#include <roscpp_nodewrap/ParamServer.tpp>

#endif
