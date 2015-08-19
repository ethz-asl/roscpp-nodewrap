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

/** \file ServiceClient.h
  * \brief Header file providing the ServiceClient class interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_CLIENT_H
#define ROSCPP_NODEWRAP_SERVICE_CLIENT_H

#include <roscpp_nodewrap/ServiceClientImpl.h>

#include <roscpp_nodewrap/diagnostics/ServiceClientStatusTask.h>

namespace nodewrap {
  /** \brief ROS service client wrapper
    * 
    * This class is a wrapper for native ROS service client. Its interface
    * is largely identical to that of the native ROS service client, but
    * provides additional accessors and diagnostics.
    */
  
  class ServiceClient {
  friend class NodeImpl;
  friend class ServiceClientStatusTask;
  public:
    /** \brief Default constructor
      */
    ServiceClient();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source service client which is being copied
      *   to this service client.
      */
    ServiceClient(const ServiceClient& src);
    
    /** \brief Destructor
      */
    ~ServiceClient();
    
    /** \brief Retrieve the service this service client connects to
      */
    std::string getService() const;
    
    /** \brief True, if this service client is valid and, for a persistent
      *   service, the connection to the service server has not dropped
      */
    bool isValid() const;
    
    /** \brief True, if the service client uses a persistent connection
      *   to the service server
      */
    bool isPersistent() const;
    
    /** \brief True, if the service is both advertised and available
      */
    bool exists() const;
    
    /** \brief Wait for this service client's service to be advertised and
      *   available.
      */
    bool waitForExistence(const ros::Duration& timeout = ros::Duration(-1));
    
    /** \brief Perform shutdown of the service client
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const ServiceClient& serviceClient) const {
      return (impl < serviceClient.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const ServiceClient& serviceClient) const {
      return (impl == serviceClient.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const ServiceClient& serviceClient) const {
      return (impl != serviceClient.impl);
    };
    
    /** \brief Call the service
      */     
    template <class S> bool call(S& service);
  
    /** \brief Call the service, with service request/response messages
      */     
    template <class MReq, class MRes> bool call(MReq& request, MRes& response);
  
    /** \brief Convert this wrapped service client to a native ROS service
      *   client
      */ 
    operator ros::ServiceClient() const;
      
  private:
    /** \brief ROS service client wrapper implementation
      * 
      * This class provides the private implementation of the ROS
      * service client wrapper.
      */
    class Impl :
      public ServiceClientImpl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const NodeImplPtr& node);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the node owning this service client
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief True, if this service client implementation is valid
        */ 
      bool isValid() const;
      
      /** \brief Initialize the service client
        */
      void init(const ServiceClientOptions& defaultOptions);
            
      /** \brief Perform shutdown of the service client (implementation)
        */
      void shutdown();
            
      /** \brief The name of this service client
        */ 
      std::string name;
            
      /** \brief The type of service connected to by this service client
        */ 
      std::string serviceType;
      
      /** \brief The request message type of the service connected to by
        *   this service client
        */ 
      std::string serviceRequestType;
      
      /** \brief The response message type of the service connected to by
        *   this service client
        */ 
      std::string serviceResponseType;
      
      /** \brief The md5sum of the service type connected to by this
        *   service client
        */ 
      std::string serviceMd5Sum;
      
      /** \brief The number of requests successfully served by the service
        *   server connected to by this service client
        */ 
      size_t numServedRequests;
      
      /** \brief The number of requests not served by the service server
        *   connected to by this service client due to failure
        */ 
      size_t numFailedRequests;
      
      /** \brief The ROS service client
        */ 
      ros::ServiceClient serviceClient;
      
      /** \brief The node implementation owning this publisher
        */ 
      NodeImplPtr node;
      
      /** \brief The diagnostic task for monitoring the status of this
        *   publisher
        */ 
      ServiceClientStatusTask statusTask;
    };

    /** \brief Declaration of the service client implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the service client implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The service client's implementation
      */
    ImplPtr impl;
  };
};

#include <roscpp_nodewrap/ServiceClient.tpp>

#endif
