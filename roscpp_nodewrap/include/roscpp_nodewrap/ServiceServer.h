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

/** \file ServiceServer.h
  * \brief Header file providing the ServiceServer class interface
  */

#ifndef ROSCPP_NODEWRAP_SERVICE_SERVER_H
#define ROSCPP_NODEWRAP_SERVICE_SERVER_H

#include <roscpp_nodewrap/ServiceServerImpl.h>

#include <roscpp_nodewrap/diagnostics/ServiceServerStatusTask.h>

namespace nodewrap {
  /** \brief ROS service server wrapper
    * 
    * This class is a wrapper for native ROS service server. Its interface
    * is largely identical to that of the native ROS service server, but
    * provides additional accessors and diagnostics.
    */
  
  class ServiceServer {
  friend class NodeImpl;
  template <class MReq, class MRes> friend class ServiceServerCallbackHelperT;
  friend class ServiceServerStatusTask;
  public:
    /** \brief Default constructor
      */
    ServiceServer();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source service server which is being copied
      *   to this service server.
      */
    ServiceServer(const ServiceServer& src);
    
    /** \brief Destructor
      */
    ~ServiceServer();
    
    /** \brief Retrieve the service this service server advertises
      */
    std::string getService() const;
    
    /** \brief Perform shutdown of the service server
      */
    void shutdown();
      
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    /** \brief Lesser comparison operator
      */
    inline bool operator<(const ServiceServer& serviceServer) const {
      return (impl < serviceServer.impl);
    };
    
    /** \brief Equality comparison operator
      */
    inline bool operator==(const ServiceServer& serviceServer) const {
      return (impl == serviceServer.impl);
    };
    
    /** \brief Inequality comparison operator
      */
    inline bool operator!=(const ServiceServer& serviceServer) const {
      return (impl != serviceServer.impl);
    };
    
    /** \brief Convert this wrapped service server to a native ROS
      *   service server
      */ 
    operator ros::ServiceServer() const;
      
  private:
    /** \brief ROS service server wrapper implementation
      * 
      * This class provides the private implementation of the ROS
      * service server wrapper.
      */
    class Impl :
      public ServiceServerImpl {
    public:
      /** \brief Constructor
        */
      Impl(const std::string& name, const NodeImplPtr& node);
      
      /** \brief Destructor
        */
      virtual ~Impl();
      
      /** \brief Retrieve the node owning this service server
        */ 
      const NodeImplPtr& getNode() const;
      
      /** \brief True, if this service server implementation is valid
        */ 
      bool isValid() const;
      
      /** \brief Initialize the service server
        */
      void init(const ServiceServerOptions& defaultOptions);
            
      /** \brief Perform shutdown of the service server (implementation)
        */
      void shutdown();
            
      /** \brief The name of this service server
        */ 
      std::string name;
            
      /** \brief The type of service advertised by this service server
        */ 
      std::string serviceType;
      
      /** \brief The request message type of the service advertised by
        *   this service server
        */ 
      std::string serviceRequestType;
      
      /** \brief The response message type of the service advertised by
        *   this service server
        */ 
      std::string serviceResponseType;
      
      /** \brief The md5sum of the service type advertised by this
        *   service server
        */ 
      std::string serviceMd5Sum;
      
      /** \brief The number of requests successfully served by this
        *   service server
        */ 
      size_t numServedRequests;
      
      /** \brief The number of requests not served by this service server
        *   due to failure
        */ 
      size_t numFailedRequests;
      
      /** \brief The ROS service server
        */ 
      ros::ServiceServer serviceServer;
      
      /** \brief The node implementation owning this service server
        */ 
      NodeImplPtr node;
      
      /** \brief The diagnostic task for monitoring the status of this
        *   service server
        */ 
      ServiceServerStatusTask statusTask;
    };

    /** \brief Declaration of the service server implementation pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the service server implementation weak pointer
      *   type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The service server's implementation
      */
    ImplPtr impl;
  };
};

#endif
