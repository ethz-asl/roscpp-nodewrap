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

/** \file ConfigServer.h
  * \brief Header file providing the ConfigServer class interface
  */

#ifndef ROSCPP_NODEWRAP_CONFIG_SERVER_H
#define ROSCPP_NODEWRAP_CONFIG_SERVER_H

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>

#include <roscpp_nodewrap/AdvertiseConfigOptions.h>
#include <roscpp_nodewrap/Forwards.h>
#include <roscpp_nodewrap/ParamServer.h>

#include <roscpp_nodewrap/FindParam.h>
#include <roscpp_nodewrap/HasParam.h>
#include <roscpp_nodewrap/ListParams.h>

namespace nodewrap {
  /** \brief ROS configuration service server
    * 
    * This class provides access to a node's configuration through a ROS
    * service server interface.
    */
  class ConfigServer {
  friend class NodeImpl;
  public:
    /** \brief Default constructor
      */
    ConfigServer();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source configuration service server which is
      *   being copied to this configuration service server.
      */
    ConfigServer(const ConfigServer& src);
    
    /** \brief Destructor
      */
    ~ConfigServer();
    
    /** \brief Perform shutdown of the configuration service server
      */
    void shutdown();
      
    /** \brief Advertise a parameter by this configuration service server,
      *   templated on the parameter type and with standard options
      */
    template <typename P> ParamServer advertiseParam(const std::string&
      key, const std::string& name = std::string(), const std::string&
      service = std::string(), bool cached = true);
    
    /** \brief Advertise a parameter by this configuration service server,
      *   with full range of options
      */
    ParamServer advertiseParam(const std::string& key, const
      AdvertiseParamOptions& options);
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
  private:
    /** \brief Configuration service server implementation
      * 
      * This class provides the private implementation of the configuration
      * service server.
      */
    class Impl {
    public:
      /** \brief Default constructor
        */
      Impl(const AdvertiseConfigOptions& options, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief Query if this configuration service server is valid
        */
      bool isValid() const;
      
      /** \brief Unadvertise the configuration service server's services
        */
      void unadvertise();
      
      /** \brief Advertise a parameter by this configuration service server,
        *   templated on the parameter type and with standard options
        *   (implementation)
        */
      template <typename P> ParamServer advertiseParam(const std::string&
        key, const std::string& name = std::string(), const std::string&
        service = std::string(), bool cached = true);
      
      /** \brief Advertise a parameter by this configuration service server,
        *   with full range of options (implementation)
        */
      ParamServer advertiseParam(const std::string& key, const
        AdvertiseParamOptions& options);
      
      /** \brief Callback for listening to updates of the parameters
        *   advertised by this configuration service server
        */ 
      void updateParamCallback(XmlRpc::XmlRpcValue& params,
        XmlRpc::XmlRpcValue& result);

      /** \brief Service callback listing the parameters advertised by
        *   this configuration service server
        */ 
      bool listParamsCallback(ListParams::Request& request,
        ListParams::Response& response);
      
      /** \brief Service callback for querying if a specific parameter
        *   has been advertised by this configuration service server
        */ 
      bool hasParamCallback(HasParam::Request& request, HasParam::Response&
        response);
      
      /** \brief Service callback for finding the parameter service of a
        *   parameter advertised by this configuration service server
        */ 
      bool findParamCallback(FindParam::Request& request, FindParam::Response&
        response);
      
      /** \brief Service server listing the parameter services advertised
        *   by this configuration service server
        */ 
      ros::ServiceServer listParamsServer;
      
      /** \brief Service server for querying if a specific parameter service
        *   has been advertised by this configuration service server
        */ 
      ros::ServiceServer hasParamServer;
      
      /** \brief Service server for finding the parameter service of a
        *   parameter advertised by this configuration service server
        */ 
      ros::ServiceServer findParamServer;
      
      /** \brief The name of the configuration service advertised by this
        *   configuration service server
        */ 
      std::string service;
      
      /** \brief The node implementation owning this configuration service
        *   server
        */ 
      NodeImplPtr nodeImpl;
      
      /** \brief The configuration service server's mutex
        */ 
      mutable boost::mutex mutex;
      
      /** \brief The parameter services advertised by this configuration
        *   service server
        */ 
      std::map<std::string, ParamServer::ImplWPtr> params;
    };
    
    /** \brief Declaration of the configuration service server implementation
      *   pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the configuration service server implementation
      *   weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The configuration service server's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    ConfigServer(const AdvertiseConfigOptions& options, const NodeImplPtr&
      nodeImpl);    
  };
};

#include <roscpp_nodewrap/ConfigServer.tpp>

#endif
