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

/** \file ConfigClient.h
  * \brief Header file providing the ConfigClient class interface
  */

#ifndef ROSCPP_NODEWRAP_CONFIG_CLIENT_H
#define ROSCPP_NODEWRAP_CONFIG_CLIENT_H

#include <ros/ros.h>

#include <roscpp_nodewrap/Forwards.h>
#include <roscpp_nodewrap/ParamServer.h>

#include <roscpp_nodewrap/FindParam.h>
#include <roscpp_nodewrap/HasParam.h>
#include <roscpp_nodewrap/ListParams.h>

namespace nodewrap {
  /** \brief ROS configuration service client
    * 
    * This class provides access to a node's configuration through a ROS
    * service client interface.
    */
  class ConfigClient {
  friend class NodeImpl;
  public:
    /** \brief Default constructor
      */
    ConfigClient();
    
    /** \brief Copy constructor
      * 
      * \param[in] src The source configuration service client which is
      *   being copied to this configuration service client.
      */
    ConfigClient(const ConfigClient& src);
    
    /** \brief Destructor
      */
    ~ConfigClient();
    
    /** \brief Access the name of the configuration service this configuration
      *   service client connects to
      */
    std::string getService() const;
    
    /** \brief List the keys of the parameters advertised by the connected
      *   configuration service
      */
    std::vector<std::string> getParamKeys();
    
    /** \brief Query the name of the parameter service associated with a
      *   parameter advertised by the connected configuration service
      */
    std::string getParamService(const std::string& key);
    
    /** \brief Query if a parameter is advertised by the connected
      *   configuration service
      */
    bool hasParam(const std::string& key);
    
    /** \brief Query if this configuration service client is valid
      */
    bool isValid() const;
    
    /** \brief Query if this configuration service client uses persistent
      *   connections
      */
    bool isPersistent() const;
    
    /** \brief Query if the configuration service connected to by this
      *   configuration service client exist
      */
    bool exists() const;
    
    /** \brief Wait for the configuration service connected to by this
      *   configuration service client to become available
      */
    bool waitForExistence(ros::Duration timeout = ros::Duration(-1));
      
    /** \brief Perform shutdown of the configuration service client
      */
    void shutdown();
      
    /** \brief Create a client for a parameter service advertised by the
      *   connected configuration service
      */
    ParamClient paramClient(const std::string& key, const ParamClientOptions&
      options, ros::Duration timeout = ros::Duration(-1));
    
    /** \brief Void pointer conversion
      */
    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
        
  private:
    /** \brief Configuration service client implementation
      * 
      * This class provides the private implementation of the configuration
      * service client.
      */
    class Impl {
    public:
      /** \brief Default constructor
        */
      Impl(const std::string& service, const NodeImplPtr& nodeImpl);
      
      /** \brief Destructor
        */
      ~Impl();
      
      /** \brief List the keys of the parameters advertised by the connected
        *   configuration service (implementation)
        */
      std::vector<std::string> getParamKeys();
      
      /** \brief Query the name of the parameter service associated with a
        *   parameter advertised by the connected configuration service
        *   (implementation)
        */
      std::string getParamService(const std::string& key);
      
      /** \brief Query if a parameter is advertised by the connected
        *   configuration service (implementation)
        */
      bool hasParam(const std::string& key);
    
      /** \brief Query if this configuration service client is valid
        */
      bool isValid() const;
      
      /** \brief Disconnect from the configuration service connected to by
        *   this configuration service client
        */
      void disconnect();
      
      /** \brief Service client listing the parameter services advertised
        *   by this configuration service client
        */ 
      ros::ServiceClient listParamsClient;
      
      /** \brief Service client for querying if a specific parameter service
        *   has been advertised by this configuration service client
        */ 
      ros::ServiceClient hasParamClient;
      
      /** \brief Service client for finding the parameter service of a
        *   parameter advertised by this configuration service client
        */ 
      ros::ServiceClient findParamClient;
      
      /** \brief The name of the configuration service connected to by this
        *   configuration service client
        */ 
      std::string service;
      
      /** \brief The node implementation owning this configuration service
        *   client
        */ 
      NodeImplPtr nodeImpl;      
    };
    
    /** \brief Declaration of the configuration service client implementation
      *   pointer type
      */
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    /** \brief Declaration of the configuration service client implementation
      *   weak pointer type
      */
    typedef boost::weak_ptr<Impl> ImplWPtr;
    
    /** \brief The configuration service client's implementation
      */
    ImplPtr impl;
    
    /** \brief Constructor (private version)
      */
    ConfigClient(const std::string& service, const NodeImplPtr& nodeImpl);
  };
};

#endif
