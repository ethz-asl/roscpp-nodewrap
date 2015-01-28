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

#include <roscpp_nodewrap/ListParams.h>
#include <roscpp_nodewrap/HasParam.h>

namespace nodewrap {
  class NodeImpl;
  
  using namespace roscpp_nodewrap;
  
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
      */
    ConfigServer(const ConfigServer& src);
    
    /** \brief Destructor
      */
    ~ConfigServer();
    
    void shutdown();

    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    inline bool operator<(const ConfigServer& configServer) const {
      return (impl < configServer.impl);
    };

    inline bool operator==(const ConfigServer& configServer) const {
      return (impl == configServer.impl);
    };
    
    inline bool operator!=(const ConfigServer& configServer) const {
      return (impl != configServer.impl);
    };
    
  private:
    /** \brief Private constructor, with full range of options
      */
    ConfigServer(const boost::shared_ptr<NodeImpl>& nodeImpl);

    class Impl {
    public:
      Impl(const boost::shared_ptr<NodeImpl>& nodeImpl);
      ~Impl();
      
      bool isValid() const;
      
      void unadvertise();
      
      bool listParams(ListParams::Request& request, ListParams::Response&
        response);
      bool hasParam(HasParam::Request& request, HasParam::Response& response);
      
      void updateParam(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue&
        result);
      
      ros::ServiceServer listParamsServer;
      ros::ServiceServer hasParamServer;
      
      boost::shared_ptr<NodeImpl> nodeImpl;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    ImplPtr impl;
  };
};

#endif
