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

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <roscpp_nodewrap/ParamTraits.h>

namespace nodewrap {
  class NodeImpl;
  
  using namespace roscpp_nodewrap;
  
  /** \brief ROS parameter service client
    * 
    * This class provides access to a node's parameters through a ROS
    * service client interface.
    */
  
  class ParamClient {
  friend class NodeImpl;
  public:
    /** \brief Default constructor
      */
    ParamClient();
    
    /** \brief Copy constructor
      */
    ParamClient(const ParamClient& src);
    
    /** \brief Destructor
      */
    ~ParamClient();
    
    const std::string& getNamespace() const;
    const std::string& getKey() const;
    
    void setValue(const XmlRpc::XmlRpcValue& value);
    XmlRpc::XmlRpcValue getValue() const;
    XmlRpc::XmlRpcValue getValue(const XmlRpc::XmlRpcValue&
      defaultValue) const;
    
    void shutdown();

    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    inline bool operator<(const ParamClient& paramClient) const {
      return (impl < paramClient.impl);
    };

    inline bool operator==(const ParamClient& paramClient) const {
      return (impl == paramClient.impl);
    };
    
    inline bool operator!=(const ParamClient& paramClient) const {
      return (impl != paramClient.impl);
    };
    
  private:
    /** \brief Private constructors, with full range of options
      */
    ParamClient(const std::string& ns, const std::string& key,
      bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl);
    
    void init(const std::string& key, bool cached, const
      boost::shared_ptr<NodeImpl>& nodeImpl);
    
    class Impl {
    public:
      Impl(const std::string& key, bool cached, const
        boost::shared_ptr<NodeImpl>& nodeImpl);
      ~Impl();
      
      bool isValid() const;
      
      ros::ServiceClient subscribe(ros::ServiceClientOptions& options);
      void unsubscribe();
      
//       bool getParamName(GetParamName::Request& request,
//         GetParamName::Response& response);
//       bool getParamInfo(GetParamInfo::Request& request,
//         GetParamInfo::Response& response);
      
      std::string ns;
      std::string key;
      bool cached;
      
//       ros::ServiceClient getParamNameClient;
//       ros::ServiceClient setParamValueClient;
//       ros::ServiceClient getParamValueClient;
//       ros::ServiceClient getParamInfoClient;
      
      boost::shared_ptr<NodeImpl> nodeImpl;
      boost::mutex mutex;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    ImplPtr impl;
  };
};

#include <roscpp_nodewrap/ParamClient.tpp>

#endif
