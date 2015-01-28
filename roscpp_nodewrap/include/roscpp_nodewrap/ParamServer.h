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

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>

#include <roscpp_nodewrap/GetParamName.h>
#include <roscpp_nodewrap/GetParamInfo.h>
#include <roscpp_nodewrap/ParamTraits.h>

namespace nodewrap {
  class NodeImpl;
  
  using namespace roscpp_nodewrap;
  
  /** \brief ROS parameter service server
    * 
    * This class provides access to a node's parameters through a ROS
    * service server interface.
    */
  
  class ParamServer {
  friend class NodeImpl;
  public:
    /** \brief Default constructor
      */
    ParamServer();
    
    /** \brief Copy constructor
      */
    ParamServer(const ParamServer& src);
    
    /** \brief Destructor
      */
    ~ParamServer();
    
    std::string getKey() const;
    
    template <typename P> void setValue(const P& value);
    template <typename P> P getValue(const P& defaultValue) const;
    
    void shutdown();

    inline operator void*() const {
      return (impl && impl->isValid()) ? (void*)1 : (void*)0;
    };
    
    inline bool operator<(const ParamServer& paramServer) const {
      return (impl < paramServer.impl);
    };

    inline bool operator==(const ParamServer& paramServer) const {
      return (impl == paramServer.impl);
    };
    
    inline bool operator!=(const ParamServer& paramServer) const {
      return (impl != paramServer.impl);
    };
    
  private:
    /** \brief Private constructors, with full range of options
      */
    ParamServer(const std::string& key, const XmlRpc::XmlRpcValue& value,
      bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl);
    template <typename P> ParamServer(const std::string& key, const P&
      value, bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl);
    template <typename P, bool Cached> ParamServer(const std::string& key,
      const P& value, const boost::shared_ptr<NodeImpl>& nodeImpl);
    
    template <typename P> void init(const std::string& key, const P& value,
      bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl);
    template <typename P, bool Cached> void init(const std::string& key,
      const P& value, const boost::shared_ptr<NodeImpl>& nodeImpl);
    
    class Impl {
    public:
      Impl(const std::string& key, XmlRpc::XmlRpcValue::Type type,
        bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl);
      ~Impl();
      
      std::string getNamespace() const;
      bool isValid() const;
      
      ros::ServiceServer advertise(ros::AdvertiseServiceOptions& options);
      void unadvertise();
      
      bool getParamName(GetParamName::Request& request,
        GetParamName::Response& response);
      bool getParamInfo(GetParamInfo::Request& request,
        GetParamInfo::Response& response);
      
      std::string key;
      XmlRpc::XmlRpcValue::Type type;
      bool cached;
      
      ros::ServiceServer getParamNameServer;
      ros::ServiceServer getParamValueServer;
      ros::ServiceServer setParamValueServer;
      ros::ServiceServer getParamInfoServer;
      
      boost::shared_ptr<NodeImpl> nodeImpl;
      boost::mutex mutex;
    };
    typedef boost::shared_ptr<Impl> ImplPtr;
    
    ImplPtr impl;
    
    template <typename P> class ImplT :
      public Impl {
    public:
      typedef typename ParamTraits<P>::SetParamValue SetValueService;
      typedef typename ParamTraits<P>::GetParamValue GetValueService;
      
      typedef boost::function<bool(typename SetValueService::Request&,
        typename SetValueService::Response&)> SetValueCallback;
      typedef boost::function<bool(typename GetValueService::Request&,
        typename GetValueService::Response&)> GetValueCallback;
      
      ImplT(const std::string& key, bool cached, const SetValueCallback&
        setValueCallback, const GetValueCallback& getValueCallback, const
        boost::shared_ptr<NodeImpl>& nodeImpl);
      ~ImplT();
    };
    
    template <typename P, bool Cached> class ImplT2;
    
    template <typename P> class ImplT2<P, false> :
      public ImplT<P> {
    public:
      typedef typename ImplT<P>::SetValueService SetValueService;
      typedef typename ImplT<P>::GetValueService GetValueService;
      
      ImplT2(const std::string& key, const P& value, const
        boost::shared_ptr<NodeImpl>& nodeImpl);
      ~ImplT2();
      
      bool setParamValue(typename SetValueService::Request& request,
        typename SetValueService::Response& response);
      bool getParamValue(typename GetValueService::Request& request,
        typename GetValueService::Response& response);
    };
    
    template <typename P> class ImplT2<P, true> :
      public ImplT<P> {
    public:
      typedef typename ImplT<P>::SetValueService SetValueService;
      typedef typename ImplT<P>::GetValueService GetValueService;
      
      ImplT2(const std::string& key, const P& value, const
        boost::shared_ptr<NodeImpl>& nodeImpl);
      ~ImplT2();
      
      bool setParamValue(typename SetValueService::Request& request,
        typename SetValueService::Response& response);
      bool getParamValue(typename GetValueService::Request& request,
        typename GetValueService::Response& response);
      
      P value;
    };
  };
};

#include <roscpp_nodewrap/ParamServer.tpp>

#endif
