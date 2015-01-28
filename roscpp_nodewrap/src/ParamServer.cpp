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

#include <ros/master.h>
#include <ros/names.h>
#include <ros/xmlrpc_manager.h>

#include <roscpp_nodewrap/NodeImpl.h>

#include "roscpp_nodewrap/ParamServer.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ParamServer::ParamServer() {
}

ParamServer::ParamServer(const ParamServer& src) :
  impl(src.impl) {
}

ParamServer::ParamServer(const std::string& key, const XmlRpc::XmlRpcValue&
    value, bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl) {
  switch (value.getType()) {
    case XmlRpc::XmlRpcValue::TypeString:
      init<std::string>(key, const_cast<XmlRpc::XmlRpcValue&>(value).operator
        std::string&(), cached, nodeImpl);
      break;
    case XmlRpc::XmlRpcValue::TypeDouble:
      init<double>(key, const_cast<XmlRpc::XmlRpcValue&>(value).operator
        double&(), cached, nodeImpl);
      break;
    case XmlRpc::XmlRpcValue::TypeInt:
      init<int>(key, const_cast<XmlRpc::XmlRpcValue&>(value).operator
        int&(), cached, nodeImpl);
      break;
    case XmlRpc::XmlRpcValue::TypeBoolean:
      init<bool>(key, const_cast<XmlRpc::XmlRpcValue&>(value).operator
        bool&(), cached, nodeImpl);
      break;
    default:
      init<XmlRpc::XmlRpcValue>(key, value, cached, nodeImpl);
  }
}
  
//   if (cached) {
//     XmlRpc::XmlRpcValue params, result, payload;
//     
//     params[0] = nodeImpl->getNodeHandle().getNamespace();
//     params[1] = ros::XMLRPCManager::instance()->getServerURI();
//     params[2] = nodeImpl->getNodeHandle().resolveName(key);
//     
//     ros::master::execute("subscribeParam", params, result, payload, false);
//   }
// }

ParamServer::Impl::Impl(const std::string& key, XmlRpc::XmlRpcValue::Type
    type, bool cached, const boost::shared_ptr<NodeImpl>& nodeImpl) :
  key(key),
  type(type),
  cached(cached),
  nodeImpl(nodeImpl) {
  ros::AdvertiseServiceOptions getParamNameOptions;
  getParamNameOptions.init<GetParamName::Request, GetParamName::Response>(
    getNamespace()+"/get_name", boost::bind(&ParamServer::Impl::getParamName,
    this, _1, _2));
  getParamNameServer = advertise(getParamNameOptions);
  
  ros::AdvertiseServiceOptions getParamInfoOptions;
  getParamInfoOptions.init<GetParamInfo::Request, GetParamInfo::Response>(
    getNamespace()+"/get_info", boost::bind(&ParamServer::Impl::getParamInfo,
    this, _1, _2));
  getParamInfoServer = advertise(getParamInfoOptions);
}

ParamServer::~ParamServer() {
}

ParamServer::Impl::~Impl() {
  unadvertise();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ParamServer::getKey() const {
  if (impl && impl->isValid())
    return impl->key;
  else
    return std::string();
}
 
std::string ParamServer::Impl::getNamespace() const {
  return std::string("params/")+key;
}

bool ParamServer::Impl::isValid() const {
  return getParamNameServer.operator void*() &&
    getParamValueServer.operator void*() &&
    setParamValueServer.operator void*() &&
    getParamInfoServer.operator void*();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ParamServer::shutdown() {
  if (impl)
    impl->unadvertise();
}

ros::ServiceServer ParamServer::Impl::advertise(ros::AdvertiseServiceOptions&
    options) {
  return nodeImpl->getNodeHandle().advertiseService(options);
}
      
void ParamServer::Impl::unadvertise() {
  if (isValid()) {
    getParamNameServer.shutdown();
    getParamValueServer.shutdown();
    setParamValueServer.shutdown();
    getParamInfoServer.shutdown();
  }
}

bool ParamServer::Impl::getParamName(GetParamName::Request& request,
    GetParamName::Response& response) {
  response.name = nodeImpl->getNodeHandle().resolveName(key);
  return true;
}

bool ParamServer::Impl::getParamInfo(GetParamInfo::Request& request,
    GetParamInfo::Response& response) {
  switch (type) {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      response.type = GetParamInfo::Response::TYPE_BOOLEAN;
      break;
    case XmlRpc::XmlRpcValue::TypeInt:
      response.type = GetParamInfo::Response::TYPE_INT;
      break;
    case XmlRpc::XmlRpcValue::TypeDouble:
      response.type = GetParamInfo::Response::TYPE_DOUBLE;
      break;
    case XmlRpc::XmlRpcValue::TypeString:
      response.type = GetParamInfo::Response::TYPE_STRING;
      break;
    case XmlRpc::XmlRpcValue::TypeDateTime:
      response.type = GetParamInfo::Response::TYPE_DATETIME;
      break;
    case XmlRpc::XmlRpcValue::TypeBase64:
      response.type = GetParamInfo::Response::TYPE_BASE64;
      break;
    case XmlRpc::XmlRpcValue::TypeArray:
      response.type = GetParamInfo::Response::TYPE_ARRAY;
      break;
    case XmlRpc::XmlRpcValue::TypeStruct:
      response.type = GetParamInfo::Response::TYPE_STRUCT;
      break;
    default:
      response.type = GetParamInfo::Response::TYPE_INVALID;
  }
  
//   response.size = value.size();
//   response.limits = ;
//   response.members = ;

//   response.valid = value.valid();
  response.cached = cached;
  
  return true;
}

}
