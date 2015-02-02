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

#include "roscpp_nodewrap/ParamClient.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ParamClient::ParamClient() {
}

ParamClient::ParamClient(const ParamClient& src) :
  impl(src.impl) {
}

ParamClient::ParamClient(const std::string& key, bool cached, const
    boost::shared_ptr<NodeImpl>& nodeImpl) {
  init(key, cached, nodeImpl);
}

ParamClient::Impl::Impl(const std::string& key, bool cached, const
    boost::shared_ptr<NodeImpl>& nodeImpl) :
  key(key),
  cached(cached),
  nodeImpl(nodeImpl) {
}

ParamClient::~ParamClient() {
}

ParamClient::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ParamClient::getKey() const {
  if (impl && impl->isValid())
    return impl->key;
  else
    return std::string();
}

bool ParamClient::isCached() const {
}

bool ParamClient::Impl::isValid() const {
  return getParamNameServer.operator void*() &&
    getParamValueServer.operator void*() &&
    setParamValueServer.operator void*() &&
    getParamInfoServer.operator void*();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ParamClient::shutdown() {
  if (impl)
    impl->unadvertise();
}

ros::ServiceServer ParamClient::Impl::advertise(ros::AdvertiseServiceOptions&
    options) {
  return nodeImpl->getNodeHandle().advertiseService(options);
}
      
void ParamClient::Impl::unadvertise() {
  if (isValid()) {
    getParamNameServer.shutdown();
    getParamValueServer.shutdown();
    setParamValueServer.shutdown();
    getParamInfoServer.shutdown();
  }
}

bool ParamClient::Impl::getParamName(GetParamName::Request& request,
    GetParamName::Response& response) {
  response.name = nodeImpl->getNodeHandle().resolveName(key);
  return true;
}

bool ParamClient::Impl::getParamInfo(GetParamInfo::Request& request,
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
