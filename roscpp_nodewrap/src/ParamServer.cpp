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

#include <boost/thread/locks.hpp>

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/ParamServerOptions.h>

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

ParamServer::ParamServer(const ParamServerOptions& options, const NodeImplPtr&
    nodeImpl) :
  impl(options.helper->createServer(options, nodeImpl).impl) {
}

ParamServer::~ParamServer() {
}

ParamServer::Impl::Impl(const ParamServerOptions& options, const NodeImplPtr&
    nodeImpl) :
  service(options.service),
  name(options.name.empty() ?
    nodeImpl->getNodeHandle().resolveName(options.service) :
    nodeImpl->getNodeHandle().resolveName(options.name)),
  type(options.type),
  cached(options.cached),
  nodeImpl(nodeImpl) {
  ros::AdvertiseServiceOptions getParamInfoOptions;
  getParamInfoOptions.init<GetParamInfo::Request, GetParamInfo::Response>(
    ros::names::append(options.service, "get_info"),
    boost::bind(&ParamServer::Impl::getParamInfoCallback,
    this, _1, _2));
  getParamInfoOptions.callback_queue = options.callbackQueue;
  getParamInfoOptions.tracked_object = options.trackedObject;
  getParamInfoServer = advertise(getParamInfoOptions);
}

ParamServer::Impl::~Impl() {
  unadvertise();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ParamServer::getService() const {
  if (impl)
    return impl->service;
  else
    return std::string();
}

bool ParamServer::Impl::getParamXmlRpcValue(XmlRpc::XmlRpcValue& value) {
  boost::mutex::scoped_lock lock(mutex);
  
  if (cached)
    return nodeImpl->getNodeHandle().getParamCached(name, value);
  else
    return nodeImpl->getNodeHandle().getParam(name, value);
}

bool ParamServer::Impl::setParamXmlRpcValue(const XmlRpc::XmlRpcValue& value) {
  boost::mutex::scoped_lock lock(mutex);
  
  nodeImpl->getNodeHandle().setParam(name, value);
  return true;
}

bool ParamServer::Impl::isValid() const {
  return getParamValueServer && setParamValueServer && getParamInfoServer;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ParamServer::shutdown() {
  if (impl)
    impl->unadvertise();
}

ros::ServiceServer ParamServer::Impl::advertise(const
    ros::AdvertiseServiceOptions& options) {
  return nodeImpl->getNodeHandle().advertiseService(
    const_cast<ros::AdvertiseServiceOptions&>(options));
}

void ParamServer::Impl::unadvertise() {
  if (isValid()) {
    getParamValueServer.shutdown();
    setParamValueServer.shutdown();
    getParamInfoServer.shutdown();
  }
}

bool ParamServer::Impl::getParamInfoCallback(GetParamInfo::Request& request,
    GetParamInfo::Response& response) {
  XmlRpc::XmlRpcValue xmlRpcValue;
  getParamXmlRpcValue(xmlRpcValue);
  
  response.name = name;
  response.valid = xmlRpcValue.valid();
  
  if (response.valid) {
    response.xmlrpc_type = xmlRpcValue.getType();
    response.cached = cached;
    
    if ((response.xmlrpc_type == XmlRpc::XmlRpcValue::TypeString) ||
        (response.xmlrpc_type == XmlRpc::XmlRpcValue::TypeBase64) ||
        (response.xmlrpc_type == XmlRpc::XmlRpcValue::TypeArray) ||
        (response.xmlrpc_type == XmlRpc::XmlRpcValue::TypeStruct))
      response.xmlrpc_size = xmlRpcValue.size();
  }
  else {
    response.xmlrpc_type = XmlRpc::XmlRpcValue::TypeInvalid;
    response.cached = false;
    response.xmlrpc_size = 0;
  }
  
  response.strong_type = type.getName();
  
  return true;
}

}
