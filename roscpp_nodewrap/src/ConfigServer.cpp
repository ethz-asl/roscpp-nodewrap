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

#include <ros/xmlrpc_manager.h>

#include <roscpp_nodewrap/Exceptions.h>
#include <roscpp_nodewrap/NodeImpl.h>

#include "roscpp_nodewrap/ConfigServer.h"

/*****************************************************************************/
/* External declarations                                                     */
/*****************************************************************************/

namespace ros {
  namespace param {
    extern void paramUpdateCallback(XmlRpc::XmlRpcValue& params, 
      XmlRpc::XmlRpcValue& result);
  }
}

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ConfigServer::ConfigServer() {
}

ConfigServer::ConfigServer(const ConfigServer& src) :
  impl(src.impl) {
}

ConfigServer::ConfigServer(const AdvertiseConfigOptions& options, const
    NodeImplPtr& nodeImpl) :
  impl(new Impl(options, nodeImpl)) {
}

ConfigServer::~ConfigServer() {
}

ConfigServer::Impl::Impl(const AdvertiseConfigOptions& options, const
    NodeImplPtr& nodeImpl) :
  service(options.service),
  nodeImpl(nodeImpl) {
  ros::XMLRPCManager::instance()->unbind("paramUpdate");
  ros::XMLRPCManager::instance()->bind("paramUpdate",
    boost::bind(&ConfigServer::Impl::updateParamCallback, this, _1, _2));
  
  ros::AdvertiseServiceOptions listParamsOptions;    
  listParamsOptions.init<ListParams::Request, ListParams::Response>(
    ros::names::append(options.service, "list_params"),
    boost::bind(&ConfigServer::Impl::listParamsCallback, this, _1, _2));
  listParamsOptions.callback_queue = options.callbackQueue;
  listParamsOptions.tracked_object = options.trackedObject;
  listParamsServer = nodeImpl->getNodeHandle().advertiseService(
    listParamsOptions);
  
  ros::AdvertiseServiceOptions hasParamOptions;    
  hasParamOptions.init<HasParam::Request, HasParam::Response>(
    ros::names::append(options.service, "has_param"),
    boost::bind(&ConfigServer::Impl::hasParamCallback, this, _1, _2));
  hasParamOptions.callback_queue = options.callbackQueue;
  hasParamOptions.tracked_object = options.trackedObject;
  hasParamServer = nodeImpl->getNodeHandle().advertiseService(
    hasParamOptions);
  
  ros::AdvertiseServiceOptions findParamOptions;    
  findParamOptions.init<FindParam::Request, FindParam::Response>(
    ros::names::append(options.service, "find_param"),
    boost::bind(&ConfigServer::Impl::findParamCallback, this, _1, _2));
  findParamOptions.callback_queue = options.callbackQueue;
  findParamOptions.tracked_object = options.trackedObject;
  findParamServer = nodeImpl->getNodeHandle().advertiseService(
    findParamOptions);
}

ConfigServer::Impl::~Impl() {
  unadvertise();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

bool ConfigServer::Impl::isValid() const {
  return listParamsServer && hasParamServer && findParamServer;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ConfigServer::shutdown() {
  if (impl)
    impl->unadvertise();
}

ParamServer ConfigServer::advertiseParam(const std::string& key, const
    AdvertiseParamOptions& options) {
  if (impl)
    return impl->advertiseParam(key, options);
  else
    return ParamServer();
}

void ConfigServer::Impl::unadvertise() {
  if (isValid()) {
    for (std::map<std::string, ParamServer::ImplWPtr>::iterator it =
        params.begin(); it != params.end(); ++it) {
      ParamServer::ImplPtr param = it->second.lock();
    
      if (param)
        param->unadvertise();
    }
    
    listParamsServer.shutdown();
    hasParamServer.shutdown();
    findParamServer.shutdown();
  }
}

ParamServer ConfigServer::Impl::advertiseParam(const std::string& key, const
    AdvertiseParamOptions& options) {
  boost::mutex::scoped_lock lock(mutex);
  
  if (key.empty())
    throw InvalidParamKeyException(key, "Parameter keys may not be empty");
  
  for (size_t i = 0; i < key.size(); ++i) {
    if (!isalnum(key[i]) && (key[i] != '_') && (key[i] != '/')) {
      std::stringstream stream;
      stream << "Character [" << key[i] << "] at element [" <<
        i << "] is not valid";
      throw InvalidParamKeyException(key, stream.str());
    }
  }
  
  ParamServer paramServer;
  std::map<std::string, ParamServer::ImplWPtr>::iterator it = params.find(key);
    
  if (it == params.end()) {
    paramServer = options.helper->createServer(options, nodeImpl);
    params.insert(std::make_pair(key, paramServer.impl));
  }
  else
    paramServer.impl = it->second.lock();
  
  return paramServer;
}

void ConfigServer::Impl::updateParamCallback(XmlRpc::XmlRpcValue& params,
    XmlRpc::XmlRpcValue& result) {
}

bool ConfigServer::Impl::listParamsCallback(ListParams::Request& request, 
    ListParams::Response& response) {
  boost::mutex::scoped_lock lock(mutex);
  response.keys.reserve(params.size());
  
  for (std::map<std::string, ParamServer::ImplWPtr>::iterator it =
      params.begin(); it != params.end(); ++it) {
    if (it->second.lock())
      response.keys.push_back(it->first);
  }
  
  return true;
}

bool ConfigServer::Impl::hasParamCallback(HasParam::Request& request,
    HasParam::Response& response) {
  boost::mutex::scoped_lock lock(mutex);

  std::map<std::string, ParamServer::ImplWPtr>::iterator it =
    params.find(request.key);
  response.exists = (it != params.end()) && it->second.lock();
  
  return true;
}

bool ConfigServer::Impl::findParamCallback(FindParam::Request& request,
    FindParam::Response& response) {
  boost::mutex::scoped_lock lock(mutex);

  std::map<std::string, ParamServer::ImplWPtr>::iterator it =
    params.find(request.key);
  if (it != params.end()) {
    ParamServer::ImplPtr impl = it->second.lock();
    if (impl) {
      response.service = nodeImpl->getNodeHandle().resolveName(
        impl->service);
      return true;
    }
  }
  
  return false;
}

}
