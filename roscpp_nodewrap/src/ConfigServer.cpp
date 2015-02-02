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

#include <roscpp_nodewrap/NodeImpl.h>

#include "roscpp_nodewrap/ConfigServer.h"

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

ConfigServer::ConfigServer(const boost::shared_ptr<NodeImpl>& nodeImpl) :
  impl(new Impl(nodeImpl)) {
}

ConfigServer::Impl::Impl(const boost::shared_ptr<NodeImpl>& nodeImpl) :
  nodeImpl(nodeImpl) {
  ros::AdvertiseServiceOptions listParamsOptions;    
  listParamsOptions.init<ListParams::Request, ListParams::Response>(
    "list_params", boost::bind(&ConfigServer::Impl::listParams, this,
    _1, _2));
  listParamsServer = nodeImpl->getNodeHandle().advertiseService(
    listParamsOptions);
  
  ros::AdvertiseServiceOptions hasParamOptions;    
  hasParamOptions.init<HasParam::Request, HasParam::Response>(
    "has_param", boost::bind(&ConfigServer::Impl::hasParam, this,
    _1, _2));
  hasParamServer = nodeImpl->getNodeHandle().advertiseService(
    hasParamOptions);
  
  ros::XMLRPCManager::instance()->unbind("paramUpdate");
  ros::XMLRPCManager::instance()->bind("paramUpdate",
    boost::bind(&ConfigServer::Impl::updateParam, this, _1, _2));
}

ConfigServer::~ConfigServer() {
}

ConfigServer::Impl::~Impl() {
  unadvertise();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

bool ConfigServer::Impl::isValid() const {
  return listParamsServer.operator void*();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ConfigServer::shutdown() {
  if (impl)
    impl->unadvertise();
}

void ConfigServer::Impl::unadvertise() {
  if (isValid()) {
    listParamsServer.shutdown();
    hasParamServer.shutdown();
  }
}

bool ConfigServer::Impl::listParams(ListParams::Request& request, 
    ListParams::Response& response) {
  response.keys.reserve(nodeImpl->paramServers.size());

  for (std::map<std::string, ParamServer>::const_iterator it =
      nodeImpl->paramServers.begin(); it != nodeImpl->paramServers.end();
      ++it)
    response.keys.push_back(it->first);
  
  return true;
}

bool ConfigServer::Impl::hasParam(HasParam::Request& request,
    HasParam::Response& response) {
  response.result = nodeImpl->paramServers.count(request.key);
  return true;
}

void ConfigServer::Impl::updateParam(XmlRpc::XmlRpcValue& params,
    XmlRpc::XmlRpcValue& result) {
  ROS_INFO_STREAM("Parameter update for " << params[1]);
  ros::param::paramUpdateCallback(params, result);
}

}
