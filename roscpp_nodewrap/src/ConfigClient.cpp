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

#include <roscpp_nodewrap/NodeImpl.h>

#include "roscpp_nodewrap/ConfigClient.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ConfigClient::ConfigClient() {
}

ConfigClient::ConfigClient(const ConfigClient& src) :
  impl(src.impl) {
}

ConfigClient::ConfigClient(const std::string& service, const NodeImplPtr&
    nodeImpl) :
  impl(new Impl(service ,nodeImpl)) {
}

ConfigClient::~ConfigClient() {
}

ConfigClient::Impl::Impl(const std::string& service, const NodeImplPtr&
    nodeImpl) :
  nodeImpl(nodeImpl) {
  listParamsClient = nodeImpl->getNodeHandle().serviceClient<
    ListParams::Request, ListParams::Response>(
    ros::names::append(service, "list_params"));
  
  hasParamClient = nodeImpl->getNodeHandle().serviceClient<
    HasParam::Request, HasParam::Response>(
    ros::names::append(service, "has_param"));
  
  findParamClient = nodeImpl->getNodeHandle().serviceClient<
    FindParam::Request, FindParam::Response>(
    ros::names::append(service, "find_param"));
}

ConfigClient::Impl::~Impl() {
  disconnect();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ConfigClient::getService() const {
  if (impl)
    return impl->service;
  else
    return std::string();
}

std::vector<std::string> ConfigClient::getParamKeys() {
  if (impl)
    return impl->getParamKeys();
  else
    return std::vector<std::string>();
}

std::string ConfigClient::getParamService(const std::string& key) {
  if (impl)
    return impl->getParamService(key);
  else
    return std::string();
}

bool ConfigClient::hasParam(const std::string& key) {
  if (impl)
    return impl->hasParam(key);
  else
    return false;
}

bool ConfigClient::isValid() const {
  if (impl)
    return impl->listParamsClient.isValid() &&
      impl->hasParamClient.isValid() &&
      impl->findParamClient.isValid();
  else
    return false;
}

bool ConfigClient::isPersistent() const {
  if (impl)
    return impl->listParamsClient.isPersistent() &&
      impl->hasParamClient.isPersistent() &&
      impl->findParamClient.isPersistent();
  else
    return false;
}

bool ConfigClient::exists() const {
  if (impl)
    return impl->listParamsClient.exists() &&
      impl->hasParamClient.exists() &&
      impl->findParamClient.exists();
  else
    return false;
}

std::vector<std::string> ConfigClient::Impl::getParamKeys() {
  ListParams listParamsService;
  
  if (listParamsClient.call(listParamsService))
    return listParamsService.response.keys;
  else
    return std::vector<std::string>();
}

std::string ConfigClient::Impl::getParamService(const std::string& key) {
  FindParam findParamService;
  findParamService.request.key = key;
  
  if (findParamClient.call(findParamService))
    return findParamService.response.service;
  else
    return std::string();
}

bool ConfigClient::Impl::hasParam(const std::string& key) {
  HasParam hasParamService;
  hasParamService.request.key = key;
  
  if (hasParamClient.call(hasParamService))
    return hasParamService.response.exists;
  else
    return false;
}

bool ConfigClient::Impl::isValid() const {
  return listParamsClient.isValid() &&
    hasParamClient.isValid() &&
    findParamClient.isValid();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool ConfigClient::waitForExistence(ros::Duration timeout) {
  if (impl)
    return impl->listParamsClient.waitForExistence(timeout) &&
      impl->hasParamClient.waitForExistence(timeout) &&
      impl->findParamClient.waitForExistence(timeout);
  else
    return false;
}

void ConfigClient::shutdown() {
  if (impl)
    impl->disconnect();
}

ParamClient ConfigClient::paramClient(const std::string& key, const
    ParamClientOptions& options, ros::Duration timeout) {
  ParamClient paramClient;
  
  if (waitForExistence(timeout)) {
    std::string paramService = getParamService(key);
    
    if (!paramService.empty()) {
      ParamClientOptions paramClientOptions = options;
      paramClientOptions.service = paramService;
      
      paramClient = ParamClient(options, impl->nodeImpl);
    }
  }
  
  return paramClient;
}

void ConfigClient::Impl::disconnect() {
  if (isValid()) {
    listParamsClient.shutdown();
    hasParamClient.shutdown();
    findParamClient.shutdown();
  }
}

}
