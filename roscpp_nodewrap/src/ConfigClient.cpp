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

ConfigClient::ConfigClient(const ConfigClientOptions& options, const
    NodeImplPtr& nodeImpl) :
  impl(new Impl(options, nodeImpl)) {
}

ConfigClient::~ConfigClient() {
}

ConfigClient::Impl::Impl(const ConfigClientOptions& options, const
    NodeImplPtr& nodeImpl) :
  service(options.service),
  nodeImpl(nodeImpl) {
  ros::ServiceClientOptions listParamsOptions;
  listParamsOptions.init<ListParams::Request, ListParams::Response>(
    ros::names::append(options.service, "list_params"), options.persistent,
    options.header);
  listParamsClient = nodeImpl->getNodeHandle().serviceClient(
    listParamsOptions);
    
  ros::ServiceClientOptions hasParamOptions;
  hasParamOptions.init<HasParam::Request, HasParam::Response>(
    ros::names::append(options.service, "has_param"), options.persistent,
    options.header);
  hasParamClient = nodeImpl->getNodeHandle().serviceClient(hasParamOptions);
  
  ros::ServiceClientOptions findParamOptions;
  findParamOptions.init<FindParam::Request, FindParam::Response>(
    ros::names::append(options.service, "find_param"), options.persistent,
    options.header);
  findParamClient = nodeImpl->getNodeHandle().serviceClient(findParamOptions);
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

std::vector<std::string> ConfigClient::getParamKeys(ros::Duration timeout) {
  if (impl && impl->listParamsClient.waitForExistence(timeout))
    return impl->getParamKeys();
  else
    return std::vector<std::string>();
}

std::string ConfigClient::getParamService(const std::string& key,
    ros::Duration timeout) {
  if (impl && impl->findParamClient.waitForExistence(timeout))
    return impl->getParamService(key);
  else
    return std::string();
}

bool ConfigClient::hasParam(const std::string& key, ros::Duration timeout) {
  if (impl && impl->hasParamClient.waitForExistence(timeout))
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
  if (impl && waitForExistence(timeout))
    return impl->paramClient(key, options);
  else
    return ParamClient();
}

ParamClient ConfigClient::Impl::paramClient(const std::string& key,
    const ParamClientOptions& options) {
  ParamClient paramClient;
  std::string paramService = getParamService(key);
    
  if (!paramService.empty()) {
    ParamClientOptions resolvedOptions = options;
    resolvedOptions.service = paramService;
    
    paramClient = ParamClient(resolvedOptions, nodeImpl);
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
