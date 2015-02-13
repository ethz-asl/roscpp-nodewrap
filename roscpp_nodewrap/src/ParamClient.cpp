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
#include <roscpp_nodewrap/ParamClientOptions.h>

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

ParamClient::ParamClient(const ParamClientOptions& options, const NodeImplPtr&
    nodeImpl) :
  impl(options.helper->createClient(options, nodeImpl).impl) {
}

ParamClient::~ParamClient() {
}

ParamClient::Impl::Impl(const ParamClientOptions& options, const NodeImplPtr&
    nodeImpl) :
  service(options.service),
  type(options.type),
  nodeImpl(nodeImpl) {
  ros::ServiceClientOptions getParamInfoOptions;
  getParamInfoOptions.init<GetParamInfo::Request, GetParamInfo::Response>(
    ros::names::append(options.service, "get_info"), options.persistent,
    options.header);
  getParamInfoClient = client(getParamInfoOptions);
}

ParamClient::Impl::~Impl() {
  disconnect();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ParamClient::getService() const {
  if (impl)
    return impl->service;
  else
    return std::string();
}

std::string ParamClient::getParamName(ros::Duration timeout) {
  if (impl && impl->getParamInfoClient.waitForExistence(timeout))
    return impl->getParamName();
  else
    return std::string();
}

bool ParamClient::isValid() const {
  if (impl)
    return impl->getParamValueClient.isValid() &&
      impl->setParamValueClient.isValid() &&
      impl->getParamInfoClient.isValid();
  else
    return false;
}

bool ParamClient::isPersistent() const {
  if (impl)
    return impl->getParamValueClient.isPersistent() &&
      impl->setParamValueClient.isPersistent() &&
      impl->getParamInfoClient.isPersistent();
  else
    return false;
}

bool ParamClient::exists() const {
  if (impl)
    return impl->getParamValueClient.exists() &&
      impl->setParamValueClient.exists() &&
      impl->getParamInfoClient.exists();
  else
    return false;
}

std::string ParamClient::Impl::getParamName() {
  GetParamInfo getParamInfoService;
  
  if (getParamInfoClient.call(getParamInfoService))
    return getParamInfoService.response.name;
  else
    return std::string();
}

bool ParamClient::Impl::isValid() const {
  return getParamValueClient.isValid() &&
    setParamValueClient.isValid() &&
    getParamInfoClient.isValid();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool ParamClient::waitForExistence(ros::Duration timeout) {
  if (impl)
    return impl->getParamValueClient.waitForExistence(timeout) &&
      impl->setParamValueClient.waitForExistence(timeout) &&
      impl->getParamInfoClient.waitForExistence(timeout);
  else
    return false;
}

void ParamClient::shutdown() {
  if (impl)
    impl->disconnect();
}

ros::ServiceClient ParamClient::Impl::client(const ros::ServiceClientOptions&
    options) {
  return nodeImpl->getNodeHandle().serviceClient(
    const_cast<ros::ServiceClientOptions&>(options));
}

void ParamClient::Impl::disconnect() {
  if (isValid()) {
    getParamValueClient.shutdown();
    setParamValueClient.shutdown();
    getParamInfoClient.shutdown();
  }
}

}
