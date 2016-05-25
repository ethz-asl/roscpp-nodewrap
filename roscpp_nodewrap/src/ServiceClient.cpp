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

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/ServiceClient.h"
#include <roscpp_nodewrap/ServiceClientOptions.h>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ServiceClient::ServiceClient() {
}

ServiceClient::ServiceClient(const ServiceClient& src) :
  impl(src.impl) {
}

ServiceClient::~ServiceClient() {  
}

ServiceClient::Impl::Impl(const std::string& name, const NodeImplPtr& node) :
  name(name),
  numServedRequests(0),
  numFailedRequests(0),
  node(node) {
}

ServiceClient::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Operators                                                                 */
/*****************************************************************************/

ServiceClient::operator ros::ServiceClient() const {
  if (impl)
    return impl->serviceClient;
  else
    return ros::ServiceClient();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ServiceClient::getService() const {
  if (impl)
    return impl->serviceClient.getService();
  else
    return std::string();
}

bool ServiceClient::isValid() const {
  if (impl)
    return impl->isValid();
  else
    return false;
}

bool ServiceClient::isPersistent() const {
  if (impl)
    return impl->serviceClient.isPersistent();
  else
    return false;
}

bool ServiceClient::exists() const {
  if (impl)
    return impl->serviceClient.exists();
  else
    return false;
}

const NodeImplPtr& ServiceClient::Impl::getNode() const {
  return node;
}

bool ServiceClient::Impl::isValid() const {
  return serviceClient && statusTask;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

bool ServiceClient::waitForExistence(const ros::Duration& timeout) {
  if (impl)
    return impl->serviceClient.waitForExistence(timeout);
  else
    return false;
}

void ServiceClient::shutdown() {
  if (impl)
    impl->shutdown();
}

void ServiceClient::Impl::init(const ServiceClientOptions& defaultOptions) {
  ros::ServiceClientOptions options(defaultOptions);
  std::string ns = defaultOptions.ns.empty() ?
    ros::names::append("clients", name) : defaultOptions.ns;
  
  options.service = node->getParam(ros::names::append(ns, "service"),
    defaultOptions.service);
  options.persistent = node->getParam(ros::names::append(ns, "persistent"),
    defaultOptions.persistent);
  serviceType = defaultOptions.datatype;
  serviceRequestType = defaultOptions.req_datatype;
  serviceResponseType = defaultOptions.res_datatype;
  serviceMd5Sum = defaultOptions.md5sum;
    
  serviceClient = node->getNodeHandle().serviceClient(options);
  
  ServiceClientStatusTaskOptions statusTaskOptions(
    defaultOptions.statusTaskOptions);
  statusTaskOptions.ns = ros::names::append(ns,
    ros::names::append("diagnostics", "service_client_status"));
  statusTask = node->addDiagnosticTask<ServiceClientStatusTask>(
    std::string("ServiceClient ")+name+" Status", statusTaskOptions);
  statusTask.impl->as<ServiceClientStatusTask::Impl>().serviceClient =
    shared_from_this();
}

void ServiceClient::Impl::shutdown() {
  if (isValid()) {
    serviceClient = ros::ServiceClient();
    
    statusTask.shutdown();
  }
}

}
