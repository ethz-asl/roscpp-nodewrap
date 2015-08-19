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

#include "roscpp_nodewrap/diagnostics/ServiceClientStatusTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ServiceClientStatusTask::ServiceClientStatusTask() {
}

ServiceClientStatusTask::ServiceClientStatusTask(const
    ServiceClientStatusTask& src) :
  DiagnosticTask(src) {
}

ServiceClientStatusTask::~ServiceClientStatusTask() {
}
    
ServiceClientStatusTask::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager) {
}
    
ServiceClientStatusTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ServiceClientStatusTask::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  ServiceClient::ImplPtr serviceClientImpl =
    boost::static_pointer_cast<ServiceClient::Impl>(serviceClient.lock());
    
  if (serviceClientImpl && serviceClientImpl->isValid()) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Service client is valid.");
    
    if (serviceClientImpl->serviceClient.exists())
      status.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK,
        "Service exists.");
    else
      status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Service does not exist.");
    
    status.addf("Service connecting to", "%s",
      serviceClientImpl->serviceClient.getService().c_str());
    status.addf("Persistent service connection", "%s",
      serviceClientImpl->serviceClient.isPersistent() ? "yes" : "no");
    status.addf("Service type", "%s", serviceClientImpl->serviceType.c_str());
    status.addf("Service request type", "%s",
      serviceClientImpl->serviceRequestType.c_str());
    status.addf("Service response type", "%s",
      serviceClientImpl->serviceResponseType.c_str());
    status.addf("Service MD5 sum", "%s",
      serviceClientImpl->serviceMd5Sum.c_str());
    status.addf("Number of requests served", "%d",
      serviceClientImpl->numServedRequests);
    status.addf("Number of requests failed", "%d",
      serviceClientImpl->numFailedRequests);
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Service client is invalid.");
}

}
