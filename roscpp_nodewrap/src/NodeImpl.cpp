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

#include <roscpp_nodewrap/Signal.h>

#include "roscpp_nodewrap/NodeImpl.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

NodeImpl::NodeImpl() :
  nodelet(false) {
}

NodeImpl::~NodeImpl() {  
  Signal::unbind(SIGINT, &NodeImpl::signaled, this);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::string& NodeImpl::getName() const {
  return name;
}

bool NodeImpl::isNodelet() const {
  return nodelet;
}

bool NodeImpl::isValid() const {
  return nodeHandle && nodeHandle->ok();
}

ros::NodeHandle& NodeImpl::getNodeHandle() const {
  return *nodeHandle;
}

void NodeImpl::setDiagnosticsHardwareId(const std::string& hardwareId) {
  if (!diagnosticUpdater)
    diagnosticUpdater = DiagnosticUpdater(shared_from_this());
  
  diagnosticUpdater.setHardwareId(hardwareId);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void NodeImpl::unload() {
  this->shutdown();
}

void NodeImpl::signaled(int signal) {
  this->shutdown();
}

Timer NodeImpl::createTimer(const ros::TimerOptions& options) {
  if (!timerManager)
    timerManager = TimerManager(shared_from_this());
  
  return timerManager.addTimer(options);
}

Publisher NodeImpl::advertise(const std::string& name, const PublisherOptions&
    defaultOptions) {
  Publisher publisher;
  
  publisher.impl.reset(new Publisher::Impl(name, shared_from_this()));
  publisher.impl->init(defaultOptions);
  
  return publisher;
}

Subscriber NodeImpl::subscribe(const std::string& name, const
    SubscriberOptions& defaultOptions) {
  Subscriber subscriber;
  
  subscriber.impl.reset(new Subscriber::Impl(name, shared_from_this()));
  subscriber.impl->init(defaultOptions);
  
  return subscriber;
}

ServiceServer NodeImpl::advertiseService(const std::string& name, const
    ServiceServerOptions& defaultOptions) {
  ServiceServer serviceServer;
  
  serviceServer.impl.reset(new ServiceServer::Impl(name, shared_from_this()));
  serviceServer.impl->init(defaultOptions);
  
  return serviceServer;
}

ServiceClient NodeImpl::serviceClient(const std::string& name, const
    ServiceClientOptions& defaultOptions) {
  ServiceClient serviceClient;
  
  serviceClient.impl.reset(new ServiceClient::Impl(name, shared_from_this()));
  serviceClient.impl->init(defaultOptions);
  
  return serviceClient;
}

Worker NodeImpl::addWorker(const std::string& name, const WorkerOptions&
    defaultOptions) {
  if (!workerManager)
    workerManager = WorkerManager(shared_from_this());
  
  return workerManager.addWorker(name, defaultOptions);
}

void NodeImpl::start(const std::string& name, bool nodelet, const
    ros::NodeHandlePtr& nodeHandle) {
  this->name = name;
  this->nodelet = nodelet;
  this->nodeHandle = nodeHandle;
  
  Signal::bind(SIGINT, &NodeImpl::signaled, this);

  init();
}

void NodeImpl::shutdown() {
  if (timerManager)
    timerManager.shutdown();
  if (diagnosticUpdater)
    diagnosticUpdater.shutdown();
  if (workerManager)
    workerManager.shutdown();
  
  cleanup();

  this->name.clear();
  this->nodelet = false;
  this->nodeHandle.reset();
}

}
