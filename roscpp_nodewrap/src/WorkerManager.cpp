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

#include "roscpp_nodewrap/Exceptions.h"
#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/WorkerManager.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

WorkerManager::WorkerManager() {
}

WorkerManager::WorkerManager(const WorkerManager& src) :
  impl(src.impl) {
}

WorkerManager::WorkerManager(const NodeImplPtr& nodeImpl) :
  impl(new Impl(nodeImpl)) {
}

WorkerManager::~WorkerManager() {  
}

WorkerManager::Impl::Impl(const NodeImplPtr& nodeImpl) :
  nodeImpl(nodeImpl) {
  ros::AdvertiseServiceOptions listWorkersOptions;
  listWorkersOptions.init<ListWorkers::Request, ListWorkers::Response>(
    "list_workers", boost::bind(&WorkerManager::Impl::listWorkersCallback,
    this, _1, _2));
  listWorkersServer = nodeImpl->getNodeHandle().advertiseService(
    listWorkersOptions);
  
  ros::AdvertiseServiceOptions hasWorkerOptions;
  hasWorkerOptions.init<HasWorker::Request, HasWorker::Response>(
    "has_worker", boost::bind(&WorkerManager::Impl::hasWorkerCallback,
    this, _1, _2));
  hasWorkerServer = nodeImpl->getNodeHandle().advertiseService(
    hasWorkerOptions);
}

WorkerManager::Impl::~Impl() {
  unadvertise();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

bool WorkerManager::Impl::isValid() const {
  return listWorkersServer && hasWorkerServer;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void WorkerManager::shutdown() {
  if (impl)
    impl->unadvertise();
}

Worker WorkerManager::addWorker(const std::string& name, const WorkerOptions&
    defaultOptions) {
  boost::mutex::scoped_lock lock(impl->mutex);
  
  if (name.empty())
    throw InvalidWorkerNameException(name, "Worker name may not be empty");
  
  for (size_t i = 0; i < name.size(); ++i) {
    if (!isalnum(name[i]) && (name[i] != '_') && (name[i] != '/')) {
      std::stringstream stream;
      stream << "Character [" << name[i] << "] at element [" << i <<
        "] is not valid";
      throw InvalidWorkerNameException(name, stream.str());
    }
  }
  
  Worker worker;
  std::map<std::string, Worker::ImplWPtr>::iterator it =
    impl->workers.find(name);
    
  if (it == impl->workers.end()) {
    worker.impl.reset(new Worker::Impl(name, defaultOptions, impl->nodeImpl));
    impl->workers.insert(std::make_pair(name, worker.impl));
  }
  else
    worker.impl = it->second.lock();
  
  return worker;
}

void WorkerManager::Impl::unadvertise() {
  if (isValid()) {
    for (std::map<std::string, Worker::ImplWPtr>::iterator it =
        workers.begin(); it != workers.end(); ++it) {
      Worker::ImplPtr worker = it->second.lock();
    
      if (worker) {
        worker->cancel(true);
        worker->unadvertise();
      }
    }
    
    listWorkersServer.shutdown();
    hasWorkerServer.shutdown();
  }
}

bool WorkerManager::Impl::listWorkersCallback(ListWorkers::Request& request,
    ListWorkers::Response& response) {
  boost::mutex::scoped_lock lock(mutex);
  response.names.reserve(workers.size());
  
  for (std::map<std::string, Worker::ImplWPtr>::iterator it =
      workers.begin(); it != workers.end(); ++it) {
    if (it->second.lock())
      response.names.push_back(it->first);
  }
  
  return true;
}

bool WorkerManager::Impl::hasWorkerCallback(HasWorker::Request& request,
    HasWorker::Response& response) {
  boost::mutex::scoped_lock lock(mutex);

  std::map<std::string, Worker::ImplWPtr>::iterator it =
    workers.find(request.name);
  response.result = (it != workers.end()) && it->second.lock();
  
  return true;
}

}
