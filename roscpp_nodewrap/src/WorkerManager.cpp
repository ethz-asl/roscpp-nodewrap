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

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/worker/AsyncWorker.h"
#include "roscpp_nodewrap/worker/SyncWorker.h"
#include "roscpp_nodewrap/worker/WorkerExceptions.h"

#include "roscpp_nodewrap/worker/WorkerManager.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

WorkerManager::WorkerManager() {
}

WorkerManager::WorkerManager(const WorkerManager& src) :
  Manager<Worker, std::string>(src) {
}

WorkerManager::WorkerManager(const NodeImplPtr& node) {
  impl.reset(new Impl(node));
}

WorkerManager::~WorkerManager() {  
}

WorkerManager::Impl::Impl(const NodeImplPtr& node) :
  node(node) {
  ros::AdvertiseServiceOptions listWorkersOptions;
  listWorkersOptions.init<ListWorkers::Request, ListWorkers::Response>(
    "list_workers", boost::bind(&WorkerManager::Impl::listWorkersCallback,
    this, _1, _2));
  listWorkersServer = node->advertiseService(
    ros::names::append("worker_manager", "list_workers"),
    listWorkersOptions);
  
  ros::AdvertiseServiceOptions hasWorkerOptions;
  hasWorkerOptions.init<HasWorker::Request, HasWorker::Response>(
    "has_worker", boost::bind(&WorkerManager::Impl::hasWorkerCallback,
    this, _1, _2));
  hasWorkerServer = node->advertiseService(
    ros::names::append("worker_manager", "has_worker"),
    hasWorkerOptions);
}

WorkerManager::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const NodeImplPtr& WorkerManager::Impl::getNode() const {
  return node;
}

bool WorkerManager::Impl::isValid() const {
  return listWorkersServer && hasWorkerServer;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

Worker WorkerManager::addWorker(const std::string& name, const WorkerOptions&
    defaultOptions) {
  Worker worker;
  
  if (impl) {
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
    
    std::map<std::string, Worker::ImplWPtr>::iterator it =
      impl->instances.find(name);
      
    if (it == impl->instances.end()) {
      if (defaultOptions.synchronous)
        worker.impl.reset(new SyncWorker::Impl(name, impl));
      else
        worker.impl.reset(new AsyncWorker::Impl(name, impl));
      
      impl->instances.insert(std::make_pair(name, worker.impl));
      
      worker.impl->as<Worker::Impl>().init(defaultOptions);
      if (worker.impl->as<Worker::Impl>().autostart)
        worker.impl->as<Worker::Impl>().start();
    }
    else
      worker.impl = it->second.lock();
  }
  
  return worker;
}

void WorkerManager::Impl::shutdown() {
  boost::mutex::scoped_lock lock(mutex);
  
  if (isValid()) {
    for (std::map<std::string, Worker::ImplWPtr>::iterator it =
        instances.begin(); it != instances.end(); ++it) {
      Worker::ImplPtr worker = it->second.lock();
    
      if (worker)
        worker->shutdown();
    }
    
    listWorkersServer.shutdown();
    hasWorkerServer.shutdown();
  }
}

bool WorkerManager::Impl::listWorkersCallback(ListWorkers::Request& request,
    ListWorkers::Response& response) {
  boost::mutex::scoped_lock lock(mutex);
  response.names.reserve(instances.size());
  
  for (std::map<std::string, Worker::ImplWPtr>::iterator it =
      instances.begin(); it != instances.end(); ++it) {
    if (it->second.lock())
      response.names.push_back(it->first);
  }
  
  return true;
}

bool WorkerManager::Impl::hasWorkerCallback(HasWorker::Request& request,
    HasWorker::Response& response) {
  boost::mutex::scoped_lock lock(mutex);

  std::map<std::string, Worker::ImplWPtr>::iterator it =
    instances.find(request.name);
  response.result = (it != instances.end()) && it->second.lock();
  
  return true;
}

}
