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

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include "roscpp_nodewrap/NodeImpl.h"

#include "roscpp_nodewrap/worker/SyncWorker.h"
#include "roscpp_nodewrap/worker/WorkerQueueCallback.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

SyncWorker::SyncWorker() {
}

SyncWorker::SyncWorker(const SyncWorker& src) :
  Worker(src),
  impl(src.impl) {
}

SyncWorker::SyncWorker(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  impl(new Impl(name, defaultOptions, nodeImpl)),
  Worker(impl) {
}

SyncWorker::~SyncWorker() {  
}

SyncWorker::Impl::Impl(const std::string& name, const WorkerOptions&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  Worker::Impl(name, defaultOptions, nodeImpl),
  callbackQueue(defaultOptions.callbackQueue),
  trackedObject(defaultOptions.trackedObject),
  hasTrackedObject(defaultOptions.trackedObject) {
}

SyncWorker::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void SyncWorker::Impl::safeStart() {
}

void SyncWorker::Impl::safeWake() {
  ros::CallbackInterfacePtr callback(new WorkerQueueCallback(
    boost::bind(&Worker::Impl::runOnce, this), trackedObject,
    hasTrackedObject));
  
  if (callbackQueue)
    callbackQueue->addCallback(callback);
  else
    nodeImpl->getNodeHandle().getCallbackQueue()->addCallback(callback);
}

void SyncWorker::Impl::safeStop() {
}

}
