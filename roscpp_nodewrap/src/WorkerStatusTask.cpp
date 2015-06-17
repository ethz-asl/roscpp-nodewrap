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

#include "roscpp_nodewrap/worker/AsyncWorker.h"
#include "roscpp_nodewrap/worker/SyncWorker.h"

#include "roscpp_nodewrap/diagnostics/WorkerStatusTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

WorkerStatusTask::WorkerStatusTask() {
}

WorkerStatusTask::WorkerStatusTask(const WorkerStatusTask& src) :
  DiagnosticTask(src) {
}

WorkerStatusTask::~WorkerStatusTask() {
}
    
WorkerStatusTask::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager) {
}
    
WorkerStatusTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void WorkerStatusTask::setWorker(const Worker& worker) {
  if (impl)
    impl->as<WorkerStatusTask::Impl>().worker =
      boost::static_pointer_cast<Worker::Impl>(worker.impl);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void WorkerStatusTask::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  Worker::ImplPtr workerImpl =
    boost::static_pointer_cast<Worker::Impl>(worker.lock());
    
  if (workerImpl && workerImpl->isValid()) {
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Worker is valid.");

    if (boost::dynamic_pointer_cast<SyncWorker::Impl>(workerImpl))
      status.add("Worker type", "synchronous");
    else if (boost::dynamic_pointer_cast<AsyncWorker::Impl>(workerImpl))
      status.add("Worker type", "asynchronous");
    else
      status.add("Worker type", "other");
      
    if (workerImpl->as<Worker::Impl>().started) {
      status.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK,
        "Worker has been started.");
      
      status.add("In started state", "yes");
      status.addf("In canceled state", "%s",
        workerImpl->as<Worker::Impl>().canceled ? "yes" : "no");
      
      status.addf("Expected frequency", "%f Hz",
        workerImpl->as<Worker::Impl>().expectedCycleTime.isZero() ?
        0.0: 1.0/workerImpl->as<Worker::Impl>().expectedCycleTime.toSec());
      status.addf("Started since", "%f s",
        workerImpl->as<Worker::Impl>().getTimeSinceStart().toSec());
      
      if (workerImpl->as<Worker::Impl>().hasPrivateCallbackQueue) {
        status.add("Private callback queue", "yes");
        
        if (workerImpl->as<Worker::Impl>().actualPriority ==
            workerImpl->as<Worker::Impl>().expectedPriority)
          status.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK,
            "Worker has expected priority.");
        else
          status.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
            "Worker priority does not meet the expected priority.");
        
        status.addf("Priority", "%d",
          workerImpl->as<Worker::Impl>().actualPriority);
      }
      else
        status.add("Private callback queue", "no");
    }
    else {
      status.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "Worker has not been started.");
      
      status.add("In started state", "no");
    }
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Worker is invalid.");
}

}
