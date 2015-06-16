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

#include "roscpp_nodewrap/diagnostics/CompositeTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

CompositeTask::CompositeTask() {
}

CompositeTask::CompositeTask(const CompositeTask& src) :
  DiagnosticTask(src),
  DiagnosticTaskManager(src) {
}

CompositeTask::~CompositeTask() {
}
    
CompositeTask::Impl::Impl(const Options& defaultOptions, const std::string&
    name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager) {
}
    
CompositeTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const NodeImplPtr& CompositeTask::Impl::getNode() const {
  return manager->getNode();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void CompositeTask::shutdown() {
  DiagnosticTask::shutdown();
  DiagnosticTaskManager::shutdown();
}

void CompositeTask::resetImpl(Impl* impl) {
  DiagnosticTask::impl.reset(impl);
  DiagnosticTaskManager::impl.reset(impl);
}

void CompositeTask::Impl::startTask(diagnostic_updater::DiagnosticTask&
    task) {
  boost::mutex::scoped_lock lock(taskMutex);
  
  tasks.push_back(task.getName());
}

void CompositeTask::Impl::stopTask(const std::string& name) {
  boost::mutex::scoped_lock lock(taskMutex);
  
  tasks.remove(name);
}

void CompositeTask::Impl::run(diagnostic_updater::DiagnosticStatusWrapper&
    status) {
  diagnostic_updater::DiagnosticStatusWrapper originalStatus;
  diagnostic_updater::DiagnosticStatusWrapper combinedStatus;
  
  originalStatus.summary(status);

  boost::mutex::scoped_lock lock(taskMutex);
  
  for (std::list<std::string>::iterator it = tasks.begin();
       it != tasks.end(); ++it) {
    boost::mutex::scoped_lock lock(mutex);
  
    std::map<std::string, DiagnosticTask::ImplWPtr>::iterator jt =
      instances.find(*it);
    DiagnosticTask::ImplPtr taskImpl = jt->second.lock();
  
    if (taskImpl) {
      status.summary(originalStatus);
      
      taskImpl->as<DiagnosticTask::Impl>().run(status);
      
      combinedStatus.mergeSummary(status);
    }
  }
  
  status.summary(combinedStatus);
}

}
