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

#include "roscpp_nodewrap/diagnostics/DiagnosticTaskManager.h"

#include "roscpp_nodewrap/diagnostics/DiagnosticTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DiagnosticTask::DiagnosticTask() {
}

DiagnosticTask::DiagnosticTask(const DiagnosticTask& src) :
  Managed<DiagnosticTask, std::string>(src) {
}

DiagnosticTask::~DiagnosticTask() {  
}

DiagnosticTask::Impl::Impl(const std::string& name, const ManagerImplPtr&
    manager) :
  Managed<nodewrap::DiagnosticTask, std::string>::Impl(name, manager),
  task(name, boost::bind(&DiagnosticTask::Impl::run, this, _1)),
  started(false) {
}

DiagnosticTask::Impl::~Impl() {
  shutdown();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string DiagnosticTask::getName() const {
  return getIdentifier();
}

bool DiagnosticTask::Impl::isValid() const {
  return true;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DiagnosticTask::start() {
  if (impl)
    impl->as<DiagnosticTask::Impl>().start();
}

void DiagnosticTask::stop() {
  if (impl)
    impl->as<DiagnosticTask::Impl>().stop();
}

void DiagnosticTask::resetImpl(Impl* impl) {
  this->impl.reset(impl);
}

void DiagnosticTask::Impl::start() {
  if (!started) {
    manager->as<DiagnosticTaskManager::Impl>().startTask(task);
    
    started = true;
  }
}

void DiagnosticTask::Impl::stop() {
  if (started) {
    started = false;
    
    manager->as<DiagnosticTaskManager::Impl>().stopTask(task.getName());
  }
}

void DiagnosticTask::Impl::shutdown() {
  stop();
}

}
