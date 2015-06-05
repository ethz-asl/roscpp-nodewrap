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

#include "roscpp_nodewrap/diagnostics/DiagnosticTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DiagnosticTask::DiagnosticTask() {
}

DiagnosticTask::DiagnosticTask(const DiagnosticTask& src) :
  impl(src.impl) {
}

DiagnosticTask::DiagnosticTask(const ImplPtr& impl) :
  impl(impl) {
}

DiagnosticTask::~DiagnosticTask() {  
}

DiagnosticTask::Impl::Impl(const std::string& name, const NodeImplPtr&
    nodeImpl) :
  diagnostic_updater::DiagnosticTask(name),
  nodeImpl(nodeImpl),
  valid(true) {
  nodeImpl->diagnosticUpdater.impl->add(*this);
}

DiagnosticTask::Impl::~Impl() {
  remove();
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string DiagnosticTask::getName() const {
  if (impl)
    return impl->getName();
  else
    return std::string();
}

bool DiagnosticTask::Impl::isValid() const {
  return valid;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DiagnosticTask::shutdown() {
  if (impl)
    impl->remove();
}

void DiagnosticTask::Impl::remove() {
  if (valid) {
    nodeImpl->diagnosticUpdater.impl->removeByName(getName());
    valid = false;
  }
}

}
