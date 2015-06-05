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

#include "roscpp_nodewrap/diagnostics/FunctionTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

FunctionTask::FunctionTask() {
}

FunctionTask::FunctionTask(const FunctionTask& src) {
}

FunctionTask::FunctionTask(const std::string& name, const Options&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  impl(new Impl(name, defaultOptions, nodeImpl)),
  DiagnosticTask(impl) {
}
    
FunctionTask::~FunctionTask() {
}
    
FunctionTask::Impl::Impl(const std::string& name, const Options&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  nodewrap::DiagnosticTask::Impl(name, nodeImpl),
  callback(defaultOptions.callback) {
}
    
FunctionTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void FunctionTask::Impl::run(diagnostic_updater::DiagnosticStatusWrapper&
    status) {
  callback(status);
}

}
