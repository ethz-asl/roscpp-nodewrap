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

#include "roscpp_nodewrap/diagnostics/DiagnosticUpdater.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

DiagnosticUpdater::DiagnosticUpdater() {
}

DiagnosticUpdater::DiagnosticUpdater(const DiagnosticUpdater& src) :
  impl(src.impl) {
}

DiagnosticUpdater::DiagnosticUpdater(const NodeImplPtr& nodeImpl) :
  impl(new Impl(nodeImpl)) {
}

DiagnosticUpdater::~DiagnosticUpdater() {  
}

DiagnosticUpdater::Impl::Impl(const NodeImplPtr& nodeImpl) :
  diagnostic_updater::Updater(ros::NodeHandle(), nodeImpl->getNodeHandle(),
    nodeImpl->getName()),
  nodeImpl(nodeImpl) {
  std::string ns = "diagnostics";
  
  double period = nodeImpl->getParam(ros::names::append(ns, "period"),
    getPeriod());
  setHardwareID("none");
    
  ros::TimerOptions timerOptions;
  timerOptions.period = ros::Duration(period);
  timerOptions.oneshot = false;
  timerOptions.autostart = true;
  timerOptions.callback = boost::bind(&DiagnosticUpdater::Impl::timerCallback,
    this, _1);
  
  timer = nodeImpl->getNodeHandle().createTimer(timerOptions);
}
    
DiagnosticUpdater::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void DiagnosticUpdater::setHardwareId(const std::string& hardwareId) {
  if (impl)
    impl->setHardwareId(hardwareId);
}

void DiagnosticUpdater::Impl::setHardwareId(const std::string& hardwareId) {
  setHardwareID(hardwareId);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DiagnosticUpdater::shutdown() {
  if (impl)
    impl->shutdown();
}

void DiagnosticUpdater::Impl::shutdown() {
  boost::mutex::scoped_lock lock(mutex);
  
  for (std::map<std::string, DiagnosticTask::ImplWPtr>::iterator it =
      tasks.begin(); it != tasks.end(); ++it) {
    DiagnosticTask::ImplPtr task = it->second.lock();
  
    if (task)
      task->remove();
  }
}

void DiagnosticUpdater::Impl::timerCallback(const ros::TimerEvent&
    timerEvent) {
  force_update();
}

}
