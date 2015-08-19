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
  DiagnosticTaskManager(src) {
}

DiagnosticUpdater::DiagnosticUpdater(const NodeImplPtr& node) {
  impl.reset(new Impl(node));
}

DiagnosticUpdater::~DiagnosticUpdater() {  
}

DiagnosticUpdater::Impl::Impl(const NodeImplPtr& node) :
  updater(ros::NodeHandle(), node->getNodeHandle(), node->getName()),
  node(node) {
  std::string ns = "diagnostics";
  
  double period = node->getParam(ros::names::append(ns, "period"),
    updater.getPeriod());
  updater.setHardwareID("none");
    
  ros::TimerOptions timerOptions;
  timerOptions.period = ros::Duration(period);
  timerOptions.oneshot = false;
  timerOptions.autostart = true;
  timerOptions.callback = boost::bind(&DiagnosticUpdater::Impl::timerCallback,
    this, _1);
  
  timer = node->createTimer(timerOptions);
}
    
DiagnosticUpdater::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void DiagnosticUpdater::setHardwareId(const std::string& hardwareId) {
  if (impl)
    impl->as<DiagnosticUpdater::Impl>().setHardwareId(hardwareId);
}

const NodeImplPtr& DiagnosticUpdater::Impl::getNode() const {
  return node;
}

void DiagnosticUpdater::Impl::setHardwareId(const std::string& hardwareId) {
  updater.setHardwareID(hardwareId);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void DiagnosticUpdater::Impl::startTask(diagnostic_updater::DiagnosticTask&
    task) {
  updater.add(task);
}

void DiagnosticUpdater::Impl::stopTask(const std::string& name) {
  updater.removeByName(name);
}

void DiagnosticUpdater::Impl::timerCallback(const ros::TimerEvent&
    timerEvent) {
  updater.force_update();
}

}
