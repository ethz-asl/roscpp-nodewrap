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

#include <limits>

#include <roscpp_nodewrap/NodeImpl.h>

#include "roscpp_nodewrap/diagnostics/PeriodTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PeriodTask::PeriodTask() {
}

PeriodTask::PeriodTask(const PeriodTask& src) :
  CyclicEventTask<PeriodStatistics>(src) {
}

PeriodTask::~PeriodTask() {
}

PeriodTask::Impl::Impl(const Options& defaultOptions, const std::string&
    name, const ManagerImplPtr& manager) :
  CyclicEventTask<PeriodStatistics>::Impl(defaultOptions, name, manager) {
  std::string ns = defaultOptions.ns.empty() ? 
    ros::names::append("diagnostics", name) : defaultOptions.ns;
    
  ros::Duration windowDuration = ros::Duration(
    getNode()->getParam(ros::names::append(ns, "window_duration"),
    defaultOptions.windowDuration.toSec()));
  
  if (expected > 0.0) {
    windowSize = windowDuration.toSec()/expected;
    statistics.setRollingWindowSize(windowSize);
  }
}
    
PeriodTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

ros::Time PeriodTask::Impl::getExpectedTimeOfNextEvent() const {
  if ((expected < std::numeric_limits<double>::infinity()) &&
      !statistics.getTimeOfLastEvent().isZero())
    return statistics.getTimeOfLastEvent()+ros::Duration(expected);
  else
    return ros::Time();
}

}
