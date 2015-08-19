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

#include "roscpp_nodewrap/diagnostics/FrequencyTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

FrequencyTask::FrequencyTask() {
}

FrequencyTask::FrequencyTask(const FrequencyTask& src) :
  CyclicEventTask<FrequencyStatistics>(src) {
}

FrequencyTask::~FrequencyTask() {
}

FrequencyTask::Impl::Impl(const Options& defaultOptions, const std::string&
    name, const ManagerImplPtr& manager) :
  CyclicEventTask<FrequencyStatistics>::Impl(defaultOptions, name, manager) {
  std::string ns = defaultOptions.ns.empty() ? 
    ros::names::append("diagnostics", name) : defaultOptions.ns;
    
  ros::Duration windowDuration = ros::Duration(
    getNode()->getParam(ros::names::append(ns, "window_duration"),
    defaultOptions.windowDuration.toSec()));
  
  if (expected < std::numeric_limits<double>::infinity()) {
    windowSize = windowDuration.toSec()*expected;
    statistics.setRollingWindowSize(windowSize);
  }
}
    
FrequencyTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

ros::Time FrequencyTask::Impl::getExpectedTimeOfNextEvent() const {
  if ((expected > 0.0) && !statistics.getTimeOfLastEvent().isZero())
    return statistics.getTimeOfLastEvent()+ros::Duration(1.0/expected);
  else
    return ros::Time();
}

}
