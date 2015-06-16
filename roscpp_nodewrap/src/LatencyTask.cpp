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

#include <roscpp_nodewrap/NodeImpl.h>

#include "roscpp_nodewrap/diagnostics/LatencyTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

LatencyTask::LatencyTask() {
}

LatencyTask::LatencyTask(const LatencyTask& src) :
  StatisticsTask<LatencyStatistics>(src) {
}

LatencyTask::~LatencyTask() {
}

LatencyTask::Impl::Impl(const Options& defaultOptions, const std::string&
    name, const ManagerImplPtr& manager) :
  StatisticsTask<LatencyStatistics>::Impl(defaultOptions, name, manager) {
}
    
LatencyTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void LatencyTask::Impl::statusSummary(
    diagnostic_updater::DiagnosticStatusWrapper& status,
    const LatencyStatistics::Estimates& estimates) {
  if (estimates.numRollingSamples)
    StatisticsTask<LatencyStatistics>::Impl::statusSummary(status, estimates);
  else if (windowSize)
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "No events in window.");
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "No events expected.");
}

void LatencyTask::Impl::statusAdd(
    diagnostic_updater::DiagnosticStatusWrapper& status, const
    LatencyStatistics::Estimates& estimates) {
  const std::pair<ros::Time, ros::Time>& timesOfLastEvents =
    statistics.getTimesOfLastEvents();
    
  if (!timesOfLastEvents.first.isZero() &&
      !timesOfLastEvents.second.isZero()) {
    ros::Time now = ros::Time::now();
    
    status.addf("Time since last reference event", "%f s",
      (now-timesOfLastEvents.first).toSec());
    status.addf("Time since last referent event", "%f s",
      (now-timesOfLastEvents.second).toSec());
    
    status.addf("Latency between last events", "%f s",
      (timesOfLastEvents.second-timesOfLastEvents.first).toSec());
    
    StatisticsTask<LatencyStatistics>::Impl::statusAdd(status, estimates);
  }
}

}
