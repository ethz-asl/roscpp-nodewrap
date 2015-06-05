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

#include "roscpp_nodewrap/diagnostics/FrequencyTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

FrequencyTask::FrequencyTask() {
}

FrequencyTask::FrequencyTask(const FrequencyTask& src) {
}

FrequencyTask::FrequencyTask(const std::string& name, const Options&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  impl(new Impl(name, defaultOptions, nodeImpl)),
  DiagnosticTask(impl) {
}
    
FrequencyTask::~FrequencyTask() {
}
    
FrequencyTask::Impl::Impl(const std::string& name, const Options&
    defaultOptions, const NodeImplPtr& nodeImpl) :
  nodewrap::DiagnosticTask::Impl(name, nodeImpl),
  statistics(0),
  expected(0.0),
  warnMeanTolerance(0.05),
  errorMeanTolerance(0.1),
  warnStandardDeviationTolerance(0.05),
  errorStandardDeviationTolerance(0.1),
  started(false) {
  std::string ns = defaultOptions.ns.empty() ?
    ros::names::append("diagnostics/frequency", name) :
    defaultOptions.ns;
  
  window = ros::Duration(nodeImpl->getParam(
    ros::names::append(ns, "window"), defaultOptions.window.toSec()));
  
  warnMeanTolerance = nodeImpl->getParam(
    ros::names::append(ns, "mean_tolerance/warn"),
    defaultOptions.warnMeanTolerance);
  errorMeanTolerance = nodeImpl->getParam(
    ros::names::append(ns, "mean_tolerance/error"),
    defaultOptions.errorMeanTolerance);
  
  warnStandardDeviationTolerance = nodeImpl->getParam(
    ros::names::append(ns, "standard_deviation_tolerance/warn"),
    defaultOptions.warnStandardDeviationTolerance);
  errorStandardDeviationTolerance = nodeImpl->getParam(
    ros::names::append(ns, "standard_deviation_tolerance/error"),
    defaultOptions.errorStandardDeviationTolerance);
  
  expected = nodeImpl->getParam(
    ros::names::append(ns, "expected"), defaultOptions.expected);
  statistics.setRollingWindowSize(window.toSec()*expected);
}
    
FrequencyTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

FrequencyStatistics FrequencyTask::getStatistics() const {
  if (impl)
    return impl->getStatistics();
  else
    return FrequencyStatistics();
}

FrequencyStatistics::Estimates FrequencyTask::getStatisticsEstimates()
    const {
  if (impl)
    return impl->getStatisticsEstimates();
  else
    return FrequencyStatistics::Estimates();
}

FrequencyStatistics FrequencyTask::Impl::getStatistics() const {
  boost::mutex::scoped_lock lock(mutex);
  
  return statistics;
}

FrequencyStatistics::Estimates FrequencyTask::Impl::getStatisticsEstimates()
    const {
  boost::mutex::scoped_lock lock(mutex);
  
  return statistics.getEstimates();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void FrequencyTask::start() {
  if (impl)
    impl->start();
}

void FrequencyTask::stop() {
  if (impl)
    impl->stop();
}

void FrequencyTask::Impl::start() {
  boost::mutex::scoped_lock lock(mutex);

  if (!started)
    started = true;
}

void FrequencyTask::Impl::stop() {
  boost::mutex::scoped_lock lock(mutex);
  
  if (started) {
    started = false;
    statistics.clear();
  }
}

void FrequencyTask::Impl::run(diagnostic_updater::DiagnosticStatusWrapper&
    status) {
  boost::mutex::scoped_lock lock(mutex);
  
  if (started) {
    FrequencyStatistics rollingStatistics = statistics;
    ros::Time now = ros::Time::now();
    
    if ((expected > 0.0) &&
        (statistics.getTimeSinceLastEvent(now).toSec() > 1.0/expected))
      rollingStatistics.event(now);
    
    FrequencyStatistics::Estimates estimates = statistics.getEstimates();
    FrequencyStatistics::Estimates rollingEstimates =
      rollingStatistics.getEstimates();
    
    if (rollingEstimates.numRollingSamples) {
      double rollingStandardDeviation = sqrt(rollingEstimates.rollingVariance);
      
      double meanTolerance = fabs(rollingEstimates.rollingMean-expected)/
        expected;
      double standardDevationTolerance = rollingStandardDeviation/expected;
      
      if (meanTolerance > errorMeanTolerance)
        status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
          "Mean frequency in window exceeds tolerance by %f Hz.",
          (rollingEstimates.rollingMean < expected) ?
            rollingEstimates.rollingMean-expected*(1.0-errorMeanTolerance) :
            rollingEstimates.rollingMean-expected*(1.0+errorMeanTolerance));
      else if (meanTolerance > warnMeanTolerance)
        status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Mean frequency in window exceeds tolerance by %f Hz.",
          (rollingEstimates.rollingMean < expected) ?
            rollingEstimates.rollingMean-expected*(1.0-warnMeanTolerance) :
            rollingEstimates.rollingMean-expected*(1.0+warnMeanTolerance));
      else
        status.summary(diagnostic_msgs::DiagnosticStatus::OK,
          "Mean frequency in window meets expected tolerance.");
        
      if (standardDevationTolerance > errorStandardDeviationTolerance)
        status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
          "Frequency standard deviation in window exceeds tolerance by %f Hz.",
          rollingStandardDeviation-errorStandardDeviationTolerance*expected);
      else if (standardDevationTolerance > warnStandardDeviationTolerance)
        status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Frequency standard deviation in window exceeds tolerance by %f Hz.",
          rollingStandardDeviation-warnStandardDeviationTolerance*expected);
      else
        status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK,
          "Frequency standard deviation in window meets expected tolerance.");
    }
    else if (expected > 0.0)
      status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
        "No events in window.");
    else
      status.summary(diagnostic_msgs::DiagnosticStatus::OK,
        "No events in window.");
    
    status.addf("Expected frequency", "%f Hz", expected);
    status.addf("Time since last event", "%f s",
      statistics.getTimeSinceLastEvent().toSec());
    status.addf("Duration of window", "%f s", window.toSec());
    
    status.addf("Events since startup", "%d", estimates.numSamples);
    status.addf("Minimum frequency since startup", "%f Hz", estimates.min);
    status.addf("Maximum frequency since startup", "%f Hz", estimates.max);
    
    status.addf("Events in window", "%d", estimates.numRollingSamples);
    status.addf("Mean frequency in window", "%f Hz",
      rollingEstimates.rollingMean);
    status.addf("Frequency standard deviation in window", "%f Hz",
      sqrt(rollingEstimates.rollingVariance));
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Diagnostic task is currently inactive.");
}

}
