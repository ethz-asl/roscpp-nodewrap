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

#include <sstream>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class S>
StatisticsTask<S>::StatisticsTask() {
}

template <class S>
StatisticsTask<S>::StatisticsTask(const StatisticsTask<S>& src) :
  DiagnosticTask(src) {
}

template <class S>
StatisticsTask<S>::~StatisticsTask() {
}
    
template <class S>
StatisticsTask<S>::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  DiagnosticTask::Impl(name, manager),
  expected(0.0),
  warnMeanTolerance(0.05),
  errorMeanTolerance(0.1),
  warnStandardDeviationTolerance(0.05),
  errorStandardDeviationTolerance(0.1) {
  std::string ns = defaultOptions.ns.empty() ? 
    ros::names::append("diagnostics", name) : defaultOptions.ns;
  
  this->expected = getNode()->getParam(
    ros::names::append(ns, "expected"), defaultOptions.expected);
  
  this->windowSize = getNode()->getParam(
    ros::names::append(ns, "window_size"), (int)defaultOptions.windowSize);
  
  this->warnMeanTolerance = getNode()->getParam(
    ros::names::append(ns, "mean_tolerance/warn"),
    defaultOptions.warnMeanTolerance);
  this->errorMeanTolerance = getNode()->getParam(
    ros::names::append(ns, "mean_tolerance/error"),
    defaultOptions.errorMeanTolerance);
  
  this->warnStandardDeviationTolerance = getNode()->getParam(
    ros::names::append(ns, "standard_deviation_tolerance/warn"),
    defaultOptions.warnStandardDeviationTolerance);
  this->errorStandardDeviationTolerance = getNode()->getParam(
    ros::names::append(ns, "standard_deviation_tolerance/error"),
    defaultOptions.errorStandardDeviationTolerance);

  this->statistics.setRollingWindowSize(this->windowSize);
}
    
template <class S>
StatisticsTask<S>::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class S>
S StatisticsTask<S>::getStatistics() const {
  if (impl)
    return impl->template as<StatisticsTask<S>::Impl>().getStatistics();
  else
    return S();
}

template <class S>
typename S::Estimates StatisticsTask<S>::getStatisticsEstimates()
    const {
  if (impl)
    return impl->template as<StatisticsTask<S>::Impl>().
      getStatisticsEstimates();
  else
    return typename S::Estimates();
}

template <class S>
const S& StatisticsTask<S>::Impl::getStatistics() const {
  boost::mutex::scoped_lock lock(this->mutex);
  
  return this->statistics;
}

template <class S>
typename S::Estimates StatisticsTask<S>::Impl::getStatisticsEstimates()
    const {
  boost::mutex::scoped_lock lock(this->mutex);
  
  typename S::Estimates estimates;      
  this->statistics.extract(estimates);
  
  return estimates;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class S>
void StatisticsTask<S>::update(const typename S::Variate& sample) {
  if (this->impl)
    this->impl->template as<typename StatisticsTask<S>::Impl>().update(
      sample);
}

template <class S>
void StatisticsTask<S>::clear() {
  if (this->impl)
    this->impl->template as<StatisticsTask<S>::Impl>().clear();
}

template <class S>
void StatisticsTask<S>::Impl::update(const typename S::Variate& sample) {
  boost::mutex::scoped_lock lock(this->mutex);

  this->statistics.update(sample);
}

template <class S>
void StatisticsTask<S>::Impl::stop() {
  boost::mutex::scoped_lock lock(this->mutex);
  
  this->statistics.clear();
  
  DiagnosticTask::Impl::stop();
}

template <class S>
void StatisticsTask<S>::Impl::clear() {
  boost::mutex::scoped_lock lock(this->mutex);
  
  this->statistics.clear();
}

template <class S>
void StatisticsTask<S>::Impl::run(diagnostic_updater::DiagnosticStatusWrapper&
    status) {
  boost::mutex::scoped_lock lock(this->mutex);
  
  typename S::Estimates estimates;
  statistics.extract(estimates);

  this->statusSummary(status, estimates);
  this->statusAdd(status, estimates);
}

template <class S>
void StatisticsTask<S>::Impl::statusSummary(
    diagnostic_updater::DiagnosticStatusWrapper& status, const typename
    S::Estimates& estimates) {
  if (estimates.numRollingSamples) {
    double rollingStandardDeviation = sqrt(estimates.rollingVariance);
    double meanTolerance = fabs(estimates.rollingMean-this->expected)/
      this->expected;
    double standardDevationTolerance = rollingStandardDeviation/
      this->expected;
    
    std::string meanPrefix("Sample mean");
    if (!this->statistics.getNameOfVariates().empty()) {
      std::string lowerNameOfVariates(this->statistics.getNameOfVariates());
      lowerNameOfVariates[0] = std::tolower(lowerNameOfVariates[0]);
      meanPrefix = "Mean "+lowerNameOfVariates;
    }
    
    std::string standardDevationPrefix("Sample standard deviation");
    if (!this->statistics.getNameOfVariates().empty()) {
      std::string upperNameOfVariates(this->statistics.getNameOfVariates());
      upperNameOfVariates[0] = std::toupper(upperNameOfVariates[0]);
      standardDevationPrefix = upperNameOfVariates+" standard deviation";
    }
    
    std::string unitSuffix;
    if (!this->statistics.getUnitOfVariates().empty())
      unitSuffix = std::string(" ")+this->statistics.getUnitOfVariates();
    
    if (meanTolerance > this->errorMeanTolerance) {
      std::ostringstream stream;
      if (estimates.rollingMean < this->expected)
        stream << estimates.rollingMean-this->expected*
          (1.0-this->errorMeanTolerance);
      else
        stream << estimates.rollingMean-this->expected*
          (1.0+this->errorMeanTolerance);
      
      status.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "%s in window exceeds tolerance by %s%s.",
        meanPrefix.c_str(),
        stream.str().c_str(),
        unitSuffix.c_str());
    }
    else if (meanTolerance > this->warnMeanTolerance) {
      std::ostringstream stream;
      if (estimates.rollingMean < this->expected)
        stream << estimates.rollingMean-this->expected*
          (1.0-this->warnMeanTolerance);
      else
        stream << estimates.rollingMean-this->expected*
          (1.0+this->warnMeanTolerance);
      
      status.summaryf(diagnostic_msgs::DiagnosticStatus::WARN,
        "%s in window exceeds tolerance by %s%s.",
        meanPrefix.c_str(),
        stream.str().c_str(),
        unitSuffix.c_str());
    }
    else
      status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "%s in window meets expected tolerance.",
        meanPrefix.c_str());
    
    if (standardDevationTolerance >
        this->errorStandardDeviationTolerance) {
      std::ostringstream stream;
      stream << rollingStandardDeviation-
        this->errorStandardDeviationTolerance*this->expected;
        
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
        "%s in window exceeds tolerance by %s%s.",
        standardDevationPrefix.c_str(),
        stream.str().c_str(),
        unitSuffix.c_str());
    }
    else if (standardDevationTolerance > 
        this->warnStandardDeviationTolerance) {
      std::ostringstream stream;
      stream << rollingStandardDeviation-
        this->warnStandardDeviationTolerance*this->expected;
        
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
        "%s in window exceeds tolerance by %s%s.",
        standardDevationPrefix.c_str(),
        stream.str().c_str(),
        unitSuffix.c_str());
    }
    else
      status.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK,
        "%s in window meets expected tolerance.",
        standardDevationPrefix.c_str());
  }
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "No samples in window.");
}

template <class S>
void StatisticsTask<S>::Impl::statusAdd(
    diagnostic_updater::DiagnosticStatusWrapper& status, const typename
    S::Estimates& estimates) {
  std::string lowerNameOfVariates(this->statistics.getNameOfVariates());
  if (!lowerNameOfVariates.empty())
    lowerNameOfVariates[0] = std::tolower(lowerNameOfVariates[0]);  
  
  std::string upperNameOfVariates(this->statistics.getNameOfVariates());
  if (!upperNameOfVariates.empty())
    upperNameOfVariates[0] = std::toupper(upperNameOfVariates[0]);
  
  std::string unitSuffix;
  if (!this->statistics.getUnitOfVariates().empty())
    unitSuffix = std::string(" ")+this->statistics.getUnitOfVariates();
  
  {
    std::ostringstream stream;
    stream << expected;
    
    if (!lowerNameOfVariates.empty())
      status.addf(std::string("Expected ")+lowerNameOfVariates, "%s%s",
        stream.str().c_str(),
        unitSuffix.c_str());
    else
      status.addf("Expected value", "%s%s",
        stream.str().c_str(),
        unitSuffix.c_str());
  }
  
  status.addf("Number of samples since startup", "%d",
    estimates.numSamples);
  
  {
    std::ostringstream stream;
    stream << estimates.min;
    
    status.addf(std::string("Minimum ")+(lowerNameOfVariates.empty() ?
      "sample" : lowerNameOfVariates.c_str())+" since startup",
      "%s%s",
      stream.str().c_str(),
      unitSuffix.c_str());
  }
  
  {
    std::ostringstream stream;
    stream << estimates.max;
    
    status.addf(std::string("Maximum ")+(lowerNameOfVariates.empty() ?
      "sample" : lowerNameOfVariates)+" since startup",
      "%s%s",
      stream.str().c_str(),
      unitSuffix.c_str());
  }
  
  status.addf("Number of samples in window", "%d",
    estimates.numRollingSamples);
  
  {
    std::ostringstream stream;
    stream << estimates.rollingMean;
    
    if (!lowerNameOfVariates.empty())
      status.addf(std::string("Mean ")+lowerNameOfVariates+" in window",
        "%s%s",
        stream.str().c_str(),
        unitSuffix.c_str());
    else
      status.addf("Sample mean in window", "%s%s",
        stream.str().c_str(),
        unitSuffix.c_str());
  }
  
  {
    std::ostringstream stream;
    stream << sqrt(estimates.rollingVariance);
    
    status.addf((upperNameOfVariates.empty() ? std::string("Sample") :
      upperNameOfVariates.c_str())+" standard deviation in window",
      "%s%s",
      stream.str().c_str(),
      unitSuffix.c_str());
  }
}

}
