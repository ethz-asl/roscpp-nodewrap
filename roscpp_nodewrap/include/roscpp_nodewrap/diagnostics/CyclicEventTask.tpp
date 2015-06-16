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

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class S>
CyclicEventTask<S>::CyclicEventTask() {
}

template <class S>
CyclicEventTask<S>::CyclicEventTask(const CyclicEventTask& src) :
  StatisticsTask<S>(src) {
}

template <class S>
CyclicEventTask<S>::~CyclicEventTask() {
}

template <class S>
CyclicEventTask<S>::Impl::Impl(const Options& defaultOptions, const 
    std::string& name, const typename StatisticsTask<S>::ManagerImplPtr&
    manager) :
  StatisticsTask<S>::Impl(defaultOptions, name, manager) {
}
    
template <class S>
CyclicEventTask<S>::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class S>
ros::Time CyclicEventTask<S>::getExpectedTimeOfNextEvent() const {
  if (this->impl)
    return this->impl->template as<typename
      CyclicEventTask<S>::Impl>().getExpectedTimeOfNextEvent();
  else
    return ros::Time();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class S>
void CyclicEventTask<S>::event(const ros::Time& timeOfEvent) {
  if (this->impl)
    this->impl->template as<typename CyclicEventTask<S>::Impl>().event(
      timeOfEvent);
}

template <class S>
void CyclicEventTask<S>::Impl::event(const ros::Time& timeOfEvent) {
  boost::mutex::scoped_lock lock(this->mutex);
  
  this->statistics.event(timeOfEvent);
}

template <class S>
void CyclicEventTask<S>::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  boost::mutex::scoped_lock lock(this->mutex);

  S advancedStatistics = this->statistics;
  ros::Time now = ros::Time::now();
  
  if (now > this->getExpectedTimeOfNextEvent())
    advancedStatistics.event(now);
  
  typename S::Estimates estimates, advancedEstimates;
  this->statistics.extract(estimates);
  advancedStatistics.extract(advancedEstimates);

  estimates.min = advancedEstimates.min;
  estimates.max = advancedEstimates.max;
  
  estimates.rollingMean = advancedEstimates.rollingMean;
  estimates.rollingVariance = advancedEstimates.rollingVariance;
    
  this->statusSummary(status, estimates);
  this->statusAdd(status, estimates);
}  

template <class S>
void CyclicEventTask<S>::Impl::statusSummary(
    diagnostic_updater::DiagnosticStatusWrapper& status,
    const typename S::Estimates& estimates) {
  if (estimates.numRollingSamples)
    StatisticsTask<S>::Impl::statusSummary(status, estimates);
  else if (!this->getExpectedTimeOfNextEvent().isZero())
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "No events in window.");
  else
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "No events expected.");
}

template <class S>
void CyclicEventTask<S>::Impl::statusAdd(
    diagnostic_updater::DiagnosticStatusWrapper& status,
    const typename S::Estimates& estimates) {
  if (!this->statistics.getTimeOfLastEvent().isZero()) {
    status.addf("Time since last event", "%f s",
      this->statistics.getTimeSinceLastEvent().toSec());
    
    StatisticsTask<S>::Impl::statusAdd(status, estimates);
  }
}

}
