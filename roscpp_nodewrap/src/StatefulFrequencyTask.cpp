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

#include "roscpp_nodewrap/diagnostics/StatefulFrequencyTask.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

StatefulFrequencyTask::StatefulFrequencyTask() {
}

StatefulFrequencyTask::StatefulFrequencyTask(const StatefulFrequencyTask&
    src) :
  FrequencyTask(src) {
}

StatefulFrequencyTask::~StatefulFrequencyTask() {
}
    
StatefulFrequencyTask::Impl::Impl(const Options& defaultOptions, const
    std::string& name, const ManagerImplPtr& manager) :
  FrequencyTask::Impl(defaultOptions, name, manager),
  enabled(false) {
}
    
StatefulFrequencyTask::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void StatefulFrequencyTask::enable() {
  if (impl)
    impl->as<StatefulFrequencyTask::Impl>().enable();
}

void StatefulFrequencyTask::disable(bool clear) {
  if (impl)
    impl->as<StatefulFrequencyTask::Impl>().disable(clear);
}

void StatefulFrequencyTask::Impl::update(const double& sample) {
  boost::mutex::scoped_lock lock(stateMutex);
  
  if (enabled)
    FrequencyTask::Impl::update(sample);
}

void StatefulFrequencyTask::Impl::enable() {
  boost::mutex::scoped_lock lock(stateMutex);
  
  enabled = true;
}

void StatefulFrequencyTask::Impl::disable(bool clear) {
  boost::mutex::scoped_lock lock(stateMutex);
  
  enabled = false;

  if (clear)
    statistics.clear();  
}

void StatefulFrequencyTask::Impl::run(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  boost::mutex::scoped_lock lock(stateMutex);
  
  if (!enabled)
    status.summary(diagnostic_msgs::DiagnosticStatus::OK,
      "Diagnostic task is currently disabled.");
  else
    FrequencyTask::Impl::run(status);
}

}
