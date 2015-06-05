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

#include "roscpp_nodewrap/diagnostics/DiagnosticsExceptions.h"

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class T> T DiagnosticUpdater::addTask(const std::string& name,
    const typename T::Options& defaultOptions) {
  boost::mutex::scoped_lock lock(this->impl->mutex);
  
  if (name.empty())
    throw InvalidDiagnosticTaskNameException(name,
      "Task name may not be empty");
    
  T task;
  std::map<std::string, DiagnosticTask::ImplWPtr>::iterator it =
    this->impl->tasks.find(name);
    
  if (it == this->impl->tasks.end()) {
    task.impl.reset(new typename T::Impl(name, defaultOptions,
      this->impl->nodeImpl));

    this->impl->tasks.insert(std::make_pair(name, task.impl));
  }
  else
    task.impl = boost::static_pointer_cast<typename T::Impl>(
      it->second.lock());
  
  return task;
}

}
