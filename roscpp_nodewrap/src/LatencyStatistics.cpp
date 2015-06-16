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

#include "roscpp_nodewrap/statistics/LatencyStatistics.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

LatencyStatistics::Estimates::Estimates() {
}

LatencyStatistics::Estimates::Estimates(const Estimates& src) :
  Statistics<double>::Estimates(src),
  timesOfLastEvents(src.timesOfLastEvents) {
}

LatencyStatistics::LatencyStatistics(size_t rollingWindowSize, const
    std::string& nameOfVariates, const std::string& unitOfVariates) :
  Statistics<double>(rollingWindowSize, nameOfVariates, unitOfVariates) {
}

LatencyStatistics::LatencyStatistics(const LatencyStatistics& src) :
  Statistics<double>(src) {
}

LatencyStatistics::~LatencyStatistics() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

const std::pair<ros::Time, ros::Time>&
    LatencyStatistics::getTimesOfLastEvents() const {
  return timesOfLastEvents;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void LatencyStatistics::extract(Estimates& estimates) const {
  Statistics<double>::extract(estimates);
  estimates.timesOfLastEvents = timesOfLastEvents;
}

void LatencyStatistics::clear() {
  Statistics<double>::clear();
  
  timesOfLastEvents.first = ros::Time();
  timesOfLastEvents.second = ros::Time();
}

}
