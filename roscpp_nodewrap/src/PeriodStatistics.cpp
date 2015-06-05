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

#include <boost/accumulators/statistics/rolling_count.hpp>

#include "roscpp_nodewrap/statistics/PeriodStatistics.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

PeriodStatistics::Estimates::Estimates() :
  numSamples(0),
  min(0.0),
  max(0.0),
  numRollingSamples(0),
  rollingMean(0.0),
  rollingVariance(std::numeric_limits<double>::infinity()) {
}

PeriodStatistics::Estimates::Estimates(const Estimates& src) :
  numSamples(src.numSamples),
  min(src.min),
  max(src.max),
  numRollingSamples(src.numRollingSamples),
  rollingMean(src.rollingMean),
  rollingVariance(src.rollingVariance) {
}

PeriodStatistics::PeriodStatistics(size_t rollingWindowSize) :
  accumulator(boost::accumulators::tag::rolling_window::window_size =
    rollingWindowSize),
  rollingWindowSize(rollingWindowSize) {
}

PeriodStatistics::PeriodStatistics(const PeriodStatistics& src) :
  accumulator(src.accumulator),
  rollingWindowSize(src.rollingWindowSize) {
}

PeriodStatistics::~PeriodStatistics() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void PeriodStatistics::setRollingWindowSize(size_t rollingWindowSize) {
  this->rollingWindowSize = rollingWindowSize;
  clear();
}

size_t PeriodStatistics::getRollingWindowSize() const {
  return rollingWindowSize;
}

size_t PeriodStatistics::getNumSamples() const {
  return boost::accumulators::count(accumulator);
}

size_t PeriodStatistics::getNumRollingSamples() const {
  return boost::accumulators::rolling_count(accumulator);
}

ros::Duration PeriodStatistics::getMin() const {
  return ros::Duration(boost::accumulators::min(accumulator));
}

ros::Duration PeriodStatistics::getMax() const {
  return ros::Duration(boost::accumulators::max(accumulator));
}

ros::Duration PeriodStatistics::getRollingMean() const {
  return ros::Duration(boost::accumulators::rolling_mean(accumulator));
}

ros::Duration PeriodStatistics::getRollingVariance() const {
  return ros::Duration(boost::accumulators::variance(accumulator));
}

PeriodStatistics::Estimates PeriodStatistics::getEstimates() const {
  Estimates estimates;
  
  estimates.numSamples = boost::accumulators::count(accumulator);
  if (estimates.numSamples) {
    estimates.min = ros::Duration(boost::accumulators::min(accumulator));
    estimates.max = ros::Duration(boost::accumulators::max(accumulator));
  }
  
  estimates.numRollingSamples = boost::accumulators::rolling_count(
    accumulator);
  if (estimates.numRollingSamples)
    estimates.rollingMean = ros::Duration(boost::accumulators::rolling_mean(
      accumulator));
  if (estimates.numRollingSamples > 1)
    estimates.rollingVariance = ros::Duration(boost::accumulators::variance(
      accumulator));
  else if (estimates.numRollingSamples)
    estimates.rollingVariance = ros::Duration(0.0);
  
  return estimates;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void PeriodStatistics::Estimates::toMessage(PeriodEstimates& msg) const {
  msg.num_samples = numSamples;
  msg.min = max.toSec();
  msg.max = min.toSec();
  
  msg.num_rolling_samples = numRollingSamples;
  msg.rolling_mean = rollingMean.toSec();
  if (numRollingSamples > 1)
    msg.rolling_variance = rollingVariance.toSec();
  else
    msg.rolling_variance = std::numeric_limits<double>::infinity();
}

void PeriodStatistics::clear() {
  CyclicEventStatistics::clear();
  accumulator = Accumulator(
    boost::accumulators::tag::rolling_window::window_size = rollingWindowSize);
}

}
