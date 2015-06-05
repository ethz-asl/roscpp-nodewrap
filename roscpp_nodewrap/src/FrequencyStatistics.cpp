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

#include "roscpp_nodewrap/statistics/FrequencyStatistics.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

FrequencyStatistics::Estimates::Estimates() :
  numSamples(0),
  min(0.0),
  max(0.0),
  numRollingSamples(0),
  rollingMean(0.0),
  rollingVariance(std::numeric_limits<double>::infinity()) {
}

FrequencyStatistics::Estimates::Estimates(const Estimates& src) :
  numSamples(src.numSamples),
  min(src.min),
  max(src.max),
  numRollingSamples(src.numRollingSamples),
  rollingMean(src.rollingMean),
  rollingVariance(src.rollingVariance) {
}

FrequencyStatistics::FrequencyStatistics(size_t rollingWindowSize) :
  accumulator(boost::accumulators::tag::rolling_window::window_size =
    rollingWindowSize),
  rollingWindowSize(rollingWindowSize) {
}

FrequencyStatistics::FrequencyStatistics(const FrequencyStatistics& src) :
  accumulator(src.accumulator),
  rollingWindowSize(src.rollingWindowSize) {
}

FrequencyStatistics::~FrequencyStatistics() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

void FrequencyStatistics::setRollingWindowSize(size_t rollingWindowSize) {
  this->rollingWindowSize = rollingWindowSize;
  clear();
}

size_t FrequencyStatistics::getRollingWindowSize() const {
  return rollingWindowSize;
}

size_t FrequencyStatistics::getNumSamples() const {
  return boost::accumulators::count(accumulator);
}

size_t FrequencyStatistics::getNumRollingSamples() const {
  return boost::accumulators::rolling_count(accumulator);
}

double FrequencyStatistics::getMin() const {
  return boost::accumulators::max(accumulator);
}

double FrequencyStatistics::getMax() const {
  return boost::accumulators::min(accumulator);
}

double FrequencyStatistics::getRollingMean() const {
  return boost::accumulators::rolling_mean(accumulator);
}

double FrequencyStatistics::getRollingVariance() const {
  return boost::accumulators::rolling_variance(accumulator);
}

FrequencyStatistics::Estimates FrequencyStatistics::getEstimates() const {
  Estimates estimates;
  
  estimates.timeOfLastEvent = timeOfLastEvent;
  
  estimates.numSamples = boost::accumulators::count(accumulator);
  if (estimates.numSamples) {
    estimates.min = boost::accumulators::min(accumulator);
    estimates.max = boost::accumulators::max(accumulator);
  }
  
  estimates.numRollingSamples = boost::accumulators::rolling_count(
    accumulator);
  if (estimates.numRollingSamples)
    estimates.rollingMean = boost::accumulators::rolling_mean(accumulator);
  if (estimates.numRollingSamples > 1)
    estimates.rollingVariance = boost::accumulators::rolling_variance(
      accumulator);
  else if (estimates.numRollingSamples)
    estimates.rollingVariance = 0.0;
  
  return estimates;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void FrequencyStatistics::Estimates::toMessage(FrequencyEstimates& msg) const {
  msg.stamp_of_last_event = timeOfLastEvent;
  
  msg.num_samples = numSamples;
  msg.min = min;
  msg.max = max;
  
  msg.num_rolling_samples = numRollingSamples;
  msg.rolling_mean = rollingMean;
  msg.rolling_variance = rollingVariance;
}

void FrequencyStatistics::clear() {
  CyclicEventStatistics::clear();
  accumulator = Accumulator(
    boost::accumulators::tag::rolling_window::window_size = rollingWindowSize);
}

}
