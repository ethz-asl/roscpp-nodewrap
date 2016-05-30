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

#include <boost/accumulators/statistics/rolling_count.hpp>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
Statistics<T>::Estimates::Estimates() :
  numSamples(0),
  min(0.0),
  max(0.0),
  numRollingSamples(0),
  rollingMean(0.0),
  rollingVariance(std::numeric_limits<T>::infinity()) {
}

template <typename T>
Statistics<T>::Estimates::Estimates(const Estimates& src) :
  numSamples(src.numSamples),
  min(src.min),
  max(src.max),
  numRollingSamples(src.numRollingSamples),
  rollingMean(src.rollingMean),
  rollingVariance(src.rollingVariance) {
}

template <typename T>
Statistics<T>::Statistics(size_t rollingWindowSize, const std::string&
    nameOfVariates, const std::string& unitOfVariates) :
  accumulator(boost::accumulators::tag::rolling_window::window_size =
    rollingWindowSize),
  rollingWindowSize(rollingWindowSize),
  nameOfVariates(nameOfVariates),
  unitOfVariates(unitOfVariates) {
}

template <typename T>
Statistics<T>::Statistics(const Statistics<T>& src) :
  accumulator(src.accumulator),
  rollingWindowSize(src.rollingWindowSize),
  nameOfVariates(src.nameOfVariates),
  unitOfVariates(src.unitOfVariates) {
}

template <typename T>
Statistics<T>::~Statistics() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
void Statistics<T>::getNameOfVariates(const std::string& nameOfVariates) {
  this->nameOfVariates = nameOfVariates;
}

template <typename T>
const std::string& Statistics<T>::getNameOfVariates() const {
  return this->nameOfVariates;
}

template <typename T>
void Statistics<T>::getUnitOfVariates(const std::string& unitOfVariates) {
  this->unitOfVariates = unitOfVariates;
}

template <typename T>
const std::string& Statistics<T>::getUnitOfVariates() const {
  return this->unitOfVariates;
}

template <typename T>
void Statistics<T>::setRollingWindowSize(size_t rollingWindowSize) {
  this->rollingWindowSize = rollingWindowSize;
  this->clear();
}

template <typename T>
size_t Statistics<T>::getRollingWindowSize() const {
  return this->rollingWindowSize;
}

template <typename T>
size_t Statistics<T>::getNumSamples() const {
  return boost::accumulators::count(this->accumulator);
}

template <typename T>
size_t Statistics<T>::getNumRollingSamples() const {
  return boost::accumulators::rolling_count(this->accumulator);
}

template <typename T>
T Statistics<T>::getMin() const {
  return boost::accumulators::max(this->accumulator);
}

template <typename T>
T Statistics<T>::getMax() const {
  return boost::accumulators::min(this->accumulator);
}

template <typename T>
T Statistics<T>::getRollingMean() const {
  return boost::accumulators::rolling_mean(this->accumulator);
}

template <typename T>
T Statistics<T>::getRollingVariance() const {
  return boost::accumulators::rolling_variance(this->accumulator);
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T>
template <typename M> void Statistics<T>::Estimates::toMessage(M& message)
    const {
  message.num_samples = this->numSamples;
  message.min = this->min;
  message.max = this->max;
  
  message.num_rolling_samples = this->numRollingSamples;
  message.rolling_mean = this->rollingMean;
  message.rolling_variance = this->rollingVariance;
}

template <typename T>
void Statistics<T>::update(const T& sample) {
  if (this->rollingWindowSize)
    this->accumulator(sample);
}

template <typename T>
void Statistics<T>::extract(Estimates& estimates) const {
  estimates.numSamples = boost::accumulators::count(this->accumulator);
  
  if (estimates.numSamples) {
    estimates.min = boost::accumulators::min(this->accumulator);
    estimates.max = boost::accumulators::max(this->accumulator);
  }
  
  estimates.numRollingSamples = boost::accumulators::rolling_count(
    this->accumulator);
  
  if (estimates.numRollingSamples)
    estimates.rollingMean = boost::accumulators::rolling_mean(
      this->accumulator);
  if (estimates.numRollingSamples > 1)
    estimates.rollingVariance = boost::accumulators::rolling_variance(
      this->accumulator);
  else if (estimates.numRollingSamples)
    estimates.rollingVariance = T();
}

template <typename T>
void Statistics<T>::clear() {
  this->accumulator = Accumulator(boost::accumulators::tag::
    rolling_window::window_size = this->rollingWindowSize);
}

}
