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

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <typename T>
CyclicEventStatistics<T>::Estimates::Estimates() {
}

template <typename T>
CyclicEventStatistics<T>::Estimates::Estimates(const Estimates& src) :
  Statistics<T>::Estimates(src),
  timeOfLastEvent(src.timeOfLastEvent) {
}

template <typename T>
CyclicEventStatistics<T>::CyclicEventStatistics(size_t rollingWindowSize,
    const std::string& nameOfVariates, const std::string& unitOfVariates) :
  Statistics<T>(rollingWindowSize, nameOfVariates, unitOfVariates) {
}

template <typename T>
CyclicEventStatistics<T>::CyclicEventStatistics(const
    CyclicEventStatistics<T>& src) :
  Statistics<T>(src),
  timeOfLastEvent(src.timeOfLastEvent) {
}

template <typename T>
CyclicEventStatistics<T>::~CyclicEventStatistics() {  
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <typename T>
const ros::Time& CyclicEventStatistics<T>::getTimeOfLastEvent() const {
  return timeOfLastEvent;
}

template <typename T>
ros::Duration CyclicEventStatistics<T>::getTimeSinceLastEvent(const
    ros::Time& now) const {
  if (!this->timeOfLastEvent.isZero())
    return now-this->timeOfLastEvent;
  else
    return ros::Duration();
}    

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <typename T>
template <typename M> void CyclicEventStatistics<T>::Estimates::toMessage(
    M& message) const {
  Statistics<T>::Estimates::template toMessage<M>(message);
  message.stamp_of_last_event = timeOfLastEvent;
}

template <typename T>
void CyclicEventStatistics<T>::event(const ros::Time& timeOfEvent) {
  this->timeOfLastEvent = timeOfEvent;
}

template <typename T>
void CyclicEventStatistics<T>::extract(Estimates& estimates) const {
  Statistics<T>::extract(estimates);
  estimates.timeOfLastEvent = timeOfLastEvent;
}

template <typename T>
void CyclicEventStatistics<T>::clear() {
  Statistics<T>::clear();
  this->timeOfLastEvent = ros::Time();
}

}
