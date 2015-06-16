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
/* Methods                                                                   */
/*****************************************************************************/

template <typename M> void MessageLatencyTask::message(const
    boost::shared_ptr<M>& message, const ros::Time& timeOfEvent) {
  if (this->impl)
    HasHeaderTraits<M>::message(this->impl, message, timeOfEvent);
}

template <typename M>
void MessageLatencyTask::HasHeaderTraits<M, typename
    boost::disable_if<ros::message_traits::HasHeader<M> >::type>::
    message(const ImplPtr& impl, const boost::shared_ptr<M>& message,
    const ros::Time& timeOfEvent) {
}

template <typename M>
void MessageLatencyTask::HasHeaderTraits<M, typename
    boost::enable_if<ros::message_traits::HasHeader<M> >::type>::
    message(const ImplPtr& impl, const boost::shared_ptr<M>& message,
    const ros::Time& timeOfEvent) {
  impl->as<MessageLatencyTask::Impl>().statistics.events(
    message->header.stamp, timeOfEvent);
}

}
