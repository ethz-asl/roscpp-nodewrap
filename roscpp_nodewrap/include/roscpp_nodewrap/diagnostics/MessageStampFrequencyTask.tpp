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

template <typename M> void MessageStampFrequencyTask::message(const
    boost::shared_ptr<M>& message) {
  if (this->impl)
    HasHeaderTraits<M>::message(this->impl, message);
}

template <typename M>
void MessageStampFrequencyTask::HasHeaderTraits<M, typename
    boost::disable_if<ros::message_traits::HasHeader<M> >::type>::
    message(const ImplPtr& impl, const boost::shared_ptr<M>& message) {
}

template <typename M>
void MessageStampFrequencyTask::HasHeaderTraits<M, typename
    boost::enable_if<ros::message_traits::HasHeader<M> >::type>::
    message(const ImplPtr& impl, const boost::shared_ptr<M>& message) {
  impl->as<MessageStampFrequencyTask::Impl>().statistics.event(
    message->header.stamp);
}

}
