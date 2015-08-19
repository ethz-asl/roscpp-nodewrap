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

template <typename M> void Publisher::publish(const boost::shared_ptr<M>&
    message) const {
  if (impl) {
    ros::Time now = ros::Time::now();
    
    impl->publisher.publish(message);
    
    ++impl->numPublishedMessages;
    
    impl->publishingFrequencyTask.event(now);
    if (impl->messageHasHeader) {
      impl->messageStampFrequencyTask.message(message);
      impl->messageLatencyTask.message(message, now);
    }
  }
  else
    ROS_ASSERT_MSG(false, "Call to publish() on an invalid Publisher");
}

}
