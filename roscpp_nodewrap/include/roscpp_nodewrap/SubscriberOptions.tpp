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

#include "roscpp_nodewrap/SubscriberCallbackHelper.h"

namespace nodewrap {

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class M> void SubscriberOptions::init(
    const std::string& topic, uint32_t queue_size, const
    boost::function<void(const boost::shared_ptr<M const>&)>& callback) {
  this->topic = topic;
  this->queue_size = queue_size;
  this->md5sum = ros::message_traits::template md5sum<M>();
  this->datatype = ros::message_traits::template datatype<M>();
  
  this->helper = ros::SubscriptionCallbackHelperPtr(new
    SubscriberCallbackHelperT<M>(callback)); 
}

}
