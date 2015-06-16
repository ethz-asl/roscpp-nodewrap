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

#include "roscpp_nodewrap/Subscriber.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class M>
SubscriberCallbackHelperT<M>::SubscriberCallbackHelperT(const Callback&
    callback) :
  helper(new ros::SubscriptionCallbackHelperT<Parameter>(
    boost::bind(&SubscriberCallbackHelperT<M>::wrappedCallback, this, _1))),
  callback(callback) {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class M>
const std::type_info& SubscriberCallbackHelperT<M>::getTypeInfo() {
  return helper->getTypeInfo();
}

template <class M>
bool SubscriberCallbackHelperT<M>::isConst() {
  return helper->isConst();
}

template <class M>
bool SubscriberCallbackHelperT<M>::hasHeader() {
  return helper->hasHeader();
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class M>
ros::VoidConstPtr SubscriberCallbackHelperT<M>::deserialize(const
    ros::SubscriptionCallbackHelperDeserializeParams& params) {
  return helper->deserialize(params);
}

template <class M>
void SubscriberCallbackHelperT<M>::call(
    ros::SubscriptionCallbackHelperCallParams& params) {
  helper->call(params);
}

template <class M>
void SubscriberCallbackHelperT<M>::wrappedCallback(Parameter message) {
  Subscriber::ImplPtr subscriberImpl =
    boost::static_pointer_cast<Subscriber::Impl>(this->subscriber.lock());
  
  if (subscriberImpl) {
    ros::Time now = ros::Time::now();
    
    callback(message);
    
    ++subscriberImpl->numProcessedMessages;
    
    subscriberImpl->processingFrequencyTask.event(now);
    
    if (subscriberImpl->messageHasHeader) {
      subscriberImpl->messageStampFrequencyTask.message(message);
      subscriberImpl->messageLatencyTask.message(message, now);
    }
  }
  else
    callback(message);
}

}
