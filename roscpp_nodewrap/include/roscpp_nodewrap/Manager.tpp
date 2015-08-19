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

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class T, typename I>
Manager<T, I>::Manager() {
}

template <class T, typename I>
Manager<T, I>::Manager(const Manager<T, I>& src) :
  impl(src.impl) {
}

template <class T, typename I>
Manager<T, I>::~Manager() {  
}

template <class T, typename I>
Manager<T, I>::Impl::Impl() {
}
    
template <class T, typename I>
Manager<T, I>::Impl::~Impl() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class T, typename I>
void Manager<T, I>::shutdown() {
  if (this->impl)
    this->impl->shutdown();
}

template <class T, typename I>
void Manager<T, I>::Impl::add(const typename T::ImplPtr& instance) {
  boost::mutex::scoped_lock lock(this->mutex);

  this->instances[instance->identifier] = instance;
}

template <class T, typename I>
typename T::ImplPtr Manager<T, I>::Impl::remove(const I& identifier) {
  boost::mutex::scoped_lock lock(this->mutex);

  typename T::ImplPtr instance;
  
  typename std::map<I, typename T::ImplWPtr>::iterator it =
    this->instances.find(identifier);
  if (it != this->instances.end()) {
    instance = it->second.lock();
    this->instances.erase(it);
  }
  
  return instance;
}

template <class T, typename I>
typename T::ImplPtr Manager<T, I>::Impl::find(const I& identifier) const {
  boost::mutex::scoped_lock lock(this->mutex);
  
  typename std::map<I, typename T::ImplWPtr>::const_iterator it =
    this->instances.find(identifier);
  if (it != this->instances.end())
    return it->second.lock();
    
  return typename T::ImplPtr();
}

template <class T, typename I>
template <class D> D& Manager<T, I>::Impl::as() {
  return *static_cast<D*>(this);
}

template <class T, typename I>
template <class D> const D& Manager<T, I>::Impl::as() const {
  return *static_cast<const D*>(this);
}

}
