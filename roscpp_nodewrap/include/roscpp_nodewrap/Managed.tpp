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

#include "roscpp_nodewrap/Manager.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

template <class T, typename I>
Managed<T, I>::Managed() {
}

template <class T, typename I>
Managed<T, I>::Managed(const Managed<T, I>& src) :
  impl(src.impl) {
}

template <class T, typename I>
Managed<T, I>::~Managed() {  
}

template <class T, typename I>
Managed<T, I>::Impl::Impl(const I& identifier, const ManagerImplPtr& manager) :
  identifier(identifier),
  manager(manager) {
}

template <class T, typename I>
Managed<T, I>::Impl::~Impl() {
  manager->remove(this->identifier);
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

template <class T, typename I>
I Managed<T, I>::getIdentifier() const {
  if (this->impl)
    return this->impl->getIdentifier();
  else
    return I();
}

template <class T, typename I>
const NodeImplPtr& Managed<T, I>::Impl::getNode() const {
  return manager->getNode();
}

template <class T, typename I>
const I& Managed<T, I>::Impl::getIdentifier() const {
  return this->identifier;
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

template <class T, typename I>
void Managed<T, I>::shutdown() {
  if (this->impl)
    this->impl->shutdown();
}

template <class T, typename I>
template <class D> D& Managed<T, I>::Impl::as() {
  return *static_cast<D*>(this);
}

template <class T, typename I>
template <class D> const D& Managed<T, I>::Impl::as() const {
  return *static_cast<const D*>(this);
}

}
