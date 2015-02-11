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

#include <cxxabi.h>

#include "roscpp_nodewrap/ParamType.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ParamType::ParamType() {
}

ParamType::ParamType(const ParamType& src) :
  impl(src.impl) {
}

ParamType::~ParamType() {
}

ParamType::Impl::Impl(const std::type_info& info) {
  size_t length;
  int status;
  
  name = abi::__cxa_demangle(info.name(), 0, &length, &status);
  
  if (status)
    name = info.name();
}

ParamType::Impl::~Impl() {
}

/*****************************************************************************/
/* Accessors                                                                 */
/*****************************************************************************/

std::string ParamType::getName() const {
  if (impl)
    return impl->name;
  else
    return std::string();
}

bool ParamType::Impl::isValid() const {
  return !name.empty();
}

}
