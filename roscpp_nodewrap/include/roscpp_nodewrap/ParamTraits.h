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

/** \file ParamTraits.h
  * \brief Header file providing the ParamTraits class interface
  */

#ifndef ROSCPP_NODEWRAP_PARAM_TRAITS_H
#define ROSCPP_NODEWRAP_PARAM_TRAITS_H

#include <boost/type_traits.hpp>

#include <ros/ros.h>

namespace nodewrap {
  /** \brief ROS parameter traits
    * 
    * This class defines type traits for standard parameter value types
    * through template specialization..
    */
  class ParamTraits {
  public:
    /** \brief Type trait for identifying integer types
      */
    template <typename T> class IsInteger :
    public boost::type_traits::ice_and<boost::is_integral<T>::value,
      boost::type_traits::ice_not<boost::is_same<T, bool>::value>::value> {};
      
    /** \brief Type trait for identifying floating-point types
      */
    template <typename T> class IsFloat :
    public boost::is_float<T> {};
  };
};

#endif
