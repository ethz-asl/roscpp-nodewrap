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

/** \file Exceptions.h
  * \brief Header file defining exceptions for the ROS node wrapper
  */

#ifndef ROSCPP_NODEWRAP_EXCEPTIONS_H
#define ROSCPP_NODEWRAP_EXCEPTIONS_H

#include <ros/exception.h>

namespace nodewrap {
  /** \brief Exception thrown in case of an invalid parameter key
    */ 
  class InvalidParamKeyException :
    public ros::Exception {
  public:
    InvalidParamKeyException(const std::string& key, const std::string&
      reason);
  };
  
  /** \brief Exception thrown in case of a parameter type mismatch
    */ 
  class ParamTypeMismatchException :
    public ros::Exception {
  public:
    ParamTypeMismatchException(const std::string& expectedType, const
      std::string& providedType);
  };
};

#endif
