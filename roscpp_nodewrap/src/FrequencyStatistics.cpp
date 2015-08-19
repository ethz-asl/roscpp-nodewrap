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

#include "roscpp_nodewrap/statistics/FrequencyStatistics.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

FrequencyStatistics::FrequencyStatistics(size_t rollingWindowSize, const
    std::string& nameOfVariates, const std::string& unitOfVariates) :
  CyclicEventStatistics<double>(rollingWindowSize, nameOfVariates,
    unitOfVariates) {
}

FrequencyStatistics::FrequencyStatistics(const FrequencyStatistics& src) :
  CyclicEventStatistics<double>(src) {
}

FrequencyStatistics::~FrequencyStatistics() {  
}

}
