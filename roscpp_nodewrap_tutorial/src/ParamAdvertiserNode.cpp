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

#include "roscpp_nodewrap_tutorial/ParamAdvertiserNode.h"

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

ParamAdvertiserNode::ParamAdvertiserNode() {
}

ParamAdvertiserNode::~ParamAdvertiserNode() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void ParamAdvertiserNode::init() {
  NODEWRAP_INFO("Hello!");

//   advertiseParam("chat/name", name, false);
//   advertiseParam("chat/initiate", initiate);
//   advertiseParam("chat/say", say, &ChatterNode::sayUpdate);
}

void ParamAdvertiserNode::cleanup() {
  NODEWRAP_INFO("Good bye!");
}

}
