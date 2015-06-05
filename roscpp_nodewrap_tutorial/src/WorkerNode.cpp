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

#include "roscpp_nodewrap_tutorial/WorkerNode.h"

NODEWRAP_EXPORT_CLASS(roscpp_nodewrap_tutorial, nodewrap::WorkerNode)

namespace nodewrap {

/*****************************************************************************/
/* Constructors and Destructor                                               */
/*****************************************************************************/

WorkerNode::WorkerNode() :
  flowers(0) {
}

WorkerNode::~WorkerNode() {
}

/*****************************************************************************/
/* Methods                                                                   */
/*****************************************************************************/

void WorkerNode::init() {
  flowers = getParam("field/flowers", flowers);
  
  subscriber = subscribe("wake_nursing", "/wake_nursing", 100,
    &WorkerNode::wakeNursing);
  NODEWRAP_INFO("Subscribed to: %s", subscriber.getTopic().c_str());
  
  task = addDiagnosticTask("field Flowers", &WorkerNode::diagnoseField);
  NODEWRAP_INFO("Created diagnostic task");
  
  houseKeeping = addWorker("house_keeping", 0, &WorkerNode::doHouseKeeping);
  NODEWRAP_INFO("Created worker [%s]", houseKeeping.getName().c_str());
  fanning = addWorker("fanning", 0, &WorkerNode::doFanning);
  NODEWRAP_INFO("Created worker [%s]", fanning.getName().c_str());
  nursing = addWorker("nursing", 0, &WorkerNode::doNursing, true, true);
  NODEWRAP_INFO("Created worker [%s]", nursing.getName().c_str());
  collecting = addWorker("collecting", 0, &WorkerNode::doCollecting);
  NODEWRAP_INFO("Created worker [%s]", collecting.getName().c_str());
  
  NODEWRAP_INFO("Hello, all workers have been created!");
}

void WorkerNode::cleanup() {
  NODEWRAP_INFO("Good bye, all workers have been stopped!");
}

void WorkerNode::diagnoseField(diagnostic_updater::DiagnosticStatusWrapper&
    status) const {
  if (!flowers)
    status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
      "Maya ran out of flowers and is starving.");
  else
    status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
      "Maya still has flowers %d to collect and is happy.", flowers);
}

bool WorkerNode::doHouseKeeping(const WorkerEvent& event) {
  NODEWRAP_DEBUG("Sweeping my cell...");
  return true;
}

bool WorkerNode::doFanning(const WorkerEvent& event) {
  NODEWRAP_DEBUG("Cooling down the hive...");
  
  ros::Duration duration = event.expectedCycleTime*2.0;
  duration.sleep();
  
  return true;
}

void WorkerNode::wakeNursing(const std_msgs::Empty::ConstPtr& msg) {
  nursing.wake();
}

bool WorkerNode::doNursing(const WorkerEvent& event) {
  NODEWRAP_DEBUG("Feeding the larvae...");
  return true;
}

bool WorkerNode::doCollecting(const WorkerEvent& event) {
  if (flowers > 0) {
    NODEWRAP_DEBUG("Collecting delicious nectar...");
    --flowers;
    
    return true;
  }
  else
    return false;
}

}
