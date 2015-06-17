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

/** \file WorkerNode.h
  * \brief Header file providing the example WorkerNode class interface
  */

#ifndef ROSCPP_NODEWRAP_TUTORIAL_WORKER_NODE_HPP
#define ROSCPP_NODEWRAP_TUTORIAL_WORKER_NODE_HPP

#include <std_msgs/Empty.h>

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

namespace nodewrap {
  /** \brief Example worker node
    * 
    * This worker node demonstrates the concept of using multiple asynchronous
    * and synchronous workers with the ROS node and ROS nodelet template
    * wrappers. For this purpose, we consider the node called Maya, named after
    * the main character of a German comic book series about a worker bee,
    * under the following two use cases.
    * 
    * \section worker_node Worker Node with Single-Threaded Spinner
    * 
    * In this case, the worker node is instantiated using the templated
    * ROS node wrapper as:
    * 
      \verbatim
      int main(int argc, char** argv) {
        ros::init(argc, argv, "worker_node");
        
        Node<WorkerNode> node;

        ros::spin();
        
        return 0;
      }
      \endverbatim
    * 
    * Note that the call to ros::spin() immediately passes control to the
    * single-threaded ROS spinner and blocks until the worker node has been
    * shut down. The callbacks of all workers owned by the node will thus be
    * executed sequentially from the the process's main thread.
    *
    * For an example launch configuration, consider the file
    * worker_node.launch:
    *
      \verbatim
      <launch>
        <node name="maya" pkg="roscpp_nodewrap_tutorial" type="worker_node" output="screen">
          <rosparam command="load" file="$(find roscpp_nodewrap_tutorial)/config/maya.yaml"/>
        </node>
        <node pkg="rostopic" type="rostopic" name="wake_nursing" args="pub -r 20 /wake_nursing std_msgs/Empty" output="screen"/>
        ...
      </launch>
      \endverbatim
    *
    * \section worker_nodelet Worker Nodelet with Mult-Threaded Spinner
    *
    * In the second case, the worker node is instantiated by the ROS nodelet
    * manager as ROS nodelet, and the workers will be served from the thread
    * pool of the nodelet manager. The callbacks may thus run in parallel and
    * do not block each others execution. Care must however be taken when
    * reading and writing member variables in this case, as concurrency very
    * easily leads to conflicting memory access.
    *
    * For an example launch configuration, consider the file
    * worker_nodelet.launch:
    *
      \verbatim
      <launch>
        <node pkg="nodelet" type="nodelet" name="worker" args="manager" output="screen">
          <param name="num_worker_threads" value="2"/>
        </node>
        <node name="maya" pkg="nodelet" type="nodelet" args="load roscpp_nodewrap_tutorial/WorkerNode worker" output="screen">
          <rosparam command="load" file="$(find roscpp_nodewrap_tutorial)/config/maya.yaml"/>
        </node>
        <node pkg="rostopic" type="rostopic" name="wake_nursing" args="pub -r 20 /wake_nursing std_msgs/Empty" output="screen"/>
        ...
      </launch>
      \endverbatim
    *
    * For a complete example configuration, see the ROS package which provides
    * this tutorial.
    *
    * \section worker_priorities A Word about Worker Priorities
    *
    * In this implementation, time-critical workers can be configured to 
    * use their private callback queue and higher priorization. Under the
    * hood, the worker implementation therefore needs to make calls to the
    * native worker thread handle using the pthread library.
    * 
    * Most operating systems require special user or group privileges in
    * order for the thread to modifiy its scheduling policy and increase its
    * priority. On Unbuntu, these privileges may be granted to the user at
    * login by adding any of the following lines to /etc/security/limits.conf:
    *
      \verbatim
      [user]    -  rtprio  99
      @[group]  -  rtprio  99
      \endverbatim
    *
    * Note that the above placeholders [user] and [group] should be
    * substituted by the actual user/group membership of the user executing
    * the ROS node.
    */
  class WorkerNode :
    public NodeImpl {
  public:
    /** \brief Default constructor
      * 
      * The constructor of the worker node is left empty.
      */
    WorkerNode();
    
    /** \brief Destructor
      * 
      * The destructor of the worker node is left empty.
      */
    virtual ~WorkerNode();
  protected:
    /** \brief The ROS diagnostic function task checking the remaining
      *   number of flowers in the field
      */
    FunctionTask task;
    
    /** \brief The ROS worker performing the house keeping task
      */
    Worker houseKeeping;
    
    /** \brief The ROS worker performing the fanning task
      */
    Worker fanning;
    
    /** \brief The ROS subscriber listening to the topic for waking
      *   the nursing worker
      */
    Subscriber subscriber;
    
    /** \brief The ROS worker performing the nursing task
      */
    Worker nursing;
    
    /** \brief The ROS worker performing the collecting task
      */
    Worker collecting;
    
    /** \brief The human-readable name of the worker
      */
    std::string name;
    
    /** \brief The number of flowers for collecting
      */
    int flowers;
    
    /** \brief Perform worker node initialization
      * 
      * The initialization of the worker node simply involves retrieval of
      * the parameters and creation of the different workers. In this tutorial,
      * we shall think of the workers as different tasks which need to be
      * performed by Maya, the worker bee.
      *
      * This is the initializer's implementation:
      * 
        \verbatim
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
          nursing = addWorker("nursing", 0, &WorkerNode::doNursing);
          NODEWRAP_INFO("Created worker [%s]", nursing.getName().c_str());
          collecting = addWorker("collecting", 0, &WorkerNode::doCollecting);
          NODEWRAP_INFO("Created worker [%s]", collecting.getName().c_str());
          
          NODEWRAP_INFO("Hello, all workers have been created!");
        }
        \endverbatim
      * 
      * \see NodeImpl::init
      */
    void init();
    
    /** \brief Perform worker node cleanup
      * 
      * Cleanup of the worker node is just bogus. This is the implementation:
      * 
        \verbatim
        void WorkerNode::cleanup() {
          NODEWRAP_INFO("Good bye, all workers have been stopped!");
        }
        \endverbatim
      * 
      * \see NodeImpl::cleanup
      */
    void cleanup();
    
    /** \brief Field diagnostic function task callback
      * 
      * \param[in] status The diagnostic status to be filled in by this
      *   callback.
      * 
      * The field diagnostic function task callback checks the number of
      * remaining flowers in the field and will issue an error if this number
      * drops to zero. This is its implementation:
      * 
        \verbatim
        void WorkerNode::diagnoseField(diagnostic_updater::DiagnosticStatusWrapper&
            status) const {
          if (!flowers)
            status.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
              "Maya ran out of flowers and is starving.");
          else
            status.summaryf(diagnostic_msgs::DiagnosticStatus::OK,
              "Maya still has flowers %d to collect and is happy.", flowers);
        }
        \endverbatim
      */
    void diagnoseField(diagnostic_updater::DiagnosticStatusWrapper& status)
      const;
      
    /** \brief House keeping worker callback
      * 
      * \param[in] event The worker event providing information about the
      *   state of this worker.
      * \return True, if this worker has unfinished work to do.
      * 
      * The house keeping worker callback is simply printing a debug message.
      * This is its implementation:
      * 
        \verbatim
        bool WorkerNode::doHouseKeeping(const WorkerEvent& event) {
          NODEWRAP_DEBUG("Sweeping my cell...");
          return true;
        }
        \endverbatim
      */
    bool doHouseKeeping(const WorkerEvent& event);
    
    /** \brief Fanning worker callback
      * 
      * \param[in] event The worker event providing information about the
      *   state of this worker.
      * \return True, if this worker has unfinished work to do.
      * 
      * The fanning worker callback demonstrates the concept of excessive
      * computation: It will not return to the caller until twice the
      * expected execution time has elapsed.
      *
      * This is the implementation of the fanning worker callback:
      * 
        \verbatim
        bool WorkerNode::doFanning(const WorkerEvent& event) {
          NODEWRAP_DEBUG("Cooling down the hive...");
          
          ros::Duration duration = event.expectedCycleTime*2.0;
          duration.sleep();
          
          return true;
        }
        \endverbatim
      */
    bool doFanning(const WorkerEvent& event);
    
    /** \brief Wake nursing callback
      * 
      * This callback will be invoked every time a new message arrived
      * on the topic at which the wake nursing subscriber is listening.
      * 
      * \param[in] msg A const pointer reference to the received message.
      * 
      * This is the callback's implementation:
      * 
        \verbatim
        void WorkerNode::wakeNursing(const std_msgs::Empty::ConstPtr& msg) {
          nursing.wake();
        }
        \endverbatim
      *
      * Note that the call to nursing.wake() will wake up the nursing worker
      * on reception of a message and return immediately, i.e., without
      * blocking.
      *
      * \see wakeNursing
      */
    void wakeNursing(const std_msgs::Empty::ConstPtr& msg);
    
    /** \brief Nursing worker callback
      * 
      * \param[in] event The worker event providing information about the
      *   state of this worker.
      * \return True, if this worker has unfinished work to do.
      * 
      * The nursing worker callback demonstrates the concept of a synchronous
      * worker: It is not timer-controlled, but will run once everytime it is
      * woken up by a non-blocking call to nursing.wake().
      *
      * This is the implementation of the nursing worker callback:
      * 
        \verbatim
        bool WorkerNode::doNursing(const WorkerEvent& event) {
          NODEWRAP_DEBUG("Feeding the larvae...");
          return true;
        }
        \endverbatim
      *
      * \see wakeNursing
      */
    bool doNursing(const WorkerEvent& event);
    
    /** \brief Collecting worker callback
      * 
      * \param[in] event The worker event providing information about the
      *   state of this worker.
      * \return True, if this worker has unfinished work to do.
      * 
      * The collecting worker callback demonstrates the concept of a worker
      * which runs out of work: Once the nectar from all available flowers
      * has been collected, the callback returns false and thus prevents
      * future invocations.
      *
      * This is the implementation of the collecting worker callback:
      * 
        \verbatim
        bool WorkerNode::doCollecting(const WorkerEvent& event) {
          if (flowers > 0) {
            NODEWRAP_DEBUG("Collecting delicious nectar...");
            --flowers;
            
            return true;
          }
          else
            return false;
        }
        \endverbatim
      */
    bool doCollecting(const WorkerEvent& event);    
  };
};

#endif
