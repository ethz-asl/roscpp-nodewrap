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

/** \file ChatterNode.h
  * \brief Header file providing the example ChatterNode class interface
  */

#ifndef ROSCPP_NODEWRAP_TUTORIAL_CHATTER_NODE_HPP
#define ROSCPP_NODEWRAP_TUTORIAL_CHATTER_NODE_HPP

#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <roscpp_nodewrap/NodeImpl.h>
#include <roscpp_nodewrap/Nodelet.h>

namespace nodewrap {
  /** \brief Example chatter node
    * 
    * This chatter node demonstrates the concept of implementing a node
    * for use with the ROS node and ROS nodelet template wrappers. For
    * this purpose, one may consider the following three use cases.
    * 
    * \section chatter_nodes Stand-alone Chatter Nodes
    * 
    * In this case, each of both chatter nodes may be instantiated using
    * the templated ROS node wrapper as:
    * 
      \verbatim
      int main(int argc, char** argv) {
        ros::init(argc, argv, "chatter_node");
        
        Node<ChatterNode> node;

        ros::spin();
        
        return 0;
      }
      \endverbatim
    * 
    * Note that one would be required to launch two separate processes
    * with distinct namespaces and parameters, one for each chatter node.
    * The messages exchanged between both chatters will be serialized and
    * deserialized by ROS to be transferred via an XML-formatted network
    * stream.
    *
    * For an example launch configuration, consider the file
    * chatter_nodes.launch:
    *
      \verbatim
      <launch>
        <node name="alice" pkg="roscpp_nodewrap_tutorial" type="chatter_node" output="screen">
          <rosparam command="load" file="$(find roscpp_nodewrap_tutorial)/etc/alice.yaml"/>
        </node>
        <node name="bob" pkg="roscpp_nodewrap_tutorial" type="chatter_node" output="screen">
          <rosparam command="load" file="$(find roscpp_nodewrap_tutorial)/etc/bob.yaml"/>
        </node>
        ...
      </launch>
      \endverbatim
    *
    * \section chatter_supernode Chatter Supernode
    * 
    * In this case, both chatter nodes may be instantiated using the
    * templated ROS node wrapper as:
    * 
      \verbatim
      int main(int argc, char** argv) {
        ros::init(argc, argv, "chatter_supernode");
        
        Node<ChatterNode> alice("alice");
        Node<ChatterNode> bob("bob");

        ros::spin();
        
        return 0;
      }
      \endverbatim
    * 
    * As a consequence of the above chatter instantiation within a single
    * main function, ROS will be able to determine that both chatter nodes
    * live within the same process and therefore skip message serialization
    * and deserialization, effectively passing pointers between the publishers
    * and subscribers of a topic.
    *
    * For an example launch configuration, consider the file
    * chatter_supernode.launch:
    *
      \verbatim
      <launch>
        <node name="chatters" pkg="roscpp_nodewrap_tutorial" type="chatter_supernode" output="screen">
          <rosparam command="load" ns="/alice" file="$(find roscpp_nodewrap_tutorial)/etc/alice.yaml"/>
          <rosparam command="load" ns="/bob" file="$(find roscpp_nodewrap_tutorial)/etc/bob.yaml"/>
        </node>
        ...
      </launch>
      \endverbatim
    *
    * \section chatter_nodelets Chatter Nodelets
    *
    * The last use case combines the efficiency of the supernode solution
    * with the flexibility of dynamically loaded plugin classes. Instead
    * of explicitly instantiating the chatter nodes in a main function,
    * the ROS nodelet wrapper allows the node implementations to be 
    * loaded (and unloaded) at runtime by a ROS nodelet manager. As both
    * chatters would thus still coexist within the same process, ROS may
    * effectively skip message serialization and deserialization.
    *
    * In order to allow for the nodelet manager to dynamically instantiate
    * a nodelet, an implementation of this nodelet must be provided in the
    * form of a shared library. To ensure that our templated ROS nodelet
    * wrapper will be compiled into such a library and registered with the
    * ROS dynamic class loading facilities, one must place the wrapper
    * macro #NODEWRAP_EXPORT_CLASS into any source file that will be compiled
    * into the library containing the node implementation. For our example
    * ChatterNode, the macro is invoked in ChatterNode.cpp as follows:
    *
      \verbatim
      NODEWRAP_EXPORT_CLASS(roscpp_nodewrap_tutorial, nodewrap::ChatterNode)
      \endverbatim
    *
    * Above, roscpp_nodewrap_tutorial names the package providing the node
    * implementation, and nodewrap::ChatterNode represents its fully
    * qualified class name (including the namespace). Note that in the
    * meantime, it became deprecated to provide the package name when
    * registering a plugin class with ROS. Here, the package name has however
    * been kept to allow for backwards compatible use (ROS fuerte and earlier)
    * of the ROS pluginlib macros.
    *
    * Note that in ROS, the discovery of nodelet plugin types further requires
    * specific export tags in the package configuration as well as direct
    * build and runtime dependencies on the nodelet package. In particular,
    * the package manifest should therefore contain the below tags:
    *
      \verbatim
      <package>
        ...
        <build_depend>nodelet</build_depend>
        <run_depend>nodelet</run_depend>
        <export>
          <nodelet plugin="${prefix}/nodelet_plugins.xml"/>
        </export>
        ...
      </package>
      \endverbatim
    *
    * The plugin manifest referred to in the package manifest as
    * nodelet_plugins.xml should then be of the form:
    *
      \verbatim
      <library path="lib/libroscpp_nodewrap_tutorial">
        <class name="roscpp_nodewrap_tutorial/ChatterNode" type="nodewrap::Nodelet<nodewrap::ChatterNode>" base_class_type="nodelet::Nodelet">
          <description>
            Example chatter node
          </description>
        </class>
      </library>
      \endverbatim
    *
    * Above, it is important to observe that the templated ROS nodelet wrapper
    * nodewrap::Nodelet must be provided as class type, with the corresponding
    * chatter node implementation nodewrap::ChatterNode being the template
    * parameter.
    *
    * For a complete example configuration, see the ROS package which provides
    * this tutorial. For an example launch configuration, consider the file
    * chatter_nodelets.launch:
    * 
      \verbatim
      <launch>
        <node pkg="nodelet" type="nodelet" name="chatters" args="manager" output="screen">
          <param name="num_worker_threads" value="2"/>
        </node>
        <node name="alice" pkg="nodelet" type="nodelet" args="load roscpp_nodewrap_tutorial/ChatterNode chatters" output="screen">
          <rosparam command="load" file="$(find roscpp_nodewrap_tutorial)/etc/alice.yaml"/>
        </node>
        <node name="bob" pkg="nodelet" type="nodelet" args="load roscpp_nodewrap_tutorial/ChatterNode chatters" output="screen">
          <rosparam command="load" file="$(find roscpp_nodewrap_tutorial)/etc/bob.yaml"/>
        </node>
        ...
      </launch>
      \endverbatim
    */
  
  class ChatterNode :
    public NodeImpl {
  public:
    /** \brief Default constructor
      * 
      * The constructor of the chatter node is left empty.
      */
    ChatterNode();
    
    /** \brief Destructor
      * 
      * The destructor of the chatter node is left empty.
      */
    virtual ~ChatterNode();
  protected:
    /** \brief The ROS publisher advertising the chat topic
      */
    Publisher publisher;
    
    /** \brief The ROS subscriber listening to the chat topic
      */
    Subscriber subscriber;
    
    /** \brief The ROS service server which can be called
      */
    ServiceServer server;
    
    /** \brief The human-readable name of the chatter
      */
    std::string name;
    
    /** \brief True if this chatter initiates chat
      */
    bool initiate;
    
    /** \brief The text message sent by this chatter
      */
    std::string say;
    
    /** \brief Perform chatter node initialization
      * 
      * The initialization of the chatter node involves advertising the
      * chat topic on which this chatter will publish its messages,
      * subscribing to the chat topic on which this chatter will listen
      * for messages from its chat partner, and retrieval of the chat
      * parameters.
      *
      * This is the initializer's implementation:
      * 
        \verbatim
        void ChatterNode::init() {
          publisher = advertise<std_msgs::String>("chat", "/chat", 100,
            boost::bind(&ChatterNode::connect, this, _1));
          NODEWRAP_INFO("Publishing to: %s", publisher.getTopic().c_str());
          subscriber = subscribe("chat", "/chat", 100, &ChatterNode::chat);
          NODEWRAP_INFO("Subscribed to: %s", subscriber.getTopic().c_str());
           
          name = getParam("chat/name", name);
          initiate = getParam("chat/initiate", false);
          say = getParam("chat/say", say);
          
          NODEWRAP_INFO("Hello, my name is %s!", name.c_str());
        }
        \endverbatim
      * 
      * Note the use of the ROS console wrapper macros instead of the native
      * ROS console macros. This ensures portability of the node implementation
      * between the ROS node and ROS nodelet template wrappers.
      * 
      * \see NodeImpl::init
      */
    void init();
    
    /** \brief Perform chatter node cleanup
      * 
      * Cleanup of the chatter node is just bogus. This is the implementation:
      * 
        \verbatim
        void ChatterNode::cleanup() {
          NODEWRAP_INFO("Good bye from %s!", name.c_str());
        }
        \endverbatim
      * 
      * \see NodeImpl::cleanup
      */
    void cleanup();
    
    /** \brief Connect callback
      * 
      * This callback will be invoked as soon as a subscriber, usually the
      * chat partner, connects to the publisher of this chatter node. One
      * of the chatter nodes then initiates the chat by publishing the first
      * text message on its chat topic.
      * 
      * \param[in] pub A publisher which can be used to publish a message to
      *   a single subscriber which, in this case, is the subscriber that
      *   established the connection.
      */
    void connect(const ros::SingleSubscriberPublisher& pub);
    
    /** \brief Chat callback
      * 
      * This callback will be invoked every time a new text message arrived
      * on the topic at which the chatter node's subscriber is listening.
      * 
      * \param[in] msg A const pointer reference to the received text message.
      * 
      * This is the callback's implementation:
      * 
        \verbatim
        void ChatterNode::chat(const std_msgs::String::ConstPtr& msg) {
          NODEWRAP_DEBUG("I heard: [%s]", msg->data.c_str());
          
          std_msgs::StringPtr reply(new std_msgs::String);
          reply->data = say;
          
          publisher.publish(reply);
          NODEWRAP_DEBUG("I said: [%s]", reply->data.c_str());
        }
        \endverbatim
      * 
      * Note that, for portability of the node implementation between the
      * ROS node and the ROS nodelet template wrappers, it is crucial that
      *
      * -# The chat callback employs the const message pointer signature
      *    (as advertised)
      * -# The reply message is instantiated and published as message pointer
      *
      * If, for efficiency reasons, you consider to make the message pointer
      * a member of the node implementation which remains valid throughout
      * the lifetime of this node and shall be reused for publishing, then
      * do not forget to mind thread-safety.
      */
    void chat(const std_msgs::String::ConstPtr& msg);
    
    /** \brief Service call callback
      * 
      * This dummy callback may be invoked by a service client calling the
      * service of this chatter node. It currently exists for compile-time
      * testing of the node implementation's service-related members and
      * therefore does nothing meaningful.
      * 
      * \param[in] request The empty service request sent to the service server
      *   by the service client.
      * \param[in,out] response The empty service response to be returned to
      *   the service client.
      * 
      * This is the callback's implementation:
      * 
        \verbatim
        bool ChatterNode::call(std_srvs::Empty::Request& request,
            std_srvs::Empty::Response& response) {
          NODEWRAP_DEBUG("I have been called");
          return true;
        }
        \endverbatim
      * 
      */
    bool call(std_srvs::Empty::Request& request, std_srvs::Empty::Response&
      response);
  };
};

#endif
