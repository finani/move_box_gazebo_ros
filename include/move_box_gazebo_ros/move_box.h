#ifndef MOVE_BOX_H_
#define MOVE_BOX_H_


#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <memory>

#include "move_box_gazebo_ros/move_box_container.h"

using namespace gazebo;
using namespace ignition;
// math::Angle::Pi
float R2D = 180.0 / 3.141592;
float D2R = 3.141592 / 180.0;

class MoveBox : public ModelPlugin {
 public:
  MoveBox();

  ~MoveBox();

  /// \brief Loads the plugin
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  /// \brief Callback for when subscribers connect
  void Connect();

  /// \brief Callback for when subscribers disconnect
  void Disconnect();

  /// \brief Thread to interact with ROS
  void QueueThread();

  /// \brief Called by the world update start event
  void OnUpdate(const common::UpdateInfo & /*_info*/);



  void LimitVelocity(math::Vector3d& v_cmd,
                     math::Vector3d& v_limit);

  void Box_CB(const std_msgs::Float32MultiArray& msg);

  // Pointer to the model
 private:
  physics::ModelPtr model;
  physics::LinkPtr link;
  physics::InertialPtr inertial;
  physics::WorldPtr world;

  std::shared_ptr<MoveBoxContainer::Box> box;

  std::string link_name;
  std::string robot_namespace;
  std::string topic_ns;

  bool debug;
  double mass;
  double Ixx;
  double Iyy;
  double Izz;
  ros::NodeHandle* rosnode;
  ros::Subscriber box_sub;

  geometry_msgs::WrenchStamped wrench_msg;

  private: boost::mutex lock;
  int connect_count;

  // Custom Callback Queue
  ros::CallbackQueue queue;
  boost::thread callback_queue_thread;

  // Pointer to the update event connection
  event::ConnectionPtr update_connection;
};

#endif // MOVE_BOX_H_
