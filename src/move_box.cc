#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <cstdint>

#include "move_box_gazebo_ros/move_box.h"

using namespace gazebo;
using namespace ignition;

MoveBox::MoveBox(): ModelPlugin() {
  this->connect_count = 0;
}

MoveBox::~MoveBox() {
  this->update_connection.reset();


  this->queue.clear();
  this->queue.disable();
  this->rosnode->shutdown();
  this->callback_queue_thread.join();
  delete this->rosnode;
}

void MoveBox::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
  // Store the pointer to the model
  this->model = _parent;
  this->world = _parent->GetWorld();
  gzdbg << "Loading MoveBox plugin" << std::endl;

  this->box = std::make_shared<MoveBoxContainer::Box>();

  // load parameters
  this->robot_namespace = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if(!_sdf->HasElement("bodyName")) {
    gzerr << "MoveBox plugin missing <bodyName>, cannot proceed" << std::endl;
    return;
  }else {
    this->link_name = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  this->link = this->model->GetLink(this->link_name);
  if(!this->link){
    gzerr << "Error: link named " << this->link_name << " does not exist" << std::endl;
    return;
  }

  this->inertial = this->link->GetInertial();
  if(!this->inertial){
    gzerr << "Error: inertial of the link named " << this->link_name << " does not loaded" << std::endl;
    return;
  }
  else {
    this->mass = this->inertial->Mass();
    this->Ixx = this->inertial->IXX();
    this->Iyy = this->inertial->IYY();
    this->Izz = this->inertial->IZZ();
  }

  if (_sdf->HasElement("xyzVelLimit")){
    this->box->vel_limit.Pos() = _sdf->Get<math::Vector3d>("xyzVelLimit");
  }


  if (!_sdf->HasElement("topicNs"))
  {
    gzmsg << "MoveBox plugin missing <topicNs>," 
        "will publish on namespace " << this->link_name << std::endl;
  }
  else {
    this->topic_ns = _sdf->GetElement("topicNs")->Get<std::string>();
  }

  if (!ros::isInitialized())
  {
    gzerr << "A ROS node for Gazebo has not been initialized, unable to load "
      "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in "
      "the gazebo_ros package. If you want to use this plugin without ROS, "
      "set <shouldPublish> to false" << std::endl;
    return;
  }

  this->rosnode = new ros::NodeHandle(this->robot_namespace);
  this->rosnode->setCallbackQueue(&this->queue);

  this->box_sub = this->rosnode->subscribe(
      this->topic_ns + "/cmd", 1, &MoveBox::Box_CB, this);

  // Custom Callback Queue
  this->callback_queue_thread = boost::thread( boost::bind( &MoveBox::QueueThread,this ) );


  this->debug = false;
  if (_sdf->HasElement("debug")) {
    this->debug = _sdf->GetElement("debug")->Get<bool>();
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&MoveBox::OnUpdate, this, _1));
}

void MoveBox::Connect() {
  this->connect_count++;
}

void MoveBox::Disconnect() {
  this->connect_count--;
}

void MoveBox::QueueThread() {
  static const double timeout = 0.01;

  while (this->rosnode->ok())
  {
    this->queue.callAvailable(ros::WallDuration(timeout));
  }
}

// Called by the world update start event
void MoveBox::OnUpdate(const common::UpdateInfo & /*_info*/) {

  // Get Pose and Inertia Parameter
  math::Pose3d p_self = this->link->WorldPose();
  this->box->pose = p_self;
  double mass_self = this->mass;
  double Ixx_self = this->Ixx;
  double Iyy_self = this->Iyy;
  double Izz_self = this->Izz;

  math::Vector3d v_self = this->link->WorldLinearVel();
  math::Vector3d v_cmd(0, 0, 0);
  math::Vector3d force(0, 0, 0);
  math::Vector3d w_self = this->link->WorldAngularVel();
  math::Vector3d w_cmd(0, 0, 0);
  math::Vector3d torque(0, 0, 0);

//      math::Vector3d p_err = p_self.Pos() + p_other.Rot().RotateVector(offset_other.Pos()) - p_other.Pos();
  math::Vector3d p_err = p_self.Pos() - this->box->target_pose.Pos();
//      math::Vector3d p_err_inv = p_other.Rot().Inverse().RotateVector(p_err);
//      math::Vector3d e_err = (p_self.Rot() * offset_other.Rot().Inverse() * p_other.Rot().Inverse()).Euler();
  math::Vector3d e_err = (p_self.Rot() * this->box->target_pose.Rot().Inverse()).Euler();
  
  // Apply calculated force and torque
  v_cmd = 2.5 * -p_err;
  this->LimitVelocity(v_cmd, this->box->vel_limit.Pos());
  force = 10 * (v_cmd - v_self);
  force *= mass_self;
  force.Z() += mass_self * 9.8;

  w_cmd = 2.5 * -e_err;
  torque = 10 * (w_cmd - w_self);
  torque.X() *= Ixx_self;
  torque.Y() *= Iyy_self;
  torque.Z() *= Izz_self;

  this->link->AddForce(force);
  this->link->AddTorque(torque);

  if (this->debug) {
    system("clear");
    std::cout << std::setprecision(3);
    std::cout << "[box_msg] \t" << this->box->target_pose.Pos() << std::endl;
    std::cout << "[mass_self] \t" << mass_self << std::endl;
    std::cout << "[I_self] \t" << Ixx_self << " " << Iyy_self << " " << Izz_self << " " << std::endl;
    std::cout << std::endl;
    std::cout << "[position] \t" << p_self << std::endl;
    std::cout << "[vel_cmd] \t" << v_cmd << std::endl;
    std::cout << "[velocity] \t" << v_self << std::endl;
    std::cout << "[force] \t" << force << std::endl;
    std::cout << std::endl;
    std::cout << "[w_cmd] \t" << w_cmd << std::endl;
    std::cout << "[Ang. Vel.] \t" << w_self << std::endl;
    std::cout << "[torque] \t" << torque << std::endl;
    std::cout << std::endl;
  }
}

void MoveBox::LimitVelocity(math::Vector3d& v_cmd,
                                 math::Vector3d& v_limit) {
  if (v_cmd.X() > v_limit.X())
    v_cmd.X() = v_limit.X();
  if (v_cmd.X() < -v_limit.X())
    v_cmd.X() = -v_limit.X();

  if (v_cmd.Y() > v_limit.Y())
    v_cmd.Y() = v_limit.Y();
  if (v_cmd.Y() < -v_limit.Y())
    v_cmd.Y() = -v_limit.Y();

  if (v_cmd.Z() > v_limit.Z())
    v_cmd.Z() = v_limit.Z();
  if (v_cmd.Z() < -v_limit.Z())
    v_cmd.Z() = -v_limit.Z();
}

void MoveBox::Box_CB(const std_msgs::Float32MultiArray& msg) {
  this->box->target_pose.Pos().X() = msg.data[0];
  this->box->target_pose.Pos().Y() = msg.data[1];
  this->box->target_pose.Pos().Z() = msg.data[2];
  math::Vector3d msg_att_euler(msg.data[3]*D2R, msg.data[4]*D2R, msg.data[5]*D2R);
  this->box->target_pose.Rot().Euler(msg_att_euler);

//  this->box->target_pose.Rot() = 
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MoveBox)
