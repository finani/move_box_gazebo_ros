#ifndef MOVE_BOX_CONTAINER_H_
#define MOVE_BOX_CONTAINER_H_

#include <iostream>
#include <vector>
#include <memory>
#include <cstdint>

#include <gazebo/common/common.hh>

using namespace gazebo;
using namespace ignition;

class MoveBoxContainer {
 public:
  MoveBoxContainer() {
  }

  static MoveBoxContainer& Get() {
    static MoveBoxContainer instance;
    return instance;
  }

  struct Box {
    math::Pose3d target_pose;
    math::Pose3d pose;
    math::Pose3d vel_limit;
  };
};

#endif // MOVE_BOX_CONTAINER_H_
