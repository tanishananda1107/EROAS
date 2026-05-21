// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <uuv_gazebo_ros_plugins/JointStatePublisher.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/Joint.hh>

namespace uuv_simulator_ros
{
GZ_ADD_PLUGIN(JointStatePublisher)

JointStatePublisher::JointStatePublisher()
    : updateRate(50.0),
      updatePeriod(0.02),
      modelEntity(gz::sim::kNullEntity)
{
}

JointStatePublisher::~JointStatePublisher()
{
}

void JointStatePublisher::Configure(const gz::sim::Entity &_entity,
                                    const std::shared_ptr<const sdf::Element> &_sdf,
                                    gz::sim::EntityComponentManager &_ecm,
                                    gz::sim::EventManager &_eventManager)
{
  this->modelEntity = _entity;

  if (this->modelEntity == gz::sim::kNullEntity)
  {
    gzerr << "Invalid model entity" << std::endl;
    return;
  }

  // Get the model name for the namespace
  this->robotNamespace = _ecm.Component<gz::sim::components::ModelName>(this->modelEntity)->Data();

  // Initialize ROS 2 node
  this->node = std::make_shared<rclcpp::Node>("joint_state_publisher");

  // Retrieve the namespace used to publish the joint states
  if (_sdf->HasElement("robotNamespace"))
  {
    this->robotNamespace = _sdf->Get<std::string>("robotNamespace");
  }
  else
  {
    this->robotNamespace = "/" + this->robotNamespace;
  }

  if (this->robotNamespace[0] != '/')
  {
    this->robotNamespace = "/" + this->robotNamespace;
  }

  gzmsg << "JointStatePublisher::robotNamespace=" << this->robotNamespace << std::endl;

  if (_sdf->HasElement("updateRate"))
  {
    this->updateRate = _sdf->Get<double>("updateRate");
  }
  else
  {
    this->updateRate = 50.0;
  }

  gzmsg << "JointStatePublisher::Retrieving moving joints:" << std::endl;
  this->movingJoints.clear();
  this->updatePeriod = 1.0 / this->updateRate;

  // Get all joints from the model
  auto jointEntities = _ecm.ChildrenByComponents(
    this->modelEntity, gz::sim::components::Joint());

  for (const auto &jointEntity : jointEntities)
  {
    auto jointNameComp = _ecm.Component<gz::sim::components::JointName>(jointEntity);
    if (!jointNameComp)
    {
      continue;
    }

    std::string jointName = jointNameComp->Data();

    auto jointLowerLimitComp = _ecm.Component<gz::sim::components::JointLowerLimit>(jointEntity);
    auto jointUpperLimitComp = _ecm.Component<gz::sim::components::JointUpperLimit>(jointEntity);

    double lowerLimit = 0.0;
    double upperLimit = 0.0;

    if (jointLowerLimitComp)
    {
      lowerLimit = jointLowerLimitComp->Data()[0];
    }
    if (jointUpperLimitComp)
    {
      upperLimit = jointUpperLimitComp->Data()[0];
    }

    if (lowerLimit == 0.0 && upperLimit == 0.0)
    {
      continue;
    }

    this->movingJoints.push_back(jointName);
    gzmsg << "\t- " << jointName << std::endl;
  }

  GZ_ASSERT(this->updateRate > 0.0, "Update rate must be positive");

  // Advertise the joint states topic
  this->jointStatePub = this->node->create_publisher<sensor_msgs::msg::JointState>(
    this->robotNamespace + "/joint_states", 1);

  this->lastUpdate = this->node->get_clock()->now();
}

void JointStatePublisher::Update(const gz::sim::UpdateInfo &_info,
                                 gz::sim::EntityComponentManager &_ecm)
{
  auto simTime = rclcpp::Time(
    static_cast<uint64_t>(_info.simTime.Double() * 1e9),
    RCL_ROS_TIME);

  if ((simTime - this->lastUpdate).seconds() >= this->updatePeriod)
  {
    this->PublishJointStates(_ecm);
    this->lastUpdate = simTime;
  }
}

void JointStatePublisher::PublishJointStates(const gz::sim::EntityComponentManager &_ecm)
{
  auto modelEntity = _ecm.Component<gz::sim::components::ModelName>(this->modelEntity);
  if (!modelEntity)
  {
    return;
  }

  auto jointEntities = _ecm.ChildrenByComponents(
    this->modelEntity, gz::sim::components::Joint());

  sensor_msgs::msg::JointState jointState;

  jointState.name.resize(jointEntities.size());
  jointState.position.resize(jointEntities.size());
  jointState.velocity.resize(jointEntities.size());
  jointState.effort.resize(jointEntities.size());

  int i = 0;
  for (const auto &jointEntity : jointEntities)
  {
    auto jointNameComp = _ecm.Component<gz::sim::components::JointName>(jointEntity);
    if (!jointNameComp)
    {
      continue;
    }

    std::string jointName = jointNameComp->Data();

    if (!this->IsIgnoredJoint(jointName))
    {
      jointState.name[i] = jointName;

      auto jointPosComp = _ecm.Component<gz::sim::components::JointPosition>(jointEntity);
      auto jointVelComp = _ecm.Component<gz::sim::components::JointVelocity>(jointEntity);

      if (jointPosComp)
      {
        jointState.position[i] = jointPosComp->Data()[0];
      }
      else
      {
        jointState.position[i] = 0.0;
      }

      if (jointVelComp)
      {
        jointState.velocity[i] = jointVelComp->Data()[0];
      }
      else
      {
        jointState.velocity[i] = 0.0;
      }

      jointState.effort[i] = 0.0;
    }
    else
    {
      jointState.name[i] = jointName;
      jointState.position[i] = 0.0;
      jointState.velocity[i] = 0.0;
      jointState.effort[i] = 0.0;
    }

    ++i;
  }

  jointState.header.stamp = this->node->get_clock()->now();
  this->jointStatePub->publish(jointState);
}

bool JointStatePublisher::IsIgnoredJoint(const std::string &_jointName)
{
  if (this->movingJoints.empty())
  {
    return true;
  }

  for (const auto &joint : this->movingJoints)
  {
    if (_jointName.compare(joint) == 0)
    {
      return false;
    }
  }

  return true;
}
}
