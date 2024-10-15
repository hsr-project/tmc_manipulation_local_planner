/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.
* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.
NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
/// @brief Sample Goal Generator Class

#include <tmc_simple_path_generator/sample_goal_generator.hpp>

#include <string>
#include <vector>

#include <console_bridge/console.h>

#include <tmc_robot_kinematics_model/numeric_ik_solver.hpp>
#include <tmc_robot_kinematics_model/pinocchio_wrapper.hpp>

#include "common.hpp"

namespace {

// Size check
bool CheckRobotState(const tmc_manipulation_types::RobotState& robot_state) {
  if (robot_state.joint_state.name.size() != robot_state.joint_state.position.size()) {
    return false;
  }
  if (robot_state.multi_dof_joint_state.names.size() != robot_state.multi_dof_joint_state.poses.size()) {
    return false;
  }
  return true;
}

tmc_robot_kinematics_model::IKRequest GenerateIKRequest(
    const Eigen::Affine3d& origin_to_end,
    const tmc_manipulation_types::RobotState& initial_state,
    const tmc_simple_path_generator::SamplingParameters::Ptr& params,
    const std::string& end_frame) {
  tmc_robot_kinematics_model::IKRequest req(
      static_cast<tmc_manipulation_types::BaseMovementType>(params->base_movement_type));
  req.frame_name = end_frame;
  req.frame_to_end = Eigen::Affine3d::Identity();
  req.initial_angle.name = initial_state.joint_state.name;
  req.initial_angle.position = initial_state.joint_state.position;
  req.use_joints = params->joint_names;
  req.weight.resize(params->joint_weights.size() + params->base_weights.size());
  req.weight << params->joint_weights, params->base_weights;
  req.origin_to_base = initial_state.multi_dof_joint_state.poses[0];
  req.ref_origin_to_end = origin_to_end;
  return req;
}

}  // namespace

namespace tmc_simple_path_generator {

// constructor
SampleGoalGeneratorBase::SampleGoalGeneratorBase()
    : ik_solver_loader_("tmc_robot_kinematics_model", "tmc_robot_kinematics_model::IKSolver"),
      is_from_random_initial_state_(true) {}

// Initialize to fix the seed
void SampleGoalGeneratorBase::Initialize(const rclcpp::Node::SharedPtr& node, uint32_t random_seed) {
  engine_.seed(random_seed);

  ik_initial_range_ = std::make_shared<tmc_utils::DynamicParameter<double>>(node, "ik_initial_range", 0.5);
  set_param_handlers_.emplace_back(node->add_on_set_parameters_callback(
      std::bind(&tmc_utils::DynamicParameter<double>::SetParameterCallback, ik_initial_range_, std::placeholders::_1)));

  InitializeImpl(node);
}

// Initialize to generate seed randomly
void SampleGoalGeneratorBase::Initialize(const rclcpp::Node::SharedPtr& node) {
  Initialize(node, std::random_device()());
}

// Sampling the goal posture from joint restraint
bool SampleGoalGeneratorBase::SampleFromJointConstraints(
    const std::vector<tmc_robot_local_planner::IJointConstraint::Ptr>& joint_constraints,
    tmc_manipulation_types::RobotState& goal_state_out) {
  if (!CheckRobotState(goal_state_out)) {
    return false;
  }
  if (joint_constraints.size() == 0) {
    return false;
  }
  auto choosen_constraint = SampleConstraints(joint_constraints, engine_);
  auto constraint_state = choosen_constraint->Sample();
  return OverwriteRobotState(goal_state_out,
                             constraint_state.joint_state.name,
                             constraint_state.multi_dof_joint_state.names,
                             constraint_state.joint_state.position,
                             constraint_state.multi_dof_joint_state.poses);
}

// Sampling the goal posture from the posture restraint
bool SampleGoalGeneratorBase::SampleFromLinkConstraints(
    const tmc_manipulation_types::RobotState& initial_state,
    const std::vector<tmc_robot_local_planner::ILinkConstraint::Ptr>& link_constraints,
    const SamplingParameters::Ptr& params,
    tmc_manipulation_types::RobotState& goal_state_out) {
  if (!CheckRobotState(initial_state)) {
    return false;
  }
  if (link_constraints.size() == 0) {
    return false;
  }

  tmc_manipulation_types::RobotState varianced_initial_state;
  if (is_from_random_initial_state_) {
    if (!AddRandomState(initial_state, params->joint_names, params->base_names, params->joint_limit_upper,
                        params->joint_limit_lower, ik_initial_range_->value(), varianced_initial_state)) {
      return false;
    }
  } else {
    varianced_initial_state = initial_state;
  }

  auto choosen_constraint = SampleConstraints(link_constraints, engine_);
  auto origin_to_ref = choosen_constraint->Sample();
  return SolveIK(origin_to_ref, varianced_initial_state, params,
                 choosen_constraint->GetLinkName(), goal_state_out);
}


// Implementation of initialization
void SampleGoalGenerator::InitializeImpl(const rclcpp::Node::SharedPtr& node) {
  const auto solver_type = tmc_utils::GetParameter<std::string>(
      node, "solver_type", "tmc_robot_kinematics_model/NumericIKSolver");
  ik_solver_ = ik_solver_loader_.createSharedInstance(solver_type);

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
    robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }
  ik_solver_->set_robot_description(robot_description);
}

// Find ROBOT_STATE to realize the terminal posture origin_to_end
bool SampleGoalGenerator::SolveIK(const Eigen::Affine3d& origin_to_end,
                                  const tmc_manipulation_types::RobotState& initial_state,
                                  const SamplingParameters::Ptr& params,
                                  const std::string& end_frame,
                                  tmc_manipulation_types::RobotState& solution_out) {
  const auto req = GenerateIKRequest(origin_to_end, initial_state, params, end_frame);

  tmc_manipulation_types::JointState res;
  Eigen::Affine3d origin_to_hand_result;
  Eigen::Affine3d origin_to_base_solution;
  tmc_robot_kinematics_model::IKResult result;
  try {
    result = ik_solver_->Solve(req, res, origin_to_base_solution, origin_to_hand_result);
  } catch (std::exception& e) {
    CONSOLE_BRIDGE_logWarn("%s", e.what());
    return false;
  }
  if (result == tmc_robot_kinematics_model::kSuccess) {
    solution_out = initial_state;
    if (OverwriteRobotState(solution_out, params->joint_names, params->base_names, res.position,
                            tmc_manipulation_types::PoseSeq(params->base_names.size(), origin_to_base_solution))) {
      CONSOLE_BRIDGE_logDebug("IK success");
      return true;
    } else {
      CONSOLE_BRIDGE_logWarn("IK Failed");
      return false;
    }
  } else {
    CONSOLE_BRIDGE_logDebug("IK Failed %d", result);
    return false;
  }
}


// constructor
SampleGoalGeneratorMultiThread::SampleGoalGeneratorMultiThread()
    : stop_thread_(false), stop_ik_(false), num_of_running_ik_(0) {}

// Destructor
SampleGoalGeneratorMultiThread::~SampleGoalGeneratorMultiThread() {
  {
    std::unique_lock<std::mutex> lock(request_queue_mutex_);
    stop_thread_ = true;
  }
  condition_.notify_all();
  for (auto& worker : workers_) {
    worker.join();
  }
}

// Treatment at the end
void SampleGoalGeneratorMultiThread::Terminate() {
  SampleGoalGeneratorBase::Terminate();

  {
    std::unique_lock<std::mutex> lock(request_queue_mutex_);
    std::queue<Request>().swap(request_queue_);
  }
  {
    std::unique_lock<std::mutex> lock(stop_ik_mutex_);
    stop_ik_ = true;
  }
  while (true) {
    std::unique_lock<std::mutex> lock(num_of_running_ik_mutex_, std::try_to_lock);
    if (lock && (num_of_running_ik_ == 0)) break;
  }
  {
    std::unique_lock<std::mutex> lock(response_queue_mutex_);
    std::queue<Response>().swap(response_queue_);
  }
  {
    std::unique_lock<std::mutex> lock(stop_ik_mutex_);
    stop_ik_ = false;
  }
}

// Implementation of initialization
void SampleGoalGeneratorMultiThread::InitializeImpl(const rclcpp::Node::SharedPtr& node) {
  int32_t thread_pool_size = tmc_utils::GetParameter(node, "thread_pool_size", 8);
  if (thread_pool_size <= 0) {
    throw std::domain_error("thread_pool_size must be positive.");
  }

  const auto solver_type = tmc_utils::GetParameter<std::string>(
      node, "solver_type", "tmc_robot_kinematics_model/NumericIKSolver");

  auto robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description_kinematics", "");
  if (robot_description.empty()) {
      robot_description = tmc_utils::GetParameter<std::string>(node, "robot_description", "");
  }

  for (auto i = 0; i < thread_pool_size; ++i) {
    workers_.emplace_back(
        std::thread(std::bind(&SampleGoalGeneratorMultiThread::SolveIkThread, this, solver_type, robot_description)));
  }
}

// Find ROBOT_STATE to realize the terminal posture origin_to_end
bool SampleGoalGeneratorMultiThread::SolveIK(const Eigen::Affine3d& origin_to_end,
                                             const tmc_manipulation_types::RobotState& initial_state,
                                             const SamplingParameters::Ptr& params,
                                             const std::string& end_frame,
                                             tmc_manipulation_types::RobotState& solution_out) {
  Request req;
  req.request = GenerateIKRequest(origin_to_end, initial_state, params, end_frame);
  req.initial_state = initial_state;

  // Pack REQUEST in a queue
  bool add_request_queue = false;
  {
    std::unique_lock<std::mutex> lock(request_queue_mutex_);
    // Even if you accumulate too much queue, you only need to use memory for waste, so it will be twice as much as the number of threads.
    if (request_queue_.size() < workers_.size() * 2) {
      request_queue_.emplace(req);
      add_request_queue = true;
    }
  }
  if (add_request_queue) condition_.notify_one();

  // Receive if you have Response
  Response res;
  {
    std::unique_lock<std::mutex> lock(response_queue_mutex_);
    if (response_queue_.empty()) {
      return false;
    } else {
      res = std::move(response_queue_.front());
      response_queue_.pop();
    }
  }

  // Response judgment/conversion
  if (res.result != tmc_robot_kinematics_model::kSuccess) {
    CONSOLE_BRIDGE_logDebug("IK Failed %d", res.result);
    return false;
  }

  solution_out = res.initial_state;
  // You can think that params will not change while you are solving IK
  if (OverwriteRobotState(solution_out,
                          params->joint_names, params->base_names,
                          res.joint_solution.position,
                          tmc_manipulation_types::PoseSeq(params->base_names.size(), res.origin_to_base))) {
    CONSOLE_BRIDGE_logDebug("IK success");
    return true;
  } else {
    CONSOLE_BRIDGE_logWarn("IK Failed");
    return false;
  }
}

void SampleGoalGeneratorMultiThread::SolveIkThread(const std::string& solver_type,
                                                   const std::string& robot_description) {
  tmc_robot_kinematics_model::IKSolver::Ptr ik_solver;
  {
    std::unique_lock<std::mutex> lock(request_queue_mutex_);
    ik_solver = ik_solver_loader_.createSharedInstance(solver_type);
    ik_solver->set_robot_description(robot_description);
  }

  std::function<bool()> interrupt = [this]() -> bool{
    std::unique_lock<std::mutex> lock(this->stop_ik_mutex_, std::try_to_lock);
    if (lock) {
      return this->stop_ik_;
    } else {
      return false;
    }
  };

  Request req;
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(request_queue_mutex_);
      condition_.wait(lock, [this]{ return this->stop_thread_ || !this->request_queue_.empty(); });
      if (stop_thread_ && request_queue_.empty()) {
        return;
      }
      req = std::move(request_queue_.front());
      request_queue_.pop();
    }
    Eigen::Affine3d origin_to_hand_result;
    Response res;
    res.initial_state = req.initial_state;
    {
      std::unique_lock<std::mutex> lock(num_of_running_ik_mutex_);
      ++num_of_running_ik_;
    }
    try {
      res.result = ik_solver->Solve(
          req.request, interrupt, res.joint_solution, res.origin_to_base, origin_to_hand_result);
    } catch (std::exception& e) {
      CONSOLE_BRIDGE_logWarn("%s", e.what());
      res.result = tmc_robot_kinematics_model::kFail;
    }
    {
      std::unique_lock<std::mutex> lock(response_queue_mutex_);
      response_queue_.push(std::move(res));
    }
    {
      std::unique_lock<std::mutex> lock(num_of_running_ik_mutex_);
      --num_of_running_ik_;
    }
  }
}

}  // namespace tmc_simple_path_generator

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::SampleGoalGenerator,
                       tmc_simple_path_generator::SampleGoalGeneratorBase);

PLUGINLIB_EXPORT_CLASS(tmc_simple_path_generator::SampleGoalGeneratorMultiThread,
                       tmc_simple_path_generator::SampleGoalGeneratorBase);
