#include <robot_gazebo_plugin/robot_gazebo_plugin.h>

namespace robot_gazebo_plugin
{
ControlPlugin::ControlPlugin()
{
    std::cout << "Start Robot Simulation ..." << std::endl;

    // Create arm shared memory.
    arm_shm_id = shm::create_shm(robot->arm_shm_key_, &arm_shm_ptr);
    if (arm_shm_id != SHM_STATE_NO)
    {
        std::cout << "Create arm shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Create arm shared memory failed." << std::endl;
    }

    // Set the initial values of arm shared memory.
    for (unsigned int j=0; j< robot->arm_dof_; j++)
    {
        arm_shm_ptr->cur_arm_joint_positions_[j] = robot->cur_arm_joint_positions_[j];
        arm_shm_ptr->cur_arm_joint_velocities_[j] = robot->cur_arm_joint_velocities_[j];
        arm_shm_ptr->cur_arm_joint_efforts_[j] = robot->cur_arm_joint_efforts_[j];

        arm_shm_ptr->cmd_arm_joint_positions_[j] = robot->cmd_arm_joint_positions_[j];
        arm_shm_ptr->cmd_arm_joint_velocities_[j] = robot->cmd_arm_joint_velocities_[j];
        arm_shm_ptr->cmd_arm_joint_efforts_[j] = robot->cmd_arm_joint_efforts_[j];

        arm_shm_ptr->arm_control_modes_[j] = robot->arm_control_modes_[j];
    }

    // Create arm semaphore.
    arm_sem_id = sem::create_semaphore(robot->arm_sem_key_);
    if (arm_sem_id != SEM_STATE_NO)
    {
        std::cout << "Create arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Create arm semaphore failed." << std::endl;
    }

#if END_EFFECTOR_TRUE
    // Create end-effector shared memory.
    end_eff_shm_id = shm::create_shm(robot->end_eff_shm_key_, &end_eff_shm_ptr);
    if (end_eff_shm_id != SHM_STATE_NO)
    {
        std::cout << "Create end-effector shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Create end-effector shared memory failed." << std::endl;
    }

    // Set the initial values of end-effector shared memory.
    for (unsigned int j=0; j< robot->end_eff_dof_; j++)
    {
        end_eff_shm_ptr->cur_end_eff_joint_positions_[j] = robot->cur_end_eff_joint_positions_[j];
        end_eff_shm_ptr->cur_end_eff_joint_velocities_[j] = robot->cur_end_eff_joint_velocities_[j];
        end_eff_shm_ptr->cur_end_eff_joint_efforts_[j] = robot->cur_end_eff_joint_efforts_[j];

        end_eff_shm_ptr->cmd_end_eff_joint_positions_[j] = robot->cmd_end_eff_joint_positions_[j];
        end_eff_shm_ptr->cmd_end_eff_joint_velocities_[j] = robot->cmd_end_eff_joint_velocities_[j];
        end_eff_shm_ptr->cmd_end_eff_joint_efforts_[j] = robot->cmd_end_eff_joint_efforts_[j];

        end_eff_shm_ptr->end_eff_control_modes_[j] = robot->end_eff_control_modes_[j];
    }

    // Create end-effector semaphore.
    end_eff_sem_id = sem::create_semaphore(robot->end_eff_sem_key_);
    if (end_eff_sem_id != SEM_STATE_NO)
    {
        std::cout << "Create end-effector semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Create end-effector semaphore failed." << std::endl;
    }
#endif
}

ControlPlugin::~ControlPlugin()
{ 
    // Delete arm semaphore.
    if (sem::delete_semaphore(arm_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Delete arm semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete arm semaphore failed." << std::endl;
    }

    // Release arm shared memory.
    if (shm::release_shm(arm_shm_id, &arm_shm_ptr) == SHM_STATE_OK)
    {
        std::cout << "Release arm shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Release arm shared memory failed." << std::endl;
    }

#if END_EFFECTOR_TRUE
    // Delete end-effector semaphore.
    if (sem::delete_semaphore(end_eff_sem_id) == SEM_STATE_OK)
    {
        std::cout << "Delete end-effector semaphore successfully." << std::endl;
    }
    else
    {
        std::cout << "Delete end-effector semaphore failed." << std::endl;
    }

    // Release end-effector shared memory.
    if (shm::release_shm(end_eff_shm_id, &end_eff_shm_ptr) == SHM_STATE_OK)
    {
        std::cout << "Release end-effector shared memory successfully." << std::endl;
    }
    else
    {
        std::cout << "Release end-effector shared memory failed." << std::endl;
    }
#endif

    std::cout << "Simulation has been finished successfully by user." << std::endl;
}

// Overloaded Gazebo entry point
void ControlPlugin::Load(gazebo::physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  std::cout << "Load control plugin ..." << std::endl;

  // Save pointers to the model
  parent_model_ = parent;

  // Get the Gazebo simulation rate
  double sim_rate = parent_model_->GetWorld()->Physics()->GetRealTimeUpdateRate();
  std::cout << "Simulation rate: " << sim_rate << " Hz." << std::endl;

  // Get arm joint pointer from model
  for (unsigned int j=0; j< robot->arm_dof_; j++)
  {
     gazebo::physics::JointPtr armjoint = parent_model_->GetJoint(robot->arm_joint_names_[j]);
     arm_joints_.push_back(armjoint);
  }

#if END_EFFECTOR_TRUE
  // Get end-effector joint pointer from model
  for (unsigned int j=0; j< robot->end_eff_dof_; j++)
  {
     gazebo::physics::JointPtr endeffjoint = parent_model_->GetJoint(robot->end_eff_joint_names_[j]);
     end_eff_joints_.push_back(endeffjoint);
  }
#endif

  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind(&ControlPlugin::Update, this));

  std::cout << "Load control plugin successfully." << std::endl;
}

// Called by the world update start event
void ControlPlugin::Update()
{
  /* Note: 
     We must do writing operation, and then do reading operation in this function.
     Also, we must do reading operation, and then do writing operation in ROS2 controller manager.
     Doing this can form a closed-loop system.
  */

  // Get arm semaphore.
  sem::semaphore_p(arm_sem_id);
  for (unsigned int j=0; j< robot->arm_dof_; j++)
  {
    // Read control modes and commands from shared memory, then write them into gazebo.
    if ((robot->arm_control_modes_[j]=arm_shm_ptr->arm_control_modes_[j]) & robot->position_mode_)
    {
      arm_joints_[j]->SetPosition(0, robot->cmd_arm_joint_positions_[j]=arm_shm_ptr->cmd_arm_joint_positions_[j]);
    }
    else if ((robot->arm_control_modes_[j]=arm_shm_ptr->arm_control_modes_[j]) & robot->velocity_mode_)
    {
      arm_joints_[j]->SetVelocity(0, robot->cmd_arm_joint_velocities_[j]=arm_shm_ptr->cmd_arm_joint_velocities_[j]);
    }
    else if ((robot->arm_control_modes_[j]=arm_shm_ptr->arm_control_modes_[j]) & robot->effort_mode_)
    {
      arm_joints_[j]->SetForce(0, robot->cmd_arm_joint_efforts_[j]=arm_shm_ptr->cmd_arm_joint_efforts_[j]);
    }
    
    // Read current robot arm joint states from gazebo, then write them into shared memory.
    arm_shm_ptr->cur_arm_joint_positions_[j] = robot->cur_arm_joint_positions_[j] = arm_joints_[j]->Position(0);
    arm_shm_ptr->cur_arm_joint_velocities_[j] = robot->cur_arm_joint_velocities_[j] = arm_joints_[j]->GetVelocity(0);
    arm_shm_ptr->cur_arm_joint_efforts_[j] = robot->cur_arm_joint_efforts_[j] = arm_joints_[j]->GetForce(0u);
  }
  // Release arm semaphore.
  sem::semaphore_v(arm_sem_id);

#if END_EFFECTOR_TRUE
  // Get end-effector semaphore.
  sem::semaphore_p(end_eff_sem_id);
  for (unsigned int j=0; j< robot->end_eff_dof_; j++)
  {
    // Read control modes and commands from shared memory, then write them into gazebo.
    if ((robot->end_eff_control_modes_[j]=end_eff_shm_ptr->end_eff_control_modes_[j]) & robot->position_mode_)
    {
      end_eff_joints_[j]->SetPosition(0, robot->cmd_end_eff_joint_positions_[j]=end_eff_shm_ptr->cmd_end_eff_joint_positions_[j]);
    }
    else if ((robot->end_eff_control_modes_[j]=end_eff_shm_ptr->end_eff_control_modes_[j]) & robot->velocity_mode_)
    {
      end_eff_joints_[j]->SetVelocity(0, robot->cmd_end_eff_joint_velocities_[j]=end_eff_shm_ptr->cmd_end_eff_joint_velocities_[j]);
    }
    else if ((robot->end_eff_control_modes_[j]=end_eff_shm_ptr->end_eff_control_modes_[j]) & robot->effort_mode_)
    {
      end_eff_joints_[j]->SetForce(0, robot->cmd_end_eff_joint_efforts_[j]=end_eff_shm_ptr->cmd_end_eff_joint_efforts_[j]);
    }
    
    // Read current end-effector joint states from gazebo, then write them into shared memory.
    end_eff_shm_ptr->cur_end_eff_joint_positions_[j] = robot->cur_end_eff_joint_positions_[j] = end_eff_joints_[j]->Position(0);
    end_eff_shm_ptr->cur_end_eff_joint_velocities_[j] = robot->cur_end_eff_joint_velocities_[j] = end_eff_joints_[j]->GetVelocity(0);
    end_eff_shm_ptr->cur_end_eff_joint_efforts_[j] = robot->cur_end_eff_joint_efforts_[j] = end_eff_joints_[j]->GetForce(0u);
  }
  // Release end-effector semaphore.
  sem::semaphore_v(end_eff_sem_id);
#endif
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ControlPlugin)

} // end namespace gazebo_plugin
