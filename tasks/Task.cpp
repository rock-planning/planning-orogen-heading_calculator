/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace heading_calculator;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state), mTrajectory(), mPose(), mGuess(0), mPosSpline(0), 
        mTrajectoryReceived(false)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state), mTrajectory(), mPose(), mGuess(0), 
        mPosSpline(0), mTrajectoryReceived(false)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;
    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
   
    std::vector<base::Trajectory> trajectories;
    if(_trajectory.read(trajectories) == RTT::NewData) {
        RTT::log(RTT::Info) << "Received new trajectory" << RTT::endlog();
        if(trajectories.size() == 0) {
            RTT::log(RTT::Warning) << "Received trajectory vector is empty" << RTT::endlog();
            return;
        }
        mTrajectory = trajectories[0];
        // Assumption: Spline starts at the robot.
        mGuess = 0;
        mTrajectoryReceived = true;
    }
    
    
    if(_pose_samples.readNewest(mPose) == RTT::NewData) {
        if(!mTrajectoryReceived) {
            RTT::log(RTT::Info) << "Trajectory not yet received" << RTT::endlog();
            return;
        }
    
        RTT::log(RTT::Info) << "Recalculate heading" << RTT::endlog();
    
        RTT::log(RTT::Info) << "Spline length: " << mTrajectory.spline.getCurveLength() << RTT::endlog();
        // Calculates the position of the robot on the spline (initially close to 0)
        mPosSpline = mTrajectory.spline.findOneClosestPoint(mPose.position, mGuess, 
                _geometric_resolution.get());
        RTT::log(RTT::Info) << "Spline parameter of position " << mPose.position.transpose() << ": " 
                << mPosSpline << " (guess: " << mGuess << ")" << RTT::endlog();
        mGuess = mPosSpline;
        
        // Advances on the spline, starting at the robot position. 
        // Return: <goal pos on the spline, distance of the path>
        std::pair<double, double> ret_advance = mTrajectory.spline.advance(mPosSpline, 
                _goal_distance.get(),
                _geometric_resolution.get());
        base::Vector3d goal_pos = mTrajectory.spline.getPoint(ret_advance.first);
        RTT::log(RTT::Info) << "Advance " << _goal_distance.get() << " with geometric resolution " << 
                _geometric_resolution.get() << ", goal position on the spline: " << 
                ret_advance.first << ", point " << goal_pos.transpose() << RTT::endlog();
              
             
        
        // Heading will be calculated relative to the robot.
        // mPose.orientation = Eigen::Quaterniond::Identity(); 
        
        // Transform the goal point to the robot or to the reference frame (first received robot pose).
        Eigen::Affine3d w2r;
        w2r = mPose.getTransform().inverse();

        base::Vector3d goal_pos_r = w2r * goal_pos;
        // Porject to 2D!
        goal_pos_r.z() = 0;
        RTT::log(RTT::Info) << "Goal position within the robot frame: " << goal_pos_r.transpose() << RTT::endlog();
            
        // Calculate NWU rotation, angle in radians between the x-axis and the goal vector.
        double angle_rad = acos(goal_pos_r.dot(Eigen::Vector3d::UnitX()) / goal_pos_r.norm());
        // Assign prefix.
        if(goal_pos_r.y() < 0) {
            angle_rad *= -1;
        }
        
        //ignore Z
        Eigen::Vector3d vecToGoal = (mPose.position - goal_pos);
        vecToGoal.z() = 0;
        
        double dist_to_goal = vecToGoal.norm();
        if(_required_dist_to_goal.get() >= 0 && 
                dist_to_goal <= _required_dist_to_goal.get()) {
            _heading.write(base::NaN<double>());
             _heading_debug_deg.write(base::NaN<double>());
        } else {
            // Write to port.
            _heading.write(angle_rad);
            
            // Write degree to debug port.
            _heading_debug_deg.write((angle_rad / M_PI) * 180.0);
        }
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
