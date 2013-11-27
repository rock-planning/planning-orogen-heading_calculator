require 'orocos'
require 'vizkit'
require 'readline'

include Orocos
Orocos.initialize
Orocos.load_all_typekits

# Content of these language variables must not be german, otherwise '.' and ','
# are mixed reading numbers and a bad_alloc error occurs loading the scenes.
ENV['LANG'] = 'C'
ENV['LC_NUMERIC'] = 'C'

Orocos.run 'heading_calculator::Task' => 'calculator' do
    calc = TaskContext.get 'calculator'
    calc.goal_distance = 2.0
    calc.geometric_resolution = 0.02
    calc.start
  
    pose_writer = calc.pose_samples.writer
    pose_sample = pose_writer.new_sample
    
    # Test: Straight line with length 10.
    points = []
    for i in 0 .. 10
        points << Eigen::Vector3.new( i, 0, 0 )
    end
 
    # generate and write spline 
    traj_writer = calc.trajectory.writer
    traj_sample = traj_writer.new_sample
    trajectory = Types::Base::Trajectory.new 
    trajectory.spline = Types::Base::Geometry::Spline3.interpolate( points ) 
    traj_sample << trajectory
    traj_writer.write(traj_sample)
    
    puts "\nTEST: straight line"
    traj_writer = calc.trajectory.writer
    traj_sample = traj_writer.new_sample
    
    pose_sample.position[0] = 2
    pose_sample.position[1] = -2
    pose_writer.write(pose_sample)
    
    Readline::readline("Press ENTER to proceed ...")
  
    # Test: Smaller 'goal_distance' than 'required_dist_to_goal'.
    puts "\nTEST: Smaller 'goal_distance' than 'required_dist_to_goal"
    calc.goal_distance = 0.5
    calc.required_dist_to_goal = 1.0
    
    pose_sample.position[0] = 0
    pose_sample.position[1] = 0
    pose_writer.write(pose_sample)
    
    Readline::readline("Press ENTER to proceed ...")
    
  
    # Test: Curve around the robot, same distance to (3,0), (4.1) and (3,2).
    puts "\nTEST: Spline length 2 instead of 10, heading 81 instead of 90"
    calc.goal_distance = 4.0
    
    points = []
    points << Eigen::Vector3.new( 0, 0, 0 )
    points << Eigen::Vector3.new( 1, 0, 0 )
    points << Eigen::Vector3.new( 2, 0, 0 )
    points << Eigen::Vector3.new( 3, 0, 0 )
    points << Eigen::Vector3.new( 4, 0, 0 )
    points << Eigen::Vector3.new( 4, 1, 0 )
    points << Eigen::Vector3.new( 4, 2, 0 )
    points << Eigen::Vector3.new( 3, 2, 0 )
    points << Eigen::Vector3.new( 2, 2, 0 )
    points << Eigen::Vector3.new( 1, 2, 0 )
    points << Eigen::Vector3.new( 0, 2, 0 )
    
    trajectory = Types::Base::Trajectory.new 
    trajectory.spline = Types::Base::Geometry::Spline3.interpolate( points ) 
    traj_sample << trajectory
    traj_writer.write(traj_sample)
    
    pose_sample.position[0] = 3
    pose_sample.position[1] = 1
    pose_writer.write(pose_sample)
    
    Readline::readline("Press ENTER to proceed ...")
    
    puts "\nTEST: Rotates the robot -90 degree around Z"
    calc.goal_distance = 4.0
    
    pose_sample.position[0] = 3
    pose_sample.position[1] = 1
    pose_sample.orientation = Eigen::Quaternion.from_angle_axis(-0.5 * Math::PI, Eigen::Vector3.UnitZ)
    pose_writer.write(pose_sample)
    
    Readline::readline("Press ENTER to proceed ...")
    
    puts "\nTEST: Start position is within 'required_dist_to_goal' to the goal"
    calc.goal_distance = 1.0
    calc.required_dist_to_goal = 2.0
    calc.geometric_resolution = 0.02
    
    pose_sample.position[0] = 0
    pose_sample.position[1] = 0
    pose_writer.write(pose_sample)
      
    Readline::readline("Press ENTER to exit ...")
end


