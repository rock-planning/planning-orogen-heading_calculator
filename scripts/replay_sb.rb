#! /usr/bin/env ruby
#library for displaying data
require 'vizkit'
require 'readline'
require 'eigen'
require 'rock/bundle'

if !ARGV[0]  then 
    puts "usage: replay.rb log_dir"
    exit
end


#load log file 
log = Orocos::Log::Replay.open(ARGV[0])
Orocos::CORBA::max_message_size = 100000000

log.heading_calc.track(false) 
log.transformer_broadcaster.track(false) 
log.transformer_broadcaster.rename('foo')
log.name_service.deregister 'heading_calc'
log.name_service.deregister 'transformer_broadcaster'
#log.name_service.deregister 'local_planner'

Bundles.initialize
Bundles.transformer.load_conf(Bundles.find_file('config', 'transforms_scripts.rb'))

Bundles.run 'heading_calculator::Task' => 'heading_calc', :valgrind => false, :output => nil do |p|
    
    heading_calc = Bundles::get 'heading_calc'
    global_planner = Bundles::get 'global_planner'
    velodyne_slam = Bundles::get 'velodyne_slam'


    global_planner.trajectory_spline_out.connect_to(heading_calc.trajectory)
    velodyne_slam.pose_samples.connect_to(heading_calc.pose_samples, :type => :buffer, :size => 100)
    
    heading_calc.apply_conf(['default'])
    
    Bundles.transformer.setup( heading_calc )
    heading_calc.configure()
    heading_calc.start()

    Vizkit.control log

    
    Vizkit.display velodyne_slam.pose_samples, :widget => Vizkit.default_loader.RigidBodyStateVisualization
    Vizkit.display global_planner.trajectory_spline_out
#     Vizkit.display heading_calc.heading_debug_deg

    Vizkit.exec()
end

