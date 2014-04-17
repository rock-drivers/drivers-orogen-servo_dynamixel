#! /usr/bin/env ruby

require 'orocos'
include Orocos

if !ARGV[0]
    STDERR.puts "usage: test.rb <config_file> <config_name>"
    exit 1
end
config_file = ARGV[0] || "dynamixel.yml" 
configuration = ARGV[1] || "default"

puts "using config file '#{config_file}' with configuration '#{configuration}'"

Orocos.initialize

Orocos::Process.run 'servo_dynamixel::Task' => 'dynamixel_task', 'output' => nil do |p|
    driver = p.task 'dynamixel_task'

    Orocos.apply_conf_file(driver, config_file, [configuration])

    driver.configure
    driver.start

    writer = driver.command.writer

    # wait for input
    puts "enter command: (name, position)"
    while not (cmd = $stdin.readline.chomp).empty? do
	name, position = cmd.split(",")
	position = position.to_f

	joints = Types::Base::Samples::Joints.new
	joints.names << name
	joint_state = Types::Base::JointState.new
	joint_state.position = position
	joints.elements << joint_state
	writer.write joints
    end
end

