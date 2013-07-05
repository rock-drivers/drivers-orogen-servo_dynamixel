/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <boost/foreach.hpp>
#include <base/Logging.hpp>

using namespace servo_dynamixel;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
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

    // try to open the device first
    Dynamixel::Configuration dynamixel_config;
    dynamixel_config.mFilename= ( _device.value() );
    dynamixel_config.mBaudrate = 57600;
    dynamixel_.setTimeout(10000);

    if(!dynamixel_.init(&dynamixel_config))
    {
	LOG_ERROR( "Cannot open device '%s'.  %s", _device.value().c_str(), strerror(errno) );
        return false;
    }

    // go through the configuration vector and set up the servos
    BOOST_FOREACH( ServoConfiguration &sc, _servo_config.value() )
    {
	// get the id of the servo
	int id = sc.id; 

	// copy relevant config information to the status 
	ServoStatus status;
	status.id = id;
	status.enabled = false;
	status.zero = sc.zero;
	status.scale = sc.scale;
	status_map[sc.name] = status;
	
	// add it to the driver
	dynamixel_.addServo(id);

	// and make it active
	dynamixel_.setServoActive(id);

	// read the control table
	if(!dynamixel_.readControlTable())
	    return false;

	// and write an info about the setup
	uint16_t model, firmware, sid;
	dynamixel_.getControlTableEntry("Model Number", &model);
	dynamixel_.getControlTableEntry("Version of Firmware", &firmware);
	dynamixel_.getControlTableEntry("ID", &sid);
	LOG_INFO_S 
	   << "Found servo model " << model
	   << " firmware " << firmware
	   << " at id " << sid 
	   << std::endl;

	// disable the torque, so make the servo passive
	if(!dynamixel_.setControlTableEntry("Torque Enable", 0))
	    return false;

	// set control value A,B,C,D,E (see RX-28 manual)
	if (!dynamixel_.setControlTableEntry("CW Compliance Slope", sc.cwComplianceSlope ))
	    return false;
	if (!dynamixel_.setControlTableEntry("CW Compliance Margin", sc.cwComplianceMargin))
	    return false;
	if (!dynamixel_.setControlTableEntry("CCW Compliance Margin", sc.ccwComplianceMargin))
	    return false;
	if (!dynamixel_.setControlTableEntry("CCW Compliance Slope", sc.ccwComplianceSlope))
	    return false;
	if (!dynamixel_.setControlTableEntry("Punch", sc.punch))
	    return false;

	// set the limits 
	// TODO make this configurable 
	if (!dynamixel_.setControlTableEntry("CW Angle Limit", 0))
	    return false;
	if(!dynamixel_.setControlTableEntry("CCW Angle Limit", 1023))
	    return false;
	if (!dynamixel_.setControlTableEntry("Torque Limit", 1023))
	    return false;
	if(!dynamixel_.setControlTableEntry("Moving Speed", 1023))
	    return false;
    }

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

    // see if we got some commands
    base::commands::Joints cmd;
    if( _command.readNewest( cmd ) == RTT::NewData )
    {
	// go through all the cmd entries
	for( size_t cidx = 0; cidx < cmd.size(); ++cidx )
	{
	    // try to find the name of the joint in the map
	    std::map<std::string, ServoStatus>::iterator mi = 
		status_map.find( cmd.names[cidx] );

	    if( mi == status_map.end() )
	    {
		LOG_ERROR_S 
		    << "There is no servo with the name '"
		    << cmd.names[cidx] << "' in the configuration."
		    << std::endl;

		throw std::runtime_error("Command/configuration mismatch");
	    }

	    // get the servo configuration
	    ServoStatus &status( mi->second );

	    // get the servo id from the map
	    int id = status.id;

	    // and make it active
	    dynamixel_.setServoActive(id);

	    // enable servo if necessary
	    if( status.enabled == false )
	    {
		// enable the servo 
		dynamixel_.setControlTableEntry("Torque Enable", 1);

		status.enabled = true;
	    }

	    // get target joint state
	    base::JointState &target( cmd.states[cidx] );

	    if( target.hasPosition() )
	    {
		// convert the angular position given in radians to
		// the position range of the dynamixel
		uint16_t pos = (target.position + status.zero) * status.scale;

		// and write the updated goal position
		if(!dynamixel_.setGoalPosition( pos ))
		{
		    LOG_WARN_S << "Set position " << pos 
			<< "  for servo id " << id << " failed." << std::endl;
		    throw std::runtime_error("could not set target position for servo");
		}
	    }
	    else
	    {
		LOG_WARN_S << "Got a command which did not contain a position value." << std::endl;
	    }
	}
    }

    readJointStatus();

    // write to output port
    _status_samples.write( joint_status );
}

void Task::readJointStatus()
{
    // setup the result type
    if( joint_status.size() != status_map.size() );
    {
	joint_status.resize( status_map.size() );

	// fill in the joint names
	size_t i = 0;
	for( std::map<std::string, ServoStatus>::iterator mi = 
		status_map.begin(); mi != status_map.end(); ++mi )
	{
	    joint_status.names[i] = mi->first;
	    ++i;
	}
    }

    // now read out the status of all the servos
    // and write into the joint_status array
    // we use the same order as the status_map
    size_t i = 0;
    for( std::map<std::string, ServoStatus>::iterator mi = 
	    status_map.begin(); mi != status_map.end(); ++mi )
    {
	ServoStatus &sc( mi->second );

	// get the id of the servo
	int id = sc.id; 

	// and make it active
	dynamixel_.setServoActive(id);

	// read the position values
	uint16_t position;
	if( !dynamixel_.getPresentPosition( &position ) )
	    throw std::runtime_error("Could not read servo position value");

	// scale and set in the result structure
	joint_status.states[i].position = position / sc.scale - sc.zero;
	++i;
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    for( std::map<std::string, ServoStatus>::iterator mi = 
	    status_map.begin(); mi != status_map.end(); ++mi )
    {
	ServoStatus &sc( mi->second );

	// get the id of the servo
	int id = sc.id; 

	// and make it active
	dynamixel_.setServoActive(id);

	// disable all the servos 
	dynamixel_.setControlTableEntry("Torque Enable", 0);

	// set disabled flag
	sc.enabled = false;
    }
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    // clear configuration structures
    status_map.clear();
    joint_status.clear();
}
