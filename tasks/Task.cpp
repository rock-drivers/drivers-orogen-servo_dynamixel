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
    dynamixel_config.mFilename= _device.value();
    dynamixel_config.mBaudrate = _baudrate.value();
    dynamixel_.setNumberRetries( _package_retry_count.value() );
    dynamixel_.setTimeout(1000);

    if(!dynamixel_.init(&dynamixel_config))
    {
	LOG_ERROR( "Cannot open device '%s'.  %s", _device.value().c_str(), strerror(errno) );
        return false;
    }

    // go through the configuration vector and set up the servos
    uint limit_prop_count = 0;
    BOOST_FOREACH( ServoConfiguration &sc, _servo_config.value() )
    {
	// get the id of the servo
	int id = sc.id; 

	// copy relevant config information to the status 
	ServoStatus status;
	status.id = id;
	status.enabled = false;
	status.positionOffset = sc.positionOffset;
	status.positionScale = sc.positionScale;
	status.positionRange = sc.positionRange;
	status.speedScale = sc.speedScale;
	status.effortScale = sc.effortScale;
	status_map[sc.name] = status;
	
	// add it to the driver
	dynamixel_.addServo(id);

	// and make it active
	dynamixel_.setServoActive(id);

	// read the control table
	if(!dynamixel_.readControlTable())
	    return false;

    base::JointLimits limits = _joint_limits.get();

	// might be that the servo is in an error state (e.g. overload)
	// check if this is so, and try to release it
	int retryCount = _package_retry_count.value();
	while( !dynamixel_.isErrorStatusOk() && retryCount >= 0 )
	{
	    // we can release it be setting the torque limit value
	    dynamixel_.setControlTableEntry("Torque Enable", 0);
	    dynamixel_.setControlTableEntry("Max Torque", 0);
	    dynamixel_.setControlTableEntry("Torque Limit", 1023);

	    // it seems the servo might take a while to release the
	    // lock condition, sleep 100 ms
	    usleep( 1e3 * 100 );
	    retryCount--;

	    if (!dynamixel_.setControlTableEntry("Torque Limit", 1023))
		throw std::runtime_error("Could not reset torque limit");
	}

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

    // set joint limits.
    base::JointLimitRange range;
    if(limits.size() == _servo_config.value().size())
        range = limits[limit_prop_count++];
    else
        LOG_DEBUG("Joint Limits size does not match servo config size. Position Limits will stay as in EEPROM. Moving Speed and ");

    //If no position limits are given, they will stay as they are in EEPROM
    if(range.min.hasPosition()){
        uint16_t cw_angle_limit = (range.min.position + status.positionOffset) * status.positionScale;
        if (!dynamixel_.setControlTableEntry("CW Angle Limit", cw_angle_limit))
            return false;

        LOG_DEBUG("Set min position to %i (%f in radians)", cw_angle_limit, range.min.position);
    }
    else
        LOG_DEBUG("Range has no min position. Will stay as in EEPROM");

    if(range.max.hasPosition()){
        uint16_t ccw_angle_limit = (range.max.position + status.positionOffset) * status.positionScale;
        if (!dynamixel_.setControlTableEntry("CCW Angle Limit", ccw_angle_limit))
            return false;

        LOG_DEBUG("Set max position to %i (%f in radians)", ccw_angle_limit, range.max.position);
    }
    else
        LOG_DEBUG("Range has no max position. Will stay as in EEPROM");

    //If no speed or torque limit is given, they will be set to default values
    uint16_t moving_speed = 1023, torque_limit = 1023;
    if(range.max.hasSpeed())
        moving_speed = status.speedScale * range.max.speed;
    if(range.max.hasEffort())
        torque_limit = status.effortScale * range.max.effort;

    if(!dynamixel_.setControlTableEntry("Moving Speed", moving_speed))
        return false;
    LOG_DEBUG("Set moving speed to %i (%f in radians)", moving_speed, range.max.speed);
    if (!dynamixel_.setControlTableEntry("Torque Limit", torque_limit))
        return false;
    LOG_DEBUG("Set torque limit to %i (%f in radians)", torque_limit, range.max.effort);

	// disable the torque, so make the servo passive
	if(!dynamixel_.setControlTableEntry("Torque Enable", 0))
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
    //note, needs to be a while, as we can receive commands from two different tasks
    //containing only one command. If we control more that one servo we would loose
    //the second command if we use readNewest
    while( _command.read( cmd ) == RTT::NewData )
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

	    // get target joint state
	    const base::JointState &target( cmd[cidx] );

	    if( target.hasPosition() )
	    {
		// enable servo if necessary
		if( status.enabled == false )
		{
		    // enable the servo 
		    dynamixel_.setControlTableEntry("Torque Enable", 1);

		    status.enabled = true;
		}

		// convert the angular position given in radians to
        // Adapt to limit range
        base::JointLimitRange range;
        if(_joint_limits.value().size() == _servo_config.value().size())
            range = _joint_limits.value()[cidx];

        float min = 0, max = status.positionRange;
        if(range.min.hasPosition())
            min = (range.min.position + status.positionOffset) * status.positionScale;
        if(range.max.hasPosition())
            max = (range.max.position + status.positionOffset) * status.positionScale;

        float pos_f = (target.position + status.positionOffset) * status.positionScale;
        uint16_t pos = std::max( std::min( pos_f, max ), min );

        LOG_DEBUG("Position in ticks is: %i, Max: %f, Min: %f", pos, max, min);

		// and write the updated goal position
		if(!dynamixel_.setGoalPosition( pos ))
		{
		    LOG_WARN_S << "Set position " << pos 
			<< "  for servo id " << id << " failed." << std::endl;
		    throw std::runtime_error("could not set target position for servo");
		}
	    }

	    if( target.hasSpeed() )
	    {
		float speed_f = target.speed * status.speedScale;
		uint16_t speed = std::max( std::min( speed_f, 1023.0f ), 0.0f );
		if (!dynamixel_.setControlTableEntry("Moving Speed", speed))
		    throw std::runtime_error("could not set speed value for servo");
	    }

	    if( target.hasEffort() )
	    {
		float effort_f = target.effort * status.effortScale;
		uint16_t effort = std::max( std::min( effort_f, 1023.0f ), 0.0f );
		if( effort == 0 )
		{
		    // disable the servo 
		    dynamixel_.setControlTableEntry("Torque Enable", 0);
		    status.enabled = false;
		}
		else
		{
		    if (!dynamixel_.setControlTableEntry("Max Torque", effort))
			throw std::runtime_error("could not set target effort for servo");
		}
	    }

	    if( !(target.hasPosition() || target.hasEffort() || target.hasSpeed()) )
	    {
		LOG_WARN_S << "Got a command which did not contain a position or effort value." << std::endl;
	    }
	}
    }

    // get joint status and write to output port 
    readJointStatus();
    joint_status.time = base::Time::now();
    _status_samples.write( joint_status );

    // see if we have configuration for the joint_transforms 
    // and the output port for it is connected
    if( !_joint_transform.value().empty() && _transforms.connected() )
    {
	_joint_transform.value().setRigidBodyStates( joint_status, rbs );
	for( size_t i=0; i < rbs.size(); ++i )
	    _transforms.write( rbs[i] );
    }
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

	// read the position values and write the scaled version to the joints status
	uint16_t position;
	if( !dynamixel_.getPresentPosition( &position ) )
	    throw std::runtime_error("Could not read servo position value");
	joint_status[i].position = position / sc.positionScale - sc.positionOffset;

	// same for speed
	uint16_t speed;
	if( !dynamixel_.getControlTableEntry( "Present Speed", &speed ) )
	    throw std::runtime_error("Could not read servo speed value");
	joint_status[i].speed = (speed > 1023 ? 1023 - speed : speed ) / sc.speedScale; 

	// and the load value
	uint16_t effort;
	if( !dynamixel_.getControlTableEntry( "Present Load", &effort ) )
	    throw std::runtime_error("Could not read servo load value");
	joint_status[i].effort = (effort > 1023 ? 1023 - effort : effort ) / sc.effortScale;

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
