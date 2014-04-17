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
    dynamixel_.setTimeout(_timeout.value());

    if(!dynamixel_.init(&dynamixel_config))
    {
        LOG_ERROR( "Cannot open device '%s'.  %s", _device.value().c_str(), strerror(errno) );
        return false;
    }

    status_map.clear();
    
    // go through the configuration vector and set up the servos
    uint limit_prop_count = 0;
    BOOST_FOREACH( ServoConfiguration &sc, _servo_config.value() )
    {
        // get the id of the servo
        int id = sc.id;

        // copy relevant config information to the status
        ServoStatus status;
        status.id = id;
        status.enabled = (bool)_keep_torque_enabled.value();
        status.positionOffset = sc.positionOffset;
        status.positionScale = sc.positionScale;
        status.positionRange = sc.positionRange;
        status.speedScale = sc.speedScale;
        status.effortScale = sc.effortScale;

        // add it to the driver
        dynamixel_.addServo(id);

        // and make it active
        dynamixel_.setServoActive(id);

        // read the control table
        if(!dynamixel_.readControlTable())
            return false;

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

        base::JointLimits limits = _joint_limits.get();
        ServoLimits servo_limits;

        // set joint limits
        base::JointLimitRange range;
        if(limits.size() == _servo_config.value().size())
            range = limits[limit_prop_count++];

        //If a limit is not given it will be set to default (Position to (0,1023), max speed to 1023, torque limit to 1023)
        if(range.min.hasPosition())
            servo_limits.min_pos = (range.min.position + status.positionOffset) * status.positionScale;
        else
            servo_limits.min_pos = 0;

        if(range.max.hasPosition())
            servo_limits.max_pos = (range.max.position + status.positionOffset) * status.positionScale;
        else
            servo_limits.max_pos = status.positionRange;

        if(range.max.hasSpeed())
            servo_limits.max_speed = status.speedScale * range.max.speed;
        else
            servo_limits.max_speed = 1023;

        if(range.max.hasEffort())
            servo_limits.max_effort = status.effortScale * range.max.effort;
        else
            servo_limits.max_effort = 1023;

        LOG_DEBUG("Setting CW Angle limit to %i", servo_limits.min_pos);
        LOG_DEBUG("Setting CCW Angle limit to %i", servo_limits.max_pos);;
        LOG_DEBUG("Setting Moving speed to %i", servo_limits.max_speed);
        LOG_DEBUG("Setting Torque limit to %i", servo_limits.max_effort);

        if (!dynamixel_.setControlTableEntry("CW Angle Limit", servo_limits.min_pos))
            return false;
        if (!dynamixel_.setControlTableEntry("CCW Angle Limit", servo_limits.max_pos))
            return false;
        if(!dynamixel_.setControlTableEntry("Moving Speed", servo_limits.max_speed))
            return false;
        if (!dynamixel_.setControlTableEntry("Torque Limit", servo_limits.max_effort))
            return false;

        // disable the torque, so make the servo passive
        if(!dynamixel_.setControlTableEntry("Torque Enable", _keep_torque_enabled.value()))
            return false;

        status.limits = servo_limits;
        status_map[sc.name] = status;
    }

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    return true;
}

struct ServoStatusWithCommand
{
    ServoStatusWithCommand(ServoStatus &status) : status(&status)
    {
    }
    ServoStatusWithCommand() : status(NULL)
    {
    }
    ServoStatus *status;
    base::JointState cmd;
};
    
void Task::updateHook()
{
    _act_cycle_time.write((base::Time::now() - stamp_).toSeconds());
    stamp_ = base::Time::now();

    TaskBase::updateHook();

    // see if we got some commands
    base::commands::Joints cmd;
    std::map<std::string, ServoStatusWithCommand> cmdMap;
    
    //note, needs to be a while, as we can receive commands from two different tasks
    //containing only one command. If we control more that one servo we would loose
    //the second command if we use readNewest
    
    //first we accumulate all commands
    while( _command.read( cmd ) == RTT::NewData )
    {
        // go through all the cmd entries
        for( size_t cidx = 0; cidx < cmd.size(); ++cidx )
        {
            std::string &jointName(cmd.names[cidx]);

            std::map<std::string, ServoStatusWithCommand>::iterator cmdIt = cmdMap.find(jointName);
            if( cmdIt == cmdMap.end() )
            {
                // try to find the name of the joint in the map
                std::map<std::string, ServoStatus>::iterator mi =
                        status_map.find( jointName );

                if( mi == status_map.end() )
                {
                    LOG_ERROR_S
                            << "There is no servo with the name '"
                            << jointName << "' in the configuration."
                            << std::endl;

                    throw std::runtime_error("Command/configuration mismatch");
                }

                ServoStatusWithCommand statusWCmd(mi->second);
                statusWCmd.cmd = cmd[cidx];
                cmdMap.insert(std::make_pair(jointName, statusWCmd));
            }
            else
            {
                cmdIt->second.cmd = cmd[cidx];
            }
        }
    }

    for(std::map<std::string, ServoStatusWithCommand>::const_iterator it = cmdMap.begin(); it != cmdMap.end(); it++)
    {
        // get the servo configuration
        ServoStatus &status( *(it->second.status) );

        // get the servo id from the map
        int id = status.id;

        // and make it active
        dynamixel_.setServoActive(id);

        // get target joint state
        const base::JointState &target( it->second.cmd );
        const ServoLimits &servoLimit(status.limits);
        
        if( target.hasPosition() )
        {
            // enable servo if necessary
            if( status.enabled == false )
            {
                // enable the servo
                dynamixel_.setControlTableEntry("Torque Enable", 1);
                status.enabled = true;
            }

            uint16_t pos = (target.position + status.positionOffset) * status.positionScale;

            //Check joint limits
            if(_cap_at_limits.value())
                pos = std::max( std::min( pos, servoLimit.max_pos ), servoLimit.min_pos);
            else
            {
                if(pos < servoLimit.min_pos||
                    pos > servoLimit.max_pos)
                {
                    LOG_ERROR("Target position of servo %i is out of bounds: Min Pos: %i, Max Pos: %i, Target Pos: %i", id, servoLimit.min_pos, servoLimit.max_pos, pos);
                    throw std::invalid_argument("Target position out of bounds");
                }
            }

            // and write the updated goal position
            if(!dynamixel_.setGoalPosition( pos ))
            {
                LOG_ERROR("Set position %i (%f rad) for servo id %i failed! Min Pos: %f, Max Pos: %f, Offset: %f",
                          pos, target.position, id, servoLimit.min_pos, servoLimit.max_pos, status.positionOffset);
                printErrorStatus(dynamixel_.getErrorStatus());
                throw std::runtime_error("could not set target position for servo");
            }
        }

        if( target.hasSpeed() )
        {
            uint16_t speed = target.speed * status.speedScale;
            if(_cap_at_limits.get())
                speed = std::max( std::min( speed, servoLimit.max_speed ), (uint16_t)0);
            else
            {
                if(speed < 0 ||
                   speed > servoLimit.max_speed)
                {
                    LOG_ERROR("Target speed of servo %i is out of bounds: Min Speed: %i, Max Speed: %i, Target Speed: %i", id, 0, servoLimit.max_speed, speed);
                    throw std::invalid_argument("Target Speed out of bounds");
                }
            }

            if (!dynamixel_.setControlTableEntry("Moving Speed", speed)){
                LOG_ERROR("Set speed %i for servo id %i failed. Max speed is: %f", speed, id, servoLimit.max_speed);
                printErrorStatus(dynamixel_.getErrorStatus());
                throw std::runtime_error("could not set speed value for servo");
            }
        }

        if( target.hasEffort() )
        {
            uint16_t effort = target.effort * status.effortScale;

            if(_cap_at_limits.get())
                effort = std::max( std::min( effort, servoLimit.max_effort ), (uint16_t)0);
            else
            {
                if(effort < 0 ||
                    effort > servoLimit.max_effort)
                {
                    LOG_ERROR("Target effort of servo %i is out of bounds: Min effort: %i, Max effort: %i, Target effort: %i", id, 0, servoLimit.max_effort, effort);
                    throw std::invalid_argument("Target effort out of bounds");
                }
            }

            if( effort == 0 )
            {
                // disable the servo
                if(!dynamixel_.setControlTableEntry("Torque Enable", 0)){
                    LOG_ERROR("Set Torque Enable 0 for servo id %i failed", id);
                    printErrorStatus(dynamixel_.getErrorStatus());
                    throw std::runtime_error("Could not set target effort for servo");
                }
                status.enabled = false;
            }
            else
            {
                if (!dynamixel_.setControlTableEntry("Max Torque", effort)){
                    LOG_ERROR("Set effort %i for servo id %i failed", effort, id);
                    printErrorStatus(dynamixel_.getErrorStatus());
                    throw std::runtime_error("could not set target effort for servo");
                }
            }
        }

        if( !(target.hasPosition() || target.hasEffort() || target.hasSpeed()) )
        {
            LOG_WARN_S << "Got a command which did not contain a position or effort value." << std::endl;
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

void Task::printErrorStatus(ErrorStatus status)
{
    LOG_ERROR("Has error: %i", (int)status.hasError());
    LOG_ERROR("angle_limit_error: %i", (int)status.angleLimitError);
    LOG_ERROR("checksumError: %i", (int)status.checksumError);
    LOG_ERROR("inputVoltageError: %i", (int)status.inputVoltageError);
    LOG_ERROR("instructionError: %i", (int)status.instructionError);
    LOG_ERROR("overheatingError: %i", (int)status.overheatingError);
    LOG_ERROR("overloadError: %i", (int)status.overloadError);
    LOG_ERROR("rangeError: %i", (int)status.rangeError);
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
        dynamixel_.setControlTableEntry("Torque Enable", _keep_torque_enabled.value());

        // set disabled flag
        sc.enabled = (bool)_keep_torque_enabled.value();
    }
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    // clear configuration structures
    status_map.clear();
    joint_status.clear();
}
