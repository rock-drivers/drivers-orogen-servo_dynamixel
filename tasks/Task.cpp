/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"
#include <boost/foreach.hpp>
#include <base/Logging.hpp>
#include <base/commands/Joints.hpp>

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

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

// Taken from p_Monitor.cpp of dynamixel workbench
bool print_control_table(DynamixelWorkbench& dxl_wb, uint8_t id){
    const ControlItem *control_item =  dxl_wb.getControlTable(id);
    uint8_t the_number_of_control_item = dxl_wb.getTheNumberOfControlItem(id);

    uint16_t last_register_addr = control_item[the_number_of_control_item-1].address;
    uint16_t last_register_addr_length = control_item[the_number_of_control_item-1].data_length;

    uint32_t getAllRegisteredData[last_register_addr+last_register_addr_length];
    const char *log;

    if (control_item != NULL)
    {
        bool wb_result = dxl_wb.readRegister(id, (uint16_t)0, last_register_addr+last_register_addr_length, getAllRegisteredData, &log);
        if (wb_result == false)
        {
            printf("%s\n", log);
            return false;
        }
        else
        {
            for (int index = 0; index < the_number_of_control_item; index++)
            {
                uint32_t data = 0;

                if (dxl_wb.getProtocolVersion() == 2.0f)
                {
                    data = getAllRegisteredData[control_item[index].address];
                    printf("\t%s : %d\n", control_item[index].item_name, data);
                }
                else if (dxl_wb.getProtocolVersion() == 1.0f)
                {
                    switch (control_item[index].data_length)
                    {
                    case BYTE:
                        data = getAllRegisteredData[control_item[index].address];
                        printf("\t%s : %d\n", control_item[index].item_name, data);
                        break;

                    case WORD:
                        data = DXL_MAKEWORD(getAllRegisteredData[control_item[index].address], getAllRegisteredData[control_item[index].address+1]);
                        printf("\t%s : %d\n", control_item[index].item_name, data);
                        break;

                    case DWORD:
                        data = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[control_item[index].address],   getAllRegisteredData[control_item[index].address+1]),
                                DXL_MAKEWORD(getAllRegisteredData[control_item[index].address+2], getAllRegisteredData[control_item[index].address+3]));
                        printf("\t%s : %d\n", control_item[index].item_name, data);
                        break;

                    default:
                        data = getAllRegisteredData[control_item[index].address];
                        break;
                    }
                }
            }
        }
    }
    return true;
}


/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    // try to open the device first
    //dynamixel_.setNumberRetries( _package_retry_count.value() );
    //dynamixel_.setTimeout(_timeout.value());
    std::cout << "Initializing " << _device.value() << " with baudrate " << _baudrate.value() << std::endl;
    if(!dynamixel_.init(_device.value().c_str(), _baudrate.value()))
    {
        LOG_ERROR( "Cannot open device '%s'.  %s", _device.value().c_str(), strerror(errno) );
        return false;
    }

    status_map.clear();
    
    // go through the configuration vector and set up the servos
    base::JointLimits limits = _joint_limits.get();
    uint limit_prop_count = 0;
    for( ServoConfiguration &sc: _servo_config.value() )
    {
        // get the id of the servo
        int id = sc.id;
        uint16_t model_number = 0;
        if(!dynamixel_.ping(id, &model_number)){
            LOG_ERROR_S << "Could not ping device with id " << id;
            return false;
        }else{
            std::cout << "Contacted device with id " << id << ". Model number: " << model_number << std::endl;
        }

        // copy relevant config information to the status
        ServoStatus status;
        status.id = id;
        status.enabled = (bool)_keep_torque_enabled.value();
        const ModelInfo* info = dynamixel_.getModelInfo(id);
        if(info ==nullptr){
            LOG_ERROR_S << "Could not get model info";
            return false;
        }
        status.model_info = ModelInfo(*info);

        // and write an info about the setup
        std::cout << "get info" <<std::endl;
        int32_t model, firmware, sid;
        dynamixel_.getModelName(id);
        float protocol_version = dynamixel_.getProtocolVersion();
        status.protocol_version = protocol_version;
        std::cout << "get info2" <<std::endl;
        dynamixel_.readRegister(id, "Model Number", &model);
        dynamixel_.readRegister(id, "Firmware Version", &firmware);
        dynamixel_.readRegister(id, "ID", &sid);
        std::cout
                << "Found servo model " << model
                << " firmware " << firmware
                << " at id " << sid
                << " speaking protocol " << protocol_version
                << std::endl;

        // print the control table
        std::cout << "BEGIN OF CONTROL TABLE" << std::endl;
        print_control_table(dynamixel_, id);
        std::cout << "END OF CONTROL TABLE" << std::endl;

        // might be that the servo is in an error state (e.g. overload)
        // we can release it by setting turning off the torque
        dynamixel_.torqueOff(id);

        // TODO: Not valid for all models. E.g. pro models (at lkeast H42-20-300-R does not have this)
        // set control value A,B,C,D,E (see RX-28 manual)
        /*if (!dynamixel_.setControlTableEntry("CW Compliance Slope", sc.cwComplianceSlope )) // P Gain Byte 1
            return false;
        if (!dynamixel_.setControlTableEntry("CW Compliance Margin", sc.cwComplianceMargin)) // D Gain
            return false;
        if (!dynamixel_.setControlTableEntry("CCW Compliance Margin", sc.ccwComplianceMargin)) // I Gain
            return false;
        if (!dynamixel_.setControlTableEntry("CCW Compliance Slope", sc.ccwComplianceSlope)) // P Gain Byte 2
            return false;
        if (!dynamixel_.setControlTableEntry("Punch", sc.punch))
            return false;*/



        // set joint limits
        base::JointLimitRange range;
        if(limits.hasNames()){
            try{
                range = limits.getElementByName(sc.name);
            }
            catch(std::out_of_range& ex){
                LOG_WARN_S << "No joint limit defined for " <<sc.name;
                // pass
            }
        }else{
            if(limits.size() == _servo_config.value().size()){
                range = limits[limit_prop_count++];
            }
        }
        status.limit = range;

        // reverse if wanted
        if (sc.reverse){
            dynamixel_.setReverseDirection(id);
        }

        // Configure Control Table entries according to limit specification
        bool st = true;
        if(protocol_version >= 2.0){
            if(range.max.hasPosition()){
                st &= dynamixel_.itemWrite(id, "Max Position Limit", dynamixel_.convertRadian2Value(id, range.max.position));
            }
            if(range.min.hasPosition()){
                st &= dynamixel_.itemWrite(id, "Min Position Limit", dynamixel_.convertRadian2Value(id, range.min.position));
            }
            if(range.max.hasSpeed()){
                st &= dynamixel_.itemWrite(id, "Velocity Limit", dynamixel_.convertVelocity2Value(id, range.max.speed));
            }
            if(range.max.hasEffort()){
                st &= dynamixel_.itemWrite(id, "Torque Limit", dynamixel_.convertCurrent2Value(id, range.max.effort));
            }
        }
        else{
            if(range.max.hasPosition()){
                st &= dynamixel_.itemWrite(id, "CCW Angle Limit", dynamixel_.convertRadian2Value(id, range.max.position));
            }
            if(range.min.hasPosition()){
                st &= dynamixel_.itemWrite(id, "CW Angle Limit", dynamixel_.convertRadian2Value(id, range.min.position));
            }
            if(range.max.hasSpeed()){
                st &= dynamixel_.itemWrite(id, "Moving Speed", dynamixel_.convertVelocity2Value(id, range.max.speed));
            }
            if(range.max.hasEffort()){
                st &= dynamixel_.itemWrite(id, "Torque Limit", dynamixel_.convertCurrent2Value(id, range.max.effort));
            }
        }
        if (!st){
            LOG_ERROR_S << "Error during setting motor limits";
            return false;
        }

        // Activate position control mode
        // TODO: Make control mode configurable
        if (!dynamixel_.setPositionControlMode(id)){
            LOG_ERROR_S << "Error activating position control mode";
            return false;
        }

        status_map[sc.name] = status;
    }

    return true;
}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    // Activate joint
    for( ServoConfiguration &sc: _servo_config.value() ){
        dynamixel_.torqueOn(sc.id);
    }

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

        // get target joint state
        base::JointState target( it->second.cmd );
        int32_t err;
        if( target.hasSpeed() )
        {
            if(_cap_at_limits.value() && status.limit.max.hasSpeed())
                target.speed = std::min( target.speed, status.limit.max.speed );

            if (!dynamixel_.goalVelocity(id, target.speed)){
                LOG_ERROR("Set speed %i for servo id %i failed. Max speed is: %f", target.speed, id, status.limit.max.speed);
                printErrorStatus(id);
                throw std::runtime_error("could not set speed value for servo");
            }
        }

        if( target.hasEffort())
        {
            if(_cap_at_limits.get() && status.limit.max.hasEffort())
                target.effort = std::min( target.effort, status.limit.max.effort );

            std::string entry_name = "Torque Limit";
            if(status.protocol_version < 2.0){
                entry_name = "Max Torque";
            }
            if (!dynamixel_.writeRegister(id, entry_name.c_str(), dynamixel_.convertCurrent2Value(target.effort))){
                LOG_ERROR("Set effort %i for servo id %i failed", target.effort, id);
                printErrorStatus(id);
                throw std::runtime_error("could not set target effort for servo");
            }
        }
        
        if( target.hasPosition() )
        {
            // Check joint limits
            if(_cap_at_limits.value())
                target.position = std::min( target.position, status.limit.max.position );
            
            // and write the updated goal position
            if(!dynamixel_.goalPosition(id, (float)target.position))
            {
                LOG_ERROR("Set position %f rad for servo id %i failed! Min Pos: %f rad, Max Pos: %f rad",
                          target.position, id, status.limit.min.position, status.limit.max.position);
                printErrorStatus(id);
                throw std::runtime_error("could not set target position for servo");
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
    const char *log;
    for( std::map<std::string, ServoStatus>::iterator mi =
         status_map.begin(); mi != status_map.end(); ++mi )
    {
        ServoStatus &sc( mi->second );

        // get the id of the servo
        int id = sc.id;

        // read the position values and write the scaled version to the joints status
        float rad;
        if( !dynamixel_.getRadian(id, &rad, &log) ){
            printErrorStatus(id);
            throw std::runtime_error(std::string("Could not read servo position value: ") + log);
        }
        joint_status[i].position = rad;

        // same for speed
        /*if( !dynamixel_.getVelocity(id, &joint_status[i].speed, &log) ){
            printErrorStatus(id);
            throw std::runtime_error(std::string("Could not read servo speed value: ") + log);
        }*/
        int32_t data;
        if(!dynamixel_.readRegister(id, "Present_Velocity", &data, &log)){
            printErrorStatus(id);
            throw std::runtime_error(std::string("Could not read servo speed value: ") + log);
        }
        joint_status[i].speed = dynamixel_.convertValue2Velocity(id, data);

        // and the load value
        int32_t effort;
        std::string el_name = "Present_Current";
        if (sc.protocol_version < 2.0 ){
            el_name = "Present_Load";
        }
        if( !dynamixel_.readRegister(id, el_name.c_str(), &effort, &log) ){
            printErrorStatus(id);
            throw std::runtime_error(std::string("Could not read servo load value: ") + log);
        }
        joint_status[i].effort = dynamixel_.convertValue2Load(effort);

        ++i;
    }
}


void Task::printErrorStatus(const int& id)
{
    int32_t err;
    dynamixel_.readRegister(id, "Hardware Error Status", &err);

    LOG_ERROR("Has error: %i", (int)CHECK_BIT(err, 7));
    LOG_ERROR("Unknown error: %i", (int)CHECK_BIT(err, 6));
    LOG_ERROR("Overload: %i", (int)CHECK_BIT(err, 5));
    LOG_ERROR("Electrical shock: %i", (int)CHECK_BIT(err, 4));
    LOG_ERROR("Motor encoder: %i", (int)CHECK_BIT(err, 3));
    LOG_ERROR("Overheating: %i", (int)CHECK_BIT(err, 2));
    LOG_ERROR("Motor hall: %i", (int)CHECK_BIT(err, 1));
    LOG_ERROR("Input voltage: %i", (int)CHECK_BIT(err, 0));
}

void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();

    // Activate joint
    for( ServoConfiguration &sc: _servo_config.value() ){
        if(!_keep_torque_enabled){
            dynamixel_.torqueOff(sc.id);
            status_map[sc.name].enabled = false;
        }
    }
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();

    // clear configuration structures
    status_map.clear();
    joint_status.clear();
}
