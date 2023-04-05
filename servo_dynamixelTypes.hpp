#ifndef servo_dynamixel_TYPES_HPP
#define servo_dynamixel_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <vector>

namespace servo_dynamixel{
typedef struct
{
    // ID of the motor
    uint8_t id;
    // Name of item.
    // See Dynamixel Wizard, the monitor example from dynamixel-workbench for valid names for your device
    std::string item_name;
    // Int representation of the value to set
    uint8_t value;
} ControlTableAssignment;

typedef std::vector<ControlTableAssignment> DynamixelConfig;

};

#endif

