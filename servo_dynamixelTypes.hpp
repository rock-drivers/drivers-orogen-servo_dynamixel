#ifndef servo_dynamixel_TYPES_HPP
#define servo_dynamixel_TYPES_HPP

#include <string>
#include <inttypes.h>
#include <stdexcept>

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

namespace servo_dynamixel {

struct ServoConfiguration
{
    /** name of the servo, used to identify with the actuators name */
    std::string name;

    /** servo id on the chain */
    int id;

    //-- The Dynamixel controls Compliance by setting the Margin and Slope.
    //   If used well Compliance will absorb the shocks. The following graph
    //   demonstrates the use of Compliance values (length of A,B,C & D)
    //   relative to Position Error and applied torque.
    //
    //   y-axis : output torque
    //   x-axis : position error
    //
    //     CW /|\                                                  |
    //         |  _____         goal position                      |
    //         |       \        |                                  |
    //         |        \       |                                  |
    //         |         \      |                           _ _    |
    //         |          |    \|/                           | E   |
    //  CCW /__|__________|__________________________\ CW   _|_    |
    //      \  |                      |              /       |     |
    //         |                      |                     _|_E   |
    //         |                       \                           |
    //         |                        \                          |
    //         |                         \_________                |
    //         |                                                   |
    //    CCW \|/     |---|-----|-----|---|
    //                  A    B     C    D
    //
    //   A = ccwComplianceSlope
    //   B = ccwComplianceMargin
    //   C = cwComplianceMargin
    //   D = cwComplianceSlope
    //   E = punch

    /** clockwise compliance margin */
    uint8_t cwComplianceMargin;    
    /** counterclockwise compliance margin */
    uint8_t ccwComplianceMargin;
    /** clockwise compliance slope */
    uint8_t cwComplianceSlope;  
    /** counterclockwise compliance slope */
    uint8_t ccwComplianceSlope; 

    /** minimum current being supplied to the motor during action */
    uint16_t punch;

    /** scale value to convert ticks to radians */
    float scale;
    /** zero offset value in radians */
    float zero;

    void checkValid() const
    {
#define in_range( field, min, max ) if( field < min || field > max ) \
	throw std::runtime_error( #field " range is " #min "-" #max )

	in_range( cwComplianceMargin, 0, 254 );
	in_range( ccwComplianceMargin, 0, 254 );
	in_range( cwComplianceSlope, 0, 254 );
	in_range( ccwComplianceSlope, 0, 254 );
	in_range( punch, 32, 1023 );

#undef in_range
    }
};

}

#endif

