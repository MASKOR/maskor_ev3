#ifndef _COMMON_H_
#define _COMMON_H_

typedef std::string         device_type;
typedef std::string         mode_type;
typedef std::set<mode_type> mode_set;
typedef std::string         address_type;

//-----------------------------------------------------------------------------

const address_type INPUT_AUTO;  //!< Automatic input selection
const address_type OUTPUT_AUTO; //!< Automatic output selection

#ifdef EV3DEV_PLATFORM_BRICKPI
const address_type INPUT_1  { "ttyAMA0:in1" };  //!< Sensor port 1
const address_type INPUT_2  { "ttyAMA0:in2" };  //!< Sensor port 2
const address_type INPUT_3  { "ttyAMA0:in3" };  //!< Sensor port 3
const address_type INPUT_4  { "ttyAMA0:in4" };  //!< Sensor port 4

const address_type OUTPUT_A { "ttyAMA0:outA" }; //!< Motor port A
const address_type OUTPUT_B { "ttyAMA0:outB" }; //!< Motor port B
const address_type OUTPUT_C { "ttyAMA0:outC" }; //!< Motor port C
const address_type OUTPUT_D { "ttyAMA0:outD" }; //!< Motor port D
#else
const address_type INPUT_1  { "in1" };  //!< Sensor port 1
const address_type INPUT_2  { "in2" };  //!< Sensor port 2
const address_type INPUT_3  { "in3" };  //!< Sensor port 3
const address_type INPUT_4  { "in4" };  //!< Sensor port 4

const address_type OUTPUT_A { "outA" }; //!< Motor port A
const address_type OUTPUT_B { "outB" }; //!< Motor port B
const address_type OUTPUT_C { "outC" }; //!< Motor port C
const address_type OUTPUT_D { "outD" }; //!< Motor port D
#endif


#endif //COMMON_H
