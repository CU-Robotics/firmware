/**
 * @file params.hpp
 * @author Jackson Stepka (jast2434@colorado.edu) (@Pandabear1125)
 * @brief Defines a parameter struct to pass around module parameters
 * @date 2025-07
 *
 * Implementation based on VectorNav VN-100 IMU/AHRS Interface Control Document (Firmware v3.1.0.0)
 *
 * @license While the source code is provided and visible, it is not open source. All rights are reserved. 
 *          No one may copy, modify, or distribute this code without explicit permission from the author.
 *          For more information, please contact the author directly.
 */

#pragma once

// TODO: do we actually need this if all the params are only used in the primary module?

namespace vn
{

/**
 * @brief Struct to hold parameters for the VN-100 module.
 */
struct Params {
	int vn100_mode = 0;
	int vn100_port = 1;
	int vn100_msg1_rate = 2;
	int vn100_msg2_rate = 8;
};

}
