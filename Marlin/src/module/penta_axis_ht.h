/**
 * Marlin2ForPipetBot [https://github.com/DerAndere1/Marlin]
 * Copyright 2019 - 2023 DerAndere and other Marlin2ForPipetBot authors [https://github.com/DerAndere1/Marlin]
 *
 * Based on:
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once

/**
 * @file penta_axis_ht.h
 * @author DerAndere
 * @brief Kinematics for a 5 axis CNC machine in head-table configuration.
 * 
 * This machine has a swivel head and a horizontal rotary table.
 *
 * Copyright 2023 DerAndere
 *
 * Based on a relicensed verion of LinuxCNC file maxkins.c
 * Author: Chris Radek <chris@timeguy.com>
 * License: GPL Version 2 or GPL Version 3, at your option.
 *
 * Copyright (c) 2007, 2022 Chris Radek
 */


#include "../core/types.h"


// Machine rotary zero point Z offset is the distance from the center of rotation of the B axis to the gauge line.
extern float mrzp_offset_z;
extern bool tool_centerpoint_control;
extern float segments_per_second;

/**
 * penta axis head table inverse kinematics
 *
 * Calculate the joints positions for a given position, storing the result in the global delta[] array.
 * if joints control mode is active (default), input position is interpreted as joints position.
 * If rotational tool center point control mode is active, input position is interpreted as machine position using native_to_joint().
 */
void inverse_kinematics(const xyz_pos_t &raw);

/**
 * Calculate the joints positions for a given position.
 *
 * This is an expensive calculation.
 */
xyz_pos_t native_to_joint(const xyz_pos_t &native);
