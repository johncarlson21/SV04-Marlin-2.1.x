/**
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

#include "../../inc/MarlinConfig.h"

#if HAS_DUPLICATION_MODE

//#define DEBUG_DXC_MODE

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/tool_change.h"
#include "../../module/planner.h"

#if ENABLED(RTS_AVAILABLE)
  #include "../../lcd/e3v2/creality/LCD_RTS.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_DXC_MODE)
#include "../../core/debug_out.h"

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * M605: Set dual x-carriage movement mode
   *
   *   M605 S0 E H : (FULL_CONTROL) The slicer has full control over both X-carriages and can achieve optimal travel
   *             results as long as it supports dual X-carriages.
   *             If E is specified, indicate that only that one
   *             extruder is intended to be used, e.g. for runout
   *             detection.
   *
   *   M605 S1 H : (AUTO_PARK) The firmware automatically parks and unparks the X-carriages on tool-change so that
   *             additional slicer support is not required.
   *
   *   M605 S2 X R H : (DUPLICATION) The firmware moves the second X-carriage and extruder in synchronization with
   *             the first X-carriage and extruder, to print 2 copies of the same object at the same time.
   *             Set the constant X-offset and temperature differential with M605 S2 X[offs] R[deg] and
   *             follow with "M605 S2" to initiate duplicated movement. For example, use "M605 S2 X100 R2" to
   *             make a copy 100mm to the right with E1 2° hotter than E0.
   *
   *   M605 S3 H : (MIRRORED) Formbot/Vivedino-inspired mirrored mode in which the second extruder duplicates
   *             the movement of the first except the second extruder is reversed in the X axis.
   *             The temperature differential and initial X offset must be set with "M605 S2 X[offs] R[deg]",
   *             then followed by "M605 S3" to initiate mirrored movement.
   *
   *   M605 S4 H : (FULL_CONTROL) (Single Mode 2 for Sovol SV04)
   *             Equivalent to M605 S0 E1
   *
   *    M605 W  : IDEX What? command.
   *
   *    Note: the X axis should be homed after changing Dual X-carriage mode.
   *
   * If the H flag is enabled, update the user-visible mode on the host.
   */
  void GcodeSuite::M605() {
    planner.synchronize();

    if (parser.seenval('S') || !parser.seenval('W')) {
      const DualXMode previous_mode = (DualXMode)dualXPrintingModeStatus;

      if (parser.seenval('S'))
        dual_x_carriage_mode = (DualXMode)parser.value_byte();
      else
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

      if (dual_x_carriage_mode > DXC_SINGLE_1) {
        SERIAL_ECHOLNPGM("M605 out of range S0-", DXC_MAX_MODE);
        dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
      }

      SERIAL_ECHOLNPGM("M605 current mode: ", previous_mode);
      SERIAL_ECHOLNPGM("M605 new mode: ", dual_x_carriage_mode);

      uint8_t extruder = 0;
      bool with_extruder = false;

      switch (dual_x_carriage_mode) {
        case DXC_AUTO_PARK_MODE:
          idex_set_mirrored_mode(false);
          break;

        case DXC_SINGLE_1:        // S4 -> S0 E1 (backwards compat)
          extruder = 1;
          with_extruder = true;
          // fall through

        case DXC_FULL_CONTROL_MODE:
          if (parser.seenval('E')) {
            extruder = parser.value_byte();
            with_extruder = true;
          }
          if (extruder >= EXTRUDERS) {
            SERIAL_ECHOLNPGM("M605: Extruder index out of range");
            extruder = 0;
          }
          dual_x_carriage_mode = dxc_single(extruder);
          idex_set_mirrored_mode(false);
          if (with_extruder && active_extruder != extruder)
            tool_change(extruder);
          break;

        case DXC_DUPLICATION_MODE:
          idex_set_mirrored_mode(false);
          // Set the X offset, but no less than the safety gap
          if (parser.seenval('X')) duplicate_extruder_x_offset = _MAX(parser.value_linear_units(), (82) - (X1_MIN_POS));
          if (parser.seenval('R')) duplicate_extruder_temp_offset = parser.value_celsius_diff();
          // Always switch back to tool 0
          if (active_extruder != 0) tool_change(0);
          break;

        case DXC_MIRRORED_MODE: {
          if (previous_mode != DXC_DUPLICATION_MODE) {
            SERIAL_ECHOLNPGM("Printer must be in DXC_DUPLICATION_MODE prior to ");
            SERIAL_ECHOLNPGM("specifying DXC_MIRRORED_MODE.");
            dual_x_carriage_mode = previous_mode;
            return;
          }
          idex_set_mirrored_mode(true);

          // Do a small 'jog' motion in the X axis
          xyze_pos_t dest = current_position; dest.x -= 0.1f;
          for (uint8_t i = 2; --i;) {
            planner.buffer_line(dest, feedrate_mm_s, 0);
            dest.x += 0.1f;
          }
        } break;

        default:
          break;                // Cannot happen
      }

      dualXPrintingModeStatus = save_dual_x_carriage_mode =
        dual_x_carriage_mode;
#if ENABLED(RTS_AVAILABLE)
      if (parser.boolval('H'))
	SetExtruderMode(dualXPrintingModeStatus, true);
#endif
      if (dual_x_carriage_mode == DXC_MIRRORED_MODE)
        return;

      idex_set_parked(false);
      set_duplication_enabled(false);

      #ifdef EVENT_GCODE_IDEX_AFTER_MODECHANGE
        process_subcommands_now(F(EVENT_GCODE_IDEX_AFTER_MODECHANGE));
      #endif
    }

    #ifdef DEBUG_DXC_MODE
      if (parser.seen('W')) {
        DEBUG_ECHO_START();
        DEBUG_ECHOPGM("Dual X Carriage Mode ");
        switch (dual_x_carriage_mode) {
          default: DEBUG_ECHOPGM("FULL_CONTROL E", dxc_extruder_index()); break;
          case DXC_AUTO_PARK_MODE:    DEBUG_ECHOPGM("AUTO_PARK");    break;
          case DXC_DUPLICATION_MODE:  DEBUG_ECHOPGM("DUPLICATION");  break;
          case DXC_MIRRORED_MODE:     DEBUG_ECHOPGM("MIRRORED");     break;
          case DXC_SINGLE_2:          DEBUG_ECHOPGM("SINGLE_2");     break;
          default:                    DEBUG_ECHOPGM("ERROR_INVALID_MODE"); break;
        }
        DEBUG_ECHOPGM("\nActive Ext: ", active_extruder);
        if (!active_extruder_parked) DEBUG_ECHOPGM(" NOT ");
        DEBUG_ECHOPGM(" parked.");
        DEBUG_ECHOPGM("\nactive_extruder_x_pos: ", current_position.x);
        DEBUG_ECHOPGM("\ninactive_extruder_x: ", inactive_extruder_x);
        DEBUG_ECHOPGM("\nextruder_duplication_enabled: ", extruder_duplication_enabled);
        DEBUG_ECHOPGM("\nduplicate_extruder_x_offset: ", duplicate_extruder_x_offset);
        DEBUG_ECHOPGM("\nduplicate_extruder_temp_offset: ", duplicate_extruder_temp_offset);
        DEBUG_ECHOPGM("\ndelayed_move_time: ", delayed_move_time);
        DEBUG_ECHOPGM("\nX1 Home X: ", x_home_pos(0), "\nX1_MIN_POS=", X1_MIN_POS, "\nX1_MAX_POS=", X1_MAX_POS);
        DEBUG_ECHOPGM("\nX2 Home X: ", x_home_pos(1), "\nX2_MIN_POS=", X2_MIN_POS, "\nX2_MAX_POS=", X2_MAX_POS);
        DEBUG_ECHOPGM("\nX2_HOME_DIR=", X2_HOME_DIR, "\nX2_HOME_POS=", X2_HOME_POS);
        DEBUG_ECHOPGM("\nDEFAULT_DUAL_X_CARRIAGE_MODE=", STRINGIFY(DEFAULT_DUAL_X_CARRIAGE_MODE));
        DEBUG_ECHOPGM("\toolchange_settings.z_raise=", toolchange_settings.z_raise);
        DEBUG_ECHOPGM("\nDEFAULT_DUPLICATION_X_OFFSET=", DEFAULT_DUPLICATION_X_OFFSET);
        DEBUG_EOL();

        HOTEND_LOOP() {
          DEBUG_ECHOPGM_P(SP_T_STR, e);
          LOOP_NUM_AXES(a) DEBUG_ECHOPGM("  hotend_offset[", e, "].", AS_CHAR(AXIS_CHAR(a) | 0x20), "=", hotend_offset[e][a]);
          DEBUG_EOL();
        }
        DEBUG_EOL();
      }
    #endif // DEBUG_DXC_MODE
  }

#elif ENABLED(MULTI_NOZZLE_DUPLICATION)

  /**
   * M605: Set multi-nozzle duplication mode
   *
   *  S2       - Enable duplication mode
   *  P[mask]  - Bit-mask of nozzles to include in the duplication set.
   *             A value of 0 disables duplication.
   *  E[index] - Last nozzle index to include in the duplication set.
   *             A value of 0 disables duplication.
   */
  void GcodeSuite::M605() {
    bool ena = false;
    if (parser.seen("EPS")) {
      planner.synchronize();
      if (parser.seenval('P')) duplication_e_mask = parser.value_int();   // Set the mask directly
      else if (parser.seenval('E')) duplication_e_mask = _BV(parser.value_int() + 1) - 1; // Set the mask by E index
      ena = (2 == parser.intval('S', extruder_duplication_enabled ? 2 : 0));
      set_duplication_enabled(ena && (duplication_e_mask >= 3));
    }
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(STR_DUPLICATION_MODE);
    serialprint_onoff(extruder_duplication_enabled);
    if (ena) {
      SERIAL_ECHOPGM(" ( ");
      HOTEND_LOOP() if (TEST(duplication_e_mask, e)) { SERIAL_ECHO(e); SERIAL_CHAR(' '); }
      SERIAL_CHAR(')');
    }
    SERIAL_EOL();
  }

#endif // MULTI_NOZZLE_DUPLICATION

#endif // HAS_DUPICATION_MODE
