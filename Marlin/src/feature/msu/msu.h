#pragma once

#include "../../inc/MarlinConfig.h"

class MSUMP{
public:
    MSUMP();
    //add init later
    //add reset later
    //add loop for error manadgment later
    static void idler_servo_init();
    static void tool_change(uint8_t index);
    static void idler_home();
    static bool idler_is_homing();
    static void edit_MSU_BOWDEN_TUBE_SETUP_length(const float diff);
    static void move_extruder(float diff,const_feedRate_t speed, int extruder_nbr, bool sync = true);
    static void move_both_extruders(float diff,const_feedRate_t speed);
    static void idler_select_filament_nbr(int index);
    static void filament_runout();
    static void error_on_load();
    static void error_on_unload();

    static bool active_filament_change();
    static const float get_MSU_BOWDEN_TUBE_SETUP_length();

};
extern MSUMP msu;
