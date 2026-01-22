#pragma once

#include "../../inc/MarlinConfig.h"

class MSUMP{
public:
    MSUMP();
    static void idler_servo_init();
    static void tool_change(uint8_t index);
    static void idler_home();
    static bool idler_is_homing();
    static void edit_MSU_BOWDEN_TUBE_SETUP_length(const float diff);
    //static void move_extruder(float diff,const_feedRate_t speed, int extruder_nbr);
    //static void move_both_extruders(float diff,const_feedRate_t speed);
    static void move_extruder(float dist, const_feedRate_t speed, int extruder_nbr);
    static void move_extruder_sync(float dist, const_feedRate_t speed, int extruder_nbr);
    static void move_both_extruders(float dist, float step, const_feedRate_t speed);
    static void move_both_extruders_sync(float dist, float step, const_feedRate_t speed);
    static void idler_select_filament_nbr(int index);
    static void filament_runout();
    static void error_on_load();
    static void error_on_unload();

    static bool active_filament_change();
    static const float get_MSU_BOWDEN_TUBE_SETUP_length();

    static void runout_sensor_contol_set(bool set_state);
    static void cut_filament(int cut_try);
    static void park_extruder();
    static void park_extruder_for_wipe();
    static void nozzle_wipe();
    static char * text_selected_filament_nbr();
    static void pre_change_lcd_message(int a, int b);
    static void post_change_lcd_message(int a);

};
extern MSUMP msu;