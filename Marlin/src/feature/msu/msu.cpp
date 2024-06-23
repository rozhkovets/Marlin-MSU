#include "../../inc/MarlinConfig.h"

#if ENABLED(MSU)

#include "msu.h"
#include "../../module/servo.h"
#include "../../module/planner.h"

int MSU_POSITION[6] = MSU_BEARING_ANGLES; //parking, position 1 - 5

float selected_filament_nbr = -1;
float idler_first_filament_pos = 30;
float idler_angle_between_bearing = 26;
float bowdenTubeLength = MSU_BOWDEN_TUBE_LENGTH;

bool idler_engaged = false;

xyze_pos_t position;
xyze_pos_t park_position MSU_PARK_EXTRUDER_POS;
xyze_pos_t origin_position;
constexpr feedRate_t park_fr_xy = MSU_PARK_EXTRUDER_FR;


xyz_pos_t msu_park_point NOZZLE_PARK_POINT;

float steps_per_mm_correction_factor = 1;
#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
steps_per_mm_correction_factor = MSU_EXTRUDER_STEPS_PER_MM / static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
#endif

//вызов команды T
void MSUMP::tool_change(uint8_t index)
{
if (index != active_extruder) {
  //парковка перед сменой филамента
  #if ENABLED(MSU_PARK_EXTRUDER_WHILE_MSU_TOOL_CHANGE)
    origin_position = current_position;
    #ifndef MSU_PARK_EXTRUDER_MOVE
      #define MSU_PARK_EXTRUDER_MOVE 0
    #endif
    switch (MSU_PARK_EXTRUDER_MOVE) {
      case 0: do_blocking_move_to_xy(park_position, park_fr_xy); break;
      case 1: do_blocking_move_to_x(park_position.x, park_fr_xy); break;
      case 2: do_blocking_move_to_y(park_position.y, park_fr_xy); break;
      case 3: do_blocking_move_to_x(park_position.x, park_fr_xy);
              do_blocking_move_to_y(park_position.y, park_fr_xy); break;
      case 4: do_blocking_move_to_y(park_position.y, park_fr_xy);
              do_blocking_move_to_x(park_position.x, park_fr_xy); break;
    }
  #endif
  //выгрузка экструдера и (или) резка
  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    #if ENABLED(MSU_WITH_CUTTER)
      move_extruder(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
      for (int i = 0; i < MSU_SERVO_CUTTER_TRY; i++)
      {
        servo[MSU_SERVO_CUTTER_NBR].move(MSU_SERVO_CUTTER_CUT_ANGL);
        safe_delay(100);
        servo[MSU_SERVO_CUTTER_NBR].move(MSU_SERVO_CUTTER_PARK_ANGL);
        safe_delay(100);
      }
    #else  
      move_extruder(-MSU_GEAR_LENGTH, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
    #endif
  #endif

  #if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
    move_extruder(-MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
  #endif

  //Выгрузка MSU
    idler_select_filament_nbr(selected_filament_nbr);
    move_extruder(-MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);
  //Загрузка MSU
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder(MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);

  //Загрузка экструдера
  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    move_both_extruders(MSU_DIRECT_DRIVE_BOTH_LOAD_MM, MSU_DIRECT_DRIVE_BOTH_LOAD_SPEED);
    idler_select_filament_nbr(-1);
    move_extruder(MSU_GEAR_LENGTH + MSU_ORIGINAL_EXTRUDER_PURGE_LENGTH, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
  #endif

  #if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
    idler_select_filament_nbr(-1);
    move_extruder(MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
  #endif
  }
  //возврат после смены филамента
  #if ENABLED(MSU_PARK_EXTRUDER_WHILE_MSU_TOOL_CHANGE)
    do_blocking_move_to_xy(origin_position, park_fr_xy);
  #endif
}
void MSUMP::move_both_extruders(float dist, const_feedRate_t speed)
{
  // split the dist in 1mm chunks and move one extruder at a time
  for (int i = 0; i < dist; i++)
  {
    move_extruder(1, speed, MSU_EXTRUDER_NBR);
    move_extruder(1, speed, MSU_ORIGINAL_EXTRUDER_NBR);
  }
}

void MSUMP::move_extruder(float dist, const_feedRate_t speed, int extruder_nbr)
{
  SERIAL_ECHO_MSG("MSU: move_extruder: ", dist, " ", speed, ", extruder_nbr: ", extruder_nbr);
  const float old = current_position.e;
  current_position.e += dist;
  planner.buffer_line(current_position, speed, extruder_nbr);
  current_position.e = old;
  planner.set_e_position_mm(old);
  planner.synchronize();
}

// move idler to specific filament selection, -1 to park the idler
void MSUMP::idler_select_filament_nbr(int index)
{
    if (index == -1)
      servo[MSU_SERVO_IDLER_NBR].move(MSU_POSITION[0]);
    else
      servo[MSU_SERVO_IDLER_NBR].move(MSU_POSITION[index + 1]);
}
#endif



//образец для загрузки до срабатывания датчика
/*
    while (wait_for_user) {
      impatient_beep(max_beep_count);
      #if ALL(FILAMENT_CHANGE_RESUME_ON_INSERT, FILAMENT_RUNOUT_SENSOR)
        #if MULTI_FILAMENT_SENSOR
          #define _CASE_INSERTED(N) case N-1: if (READ(FIL_RUNOUT##N##_PIN) != FIL_RUNOUT##N##_STATE) wait_for_user = false; break;
          switch (active_extruder) {
            REPEAT_1(NUM_RUNOUT_SENSORS, _CASE_INSERTED)
          }
        #else
          if (READ(FIL_RUNOUT_PIN) != FIL_RUNOUT_STATE) wait_for_user = false;
        #endif
      #endif
      idle_no_sleep();
    }
*/