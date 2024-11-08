#include "../../inc/MarlinConfig.h"

#if ENABLED(MSU)

#include "msu.h"
#include "../../module/servo.h"
#include "../../module/planner.h"

#include "../../gcode/gcode.h"

#include "../../lcd/marlinui.h"

#include "../../feature/runout.h"

#define MSU_RUNOUT_SENSOR_ON_GCODE  "M412 S1"
#define MSU_RUNOUT_SENSOR_OFF_GCODE "M412 S0"
bool runout_state;

int MSU_IDLER_POSITION[6] = MSU_BEARING_ANGLES; //parking, position 1 - 5
int selected_filament_nbr = 0;

#if ENABLED(MSU_LCD_MESSAGES) 
  MString<30> my_message;
#endif

char char_arr [3];

xyze_pos_t extruder_origin_position;
xyze_pos_t extruder_park_position MSU_PARK_EXTRUDER_POS;
xyz_pos_t msu_park_point NOZZLE_PARK_POINT;			
constexpr feedRate_t park_fr_xy = MSU_PARK_EXTRUDER_FR;
xyze_pos_t extruder_park_wipe_position = MSU_PARK_EXTRUDER_WIPE_POS;

float steps_per_mm_correction_factor = 1;

#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  steps_per_mm_correction_factor = MSU_EXTRUDER_STEPS_PER_MM / static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
#endif
//вызов команды T							 
void MSUMP::tool_change(uint8_t index)
{
  //проверка на уже выбранный
  if (selected_filament_nbr == index) return; // Nothing to do
  
  //вывод сообщения на экран
  #if ENABLED(MSU_LCD_MESSAGES) 
      my_message.set(F("M117 Change to F"));
      my_message.append(selected_filament_nbr+1);
      my_message.append("- T");
      my_message.append(selected_filament_nbr);
      gcode.process_subcommands_now(my_message);
  #endif

  #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
    runout_state = runout.enabled;
    if (runout_state) gcode.process_subcommands_now(F(MSU_RUNOUT_SENSOR_OFF_GCODE));
  #endif

  //парковка перед сменой филамента
  #if ENABLED(MSU_PARK_EXTRUDER_WHILE_MSU_TOOL_CHANGE)
    extruder_origin_position = current_position; //сохранить исходное положение экструдера
    #if ALL(MSU_PARK_RETRACT_BEFORE_PARK_MM, MSU_PARK_RETRACT_BEFORE_PARK_FR)
      move_extruder(-MSU_PARK_RETRACT_BEFORE_PARK_MM, MSU_PARK_RETRACT_BEFORE_PARK_FR, MSU_ORIGINAL_EXTRUDER_NBR)
    #endif
    park_extruder();
  #endif

  #if ENABLED(MSU_BOWDEN_TUBE_SETUP)
    //Выгрузка MSU
    idler_select_filament_nbr(selected_filament_nbr);

    #if ENABLED(MSU_WITH_CUTTER)
        move_extruder(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_SPEED, MSU_EXTRUDER_NBR); //костыль резкого первого движения
        cut_filament(MSU_SERVO_CUTTER_TRY);
    #endif  

    move_extruder(-0.1, MSU_SPEED, MSU_EXTRUDER_NBR); //костыль резкого первого движения
    move_extruder(-MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //Загрузка MSU
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder(1 + MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //если загрузка неудачна, должно вызвать срабатывание датчика и M600 сразу после окончания смены филамента
    #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
      if (runout_state) gcode.process_subcommands_now(F(MSU_RUNOUT_SENSOR_ON_GCODE));
    #endif

    //purge
    move_extruder(MSU_PURGE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

  #endif

  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    //выгрузка филамента из экструдера
    move_extruder(-MSU_GEAR_LENGTH, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);

    //резка
    #if ENABLED(MSU_WITH_CUTTER)
      move_extruder(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
      cut_filament(MSU_SERVO_CUTTER_TRY);
    #endif

    //Выгрузка MSU
    idler_select_filament_nbr(selected_filament_nbr);
    move_extruder(-0.1, MSU_SPEED, MSU_EXTRUDER_NBR); //костыль резкого первого движения
    move_extruder(-MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //Загрузка MSU
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder(1 + MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);	
    move_both_extruders(MSU_DIRECT_DRIVE_BOTH_LOAD_MM, MSU_DIRECT_DRIVE_BOTH_LOAD_SPEED);
    idler_select_filament_nbr(-1);

    //если загрузка неудачна, должно вызвать срабатывание датчика и M600 сразу после окончания смены филамента
    #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
      if (runout_state) gcode.process_subcommands_now(F(MSU_RUNOUT_SENSOR_ON_GCODE));
    #endif
    
    //purge
    move_extruder(MSU_PURGE_LENGTH, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
  #endif

  #if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
    //выгрузка
    move_extruder(-MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);  
    
    //резка  
    #if ENABLED(MSU_WITH_CUTTER)
      move_extruder(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_SPEED, MSU_EXTRUDER_NBR);
      cut_filament(MSU_SERVO_CUTTER_TRY);
    #endif    
    
    //Выгрузка MSU
    idler_select_filament_nbr(selected_filament_nbr);
    move_extruder(-0.1 * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR); //костыль резкого первого движения
    move_extruder(-MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);

    //Загрузка MSU
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder(1 + MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);

    idler_select_filament_nbr(-1);
    move_extruder(MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
  
    //если загрузка неудачна, должно вызвать срабатывание датчика и M600 сразу после окончания смены филамента
    #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
      if (runout_state) gcode.process_subcommands_now(F(MSU_RUNOUT_SENSOR_ON_GCODE));
    #endif
    
    //purge
    move_extruder(MSU_PURGE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
  #endif

  //nozzle wipe
  #if ENABLED(MSU_NOZZLE_WIPE)
    nozzle_wipe();
  #endif

  #if ENABLED(MSU_LCD_MESSAGES) 
      my_message.set(F("M117 F"));
      my_message.append(selected_filament_nbr+1);
      my_message.append("- T");
      my_message.append(selected_filament_nbr);
      gcode.process_subcommands_now(my_message);
  #endif

  //возврат после смены филамента
  #if ENABLED(MSU_PARK_EXTRUDER_WHILE_MSU_TOOL_CHANGE)
    do_blocking_move_to_xy(extruder_origin_position, park_fr_xy); //вернуть экструдер на исходную позицию
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
  //SERIAL_ECHO_MSG("MSU: move_extruder: ", dist, " ", speed, " extruder_nbr: ", extruder_nbr);
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
    servo[MSU_SERVO_IDLER_NBR].move(MSU_IDLER_POSITION[0]);
  else
    servo[MSU_SERVO_IDLER_NBR].move(MSU_IDLER_POSITION[index + 1]);
}



//резка филамента
void MSUMP::cut_filament(int cut_try) 
{
  for (int i = 0; i < cut_try; i++)
  {
    servo[MSU_SERVO_CUTTER_NBR].move(MSU_SERVO_CUTTER_CUT_ANGL);
    safe_delay(100);
    servo[MSU_SERVO_CUTTER_NBR].move(MSU_SERVO_CUTTER_PARK_ANGL);
    safe_delay(100);
  }       
}

//парковка экструдера
void MSUMP::park_extruder()  
{
  #ifndef MSU_PARK_EXTRUDER_MOVE
    #define MSU_PARK_EXTRUDER_MOVE 0
  #endif
  switch (MSU_PARK_EXTRUDER_MOVE) {
    case 0: do_blocking_move_to_xy(extruder_park_position, park_fr_xy); break;
    case 1: do_blocking_move_to_x(extruder_park_position.x, park_fr_xy); break;
    case 2: do_blocking_move_to_y(extruder_park_position.y, park_fr_xy); break;
    case 3: do_blocking_move_to_x(extruder_park_position.x, park_fr_xy);
            do_blocking_move_to_y(extruder_park_position.y, park_fr_xy); break;
    case 4: do_blocking_move_to_y(extruder_park_position.y, park_fr_xy);
            do_blocking_move_to_x(extruder_park_position.x, park_fr_xy); break;
  }
  
  #if ENABLED(MSU_NOZZLE_WIPE)
    #ifndef MSU_PARK_EXTRUDER_FOR_WIPE_MOVE
      #define MSU_PARK_EXTRUDER_FOR_WIPE_MOVE 0
    #endif
    switch (MSU_PARK_EXTRUDER_FOR_WIPE_MOVE) {
      case 0: do_blocking_move_to_xy(extruder_park_wipe_position, park_fr_xy); break;
      case 1: do_blocking_move_to_x(extruder_park_wipe_position.x, park_fr_xy); break;
      case 2: do_blocking_move_to_y(extruder_park_wipe_position.y, park_fr_xy); break;
      case 3: do_blocking_move_to_x(extruder_park_wipe_position.x, park_fr_xy);
              do_blocking_move_to_y(extruder_park_wipe_position.y, park_fr_xy); break;
      case 4: do_blocking_move_to_y(extruder_park_wipe_position.y, park_fr_xy);
              do_blocking_move_to_x(extruder_park_wipe_position.x, park_fr_xy); break;
    }
  #endif
}

//очистка сопла
void MSUMP::nozzle_wipe()    
{
  gcode.process_subcommands_now(F(MSU_NOZZLE_WIPE_CGODE));
}

char * MSUMP::text_selected_filament_nbr()
{
   sprintf(char_arr, "%c", 'T');
   sprintf(char_arr+strlen(char_arr), "%d", selected_filament_nbr);
   return char_arr;
}

#endif											  