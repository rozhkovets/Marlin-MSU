#include "../../inc/MarlinConfig.h"

#if ENABLED(MSU)
#include "msu.h"
#include "../../module/servo.h" 
#include "../../module/planner.h" 
#include "../../lcd/marlinui.h"
#include "../../feature/runout.h"
#include "../../lcd/lcdprint.h"
#include "../../gcode/gcode.h"

bool runout_state; //состояние датчика окончания филамента (контроль датчика включен или выключен)

int MSU_IDLER_POSITION[6] = MSU_BEARING_ANGLES; //позиции парковки и филаментов 1-5

int selected_filament_nbr = 0; // номер текущего выбранного филамента

char char_arr[3]; //для сообщения на экране о текущем инструменте 

xyze_pos_t extruder_origin_position;
xyze_pos_t extruder_park_position MSU_PARK_EXTRUDER_POS;
xyz_pos_t msu_park_point NOZZLE_PARK_POINT;			
constexpr feedRate_t park_fr_xy = MSU_PARK_EXTRUDER_FR;
xyze_pos_t extruder_park_wipe_position = MSU_PARK_EXTRUDER_WIPE_POS;

#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  float steps_per_mm_correction_factor = MSU_EXTRUDER_STEPS_PER_MM / static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
#else
  float steps_per_mm_correction_factor = 1;
#endif

//Вызов команды T							 
void MSUMP::tool_change(uint8_t index)
{
  //0. Проверка на уже выбранный
  if (selected_filament_nbr == index) {
      ui.refresh();             
      idle();                   
      return; // Nothing to do
  }

  //1. Вывод сообщения на экран перед сменой
  #if ENABLED(MSU_MESSAGES)
    pre_change_lcd_message(selected_filament_nbr, index);
  #endif

  //2. Выключить контроль датчика окончания филамента
  #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
    runout_sensor_contol_set(false);
  #endif

  //3. Парковка экструдера
  #if ENABLED(MSU_PARK_EXTRUDER_WHILE_MSU_TOOL_CHANGE)
    extruder_origin_position = current_position; //сохранить исходное положение экструдера
    #if ALL(MSU_PARK_RETRACT_BEFORE_PARK_MM, MSU_PARK_RETRACT_BEFORE_PARK_FR)
      //move_extruder(-MSU_PARK_RETRACT_BEFORE_PARK_MM, MSU_PARK_RETRACT_BEFORE_PARK_FR, MSU_ORIGINAL_EXTRUDER_NBR)
      move_extruder_sync(-MSU_PARK_RETRACT_BEFORE_PARK_MM, MSU_PARK_RETRACT_BEFORE_PARK_FR, MSU_ORIGINAL_EXTRUDER_NBR)
    #endif
    park_extruder();
  #endif

  //3.1 Парковка экструдера в положение для очистки
  #if ENABLED(MSU_NOZZLE_WIPE)
    park_extruder_for_wipe();
  #endif

  //4.1 MSU_BOWDEN_TUBE_SETUP
  #if ENABLED(MSU_BOWDEN_TUBE_SETUP)//не проверено
    #warning "WARNING! BOWDEN_TUBE_SETUP NOT TESTED"
    //4.1.1 Поворот idler 
    //с bowden не нужен? 
    idler_select_filament_nbr(selected_filament_nbr);
  
    //4.1.2 Резка филамента
    #if ENABLED(MSU_WITH_CUTTER)
      move_extruder_sync(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_SPEED, MSU_EXTRUDER_NBR);
      cut_filament(MSU_SERVO_CUTTER_TRY);
    #endif

    //4.1.3 Выгрузка
    move_extruder_sync(-0.1, MSU_SPEED, MSU_EXTRUDER_NBR); //костыль резкого первого движения  
    move_extruder_sync(-MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //4.1.4 Загрузка
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder_sync(MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //4.1.5 Включить контроль датчика окончания филамента
    //если загрузка неудачна, должно вызвать срабатывание датчика и M600 сразу после окончания смены филамента
    #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
      runout_sensor_contol_set(true);
    #endif

    //4.1.6 Продувка
    move_extruder_sync(MSU_PURGE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
  #endif

  //4.2 MSU_DIRECT_DRIVE_SETUP
  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    //4.2.1 Выгрузка филамента из экструдера
    move_extruder_sync(-MSU_GEAR_LENGTH, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
  
    //4.2.2 Резка филамента
    #if ENABLED(MSU_WITH_CUTTER)
      move_extruder_sync(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
      cut_filament(MSU_SERVO_CUTTER_TRY);
    #endif

    //4.2.3 Выгрузка MSU
    idler_select_filament_nbr(selected_filament_nbr);
    move_extruder_sync(-0.1, MSU_SPEED, MSU_EXTRUDER_NBR); //костыль резкого первого движения
    move_extruder_sync(-MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //4.2.4 Загрузка MSU
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder_sync(1 + MSU_BOWDEN_TUBE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
    move_both_extruders_sync(MSU_DIRECT_DRIVE_BOTH_LOAD_MM, MSU_DIRECT_DRIVE_BOTH_LOAD_STEP, MSU_DIRECT_DRIVE_BOTH_LOAD_SPEED);
    idler_select_filament_nbr(-1);
    
    //4.2.5 Включить контроль датчика окончания филамента
    //если загрузка неудачна, должно вызвать срабатывание датчика и M600 сразу после окончания смены филамента
    #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
      runout_sensor_contol_set(true);
    #endif

    //4.2.6 Продувка
    move_extruder_sync(MSU_PURGE_LENGTH, MSU_ORIGINAL_EXTRUDER_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
  #endif
  
  //4.3 MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP
  #if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)//не проверено
    #warning "WARNING! DIRECT_DRIVE_LINKED_EXTRUDER_SETUP NOT TESTED"
    
    //4.3.1 Предварительная выгрузка
    move_extruder_sync(-MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //4.3.2 Резка филамента
    #if ENABLED(MSU_WITH_CUTTER)
      move_extruder_sync(-MSU_SERVO_CUTTER_RETRACT_LENGHT, MSU_SPEED, MSU_EXTRUDER_NBR);
      cut_filament(MSU_SERVO_CUTTER_TRY);
    #endif

    //4.3.3 Выгрузка MSU
    idler_select_filament_nbr(selected_filament_nbr);
    move_extruder_sync(-MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);
    
    //4.3.4 Загрузка MSU
    idler_select_filament_nbr(index);
    selected_filament_nbr = index;
    move_extruder_sync(1 + MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);
    idler_select_filament_nbr(-1);
    move_extruder_sync(MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

    //4.3.5 Включить контроль датчика окончания филамента
    //если загрузка неудачна, должно вызвать срабатывание датчика и M600 сразу после окончания смены филамента
    #if ENABLED(MSU_ON_OFF_RUNOUT_SENSOR)
      runout_sensor_contol_set(true);
    #endif

    //4.1.6 Продувка
    move_extruder_sync(MSU_PURGE_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);

  #endif

  //5. Очистка сопла
  #if ENABLED(MSU_NOZZLE_WIPE)
    nozzle_wipe();
  #endif

  //6. Вывод сообщения на экран после смены
  #if ENABLED(MSU_MESSAGES) 
    post_change_lcd_message(selected_filament_nbr);
  #endif

  //7. Возврат после смены филамента
  #if ENABLED(MSU_PARK_EXTRUDER_WHILE_MSU_TOOL_CHANGE)
    do_blocking_move_to_xy(extruder_origin_position, park_fr_xy); //вернуть экструдер на исходную позицию
    idle();                 
  #endif
  
  ui.refresh();
}

void MSUMP::move_extruder(float dist, const_feedRate_t speed, int extruder_nbr)
{
  //SERIAL_ECHO_MSG("MSU: move_extruder: ", dist, " ", speed, " extruder_nbr: ", extruder_nbr);
  const float old = current_position.e;
  current_position.e += dist;
  planner.buffer_line(current_position, speed, extruder_nbr);
  current_position.e = old;
  planner.set_e_position_mm(old);
  //planner.synchronize();    
}

void MSUMP::move_extruder_sync(float dist, const_feedRate_t speed, int extruder_nbr){
  move_extruder(dist, speed, extruder_nbr);
  planner.synchronize();
  idle();
}

void MSUMP::move_both_extruders(float dist, float step, const_feedRate_t speed)
{
  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
  if (dist == 0) return;
  float remaining = dist;
  while (remaining > 0) {
    const float e_step = min(remaining, min(step, 1.0f));
    move_extruder(e_step, speed, MSU_EXTRUDER_NBR);
    move_extruder(e_step, speed, MSU_ORIGINAL_EXTRUDER_NBR);
    remaining = remaining - e_step;
  }
  idle();
  #endif
  /*// split the dist in 1mm chunks and move one extruder at a time
  //need bug fix
  //error in linked mode
  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
  for (int i = 0; i < dist; i++)
  {
    move_extruder(1, speed, MSU_EXTRUDER_NBR);
    move_extruder(1, speed, MSU_ORIGINAL_EXTRUDER_NBR);
  }
  #endif*/
}
void MSUMP::move_both_extruders_sync(float dist, float step, const_feedRate_t speed){
  move_both_extruders(dist, step, speed);
  planner.synchronize();
  idle();
}

// повернуть idler в положение для указанного филамента, -1 для парковки idler
void MSUMP::idler_select_filament_nbr(int index)
{
  if (index == -1)
    servo[MSU_SERVO_IDLER_NBR].move(MSU_IDLER_POSITION[0]);
  else
    servo[MSU_SERVO_IDLER_NBR].move(MSU_IDLER_POSITION[index + 1]);
  idle();                   
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
    idle();                
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
  idle();          
}

//парковка экструдера в положение для очистки
void MSUMP::park_extruder_for_wipe() {
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
  idle();           
}

//очистка сопла
void MSUMP::nozzle_wipe()    
{
  gcode.process_subcommands_now(F(MSU_NOZZLE_WIPE_CGODE));
}

//сообщение перед сменой инструмента
void MSUMP::pre_change_lcd_message(int a, int b)
{
  char msg[32]; // 32 символа — стандарт для LCD

  snprintf(msg, sizeof(msg),
           "Change T%d(F%d)->T%d(F%d)",
            a % 10, (a+1) % 10,
            b % 10, (b+1) % 10 );
  ui.set_status(msg);
  ui.refresh();             
  idle();                   
}

//сообщение после смены инструмента
void MSUMP::post_change_lcd_message(int a)
{
  char msg[32]; // 32 символа — стандарт для LCD

  snprintf(msg, sizeof(msg),
           "Selected T%d(F%d)",
            a % 10, (a+1)% 10 );
  ui.set_status(msg); 
  ui.refresh();             
  idle();                   
}

//установить контроль датчика окончания филамента
void MSUMP::runout_sensor_contol_set(bool set_state){
  runout_state = runout.enabled;
  if (runout_state) { 
    if (set_state){ //включить
      queue.inject(F(MSU_RUNOUT_SENSOR_ON_GCODE));  //FIX 2
      planner.synchronize();
        //gcode.process_subcommands_now(F(MSU_RUNOUT_SENSOR_ON_GCODE));
        //idle();                 // FIX
    } else { //выключить
      queue.inject(F(MSU_RUNOUT_SENSOR_OFF_GCODE));  //FIX 2
      planner.synchronize();
        //gcode.process_subcommands_now(F(MSU_RUNOUT_SENSOR_OFF_GCODE));
        //idle();                 // FIX
    }
  }
  idle();
}

//для сообщения на экране о текущем инструменте 
char * MSUMP::text_selected_filament_nbr()
{
  sprintf(char_arr, "%c", 'T');
  sprintf(char_arr+strlen(char_arr), "%d", selected_filament_nbr);
  return char_arr;
}
#endif