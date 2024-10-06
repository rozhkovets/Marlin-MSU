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

#if  ALL(HAS_MARLINUI_MENU, MSU_MENU)

#include "menu_msu.h"
#include "../../feature/msu/msu.h"
#include "../../module/planner.h"
#include "menu_item.h"


void menu_msu_change_filament() {
  START_MENU();
  BACK_ITEM(MSG_MSU_MENU);
  //LOOP_L_N(i, 5) ACTION_ITEM_N(i, MSG_MSU_FILAMENT_N, []{ msu.tool_change(MenuItemBase::itemIndex); });
  for (int i = 0; i < 5; ++i){
    ACTION_ITEM_N(i, MSG_MSU_FILAMENT_N, []{ msu.tool_change(MenuItemBase::itemIndex); });
  }
  END_MENU();
}

void menu_msu_set_idler_position() {
  START_MENU();
  BACK_ITEM(MSG_MSU_MENU);
  ACTION_ITEM(MSG_MSU_IDLER_PARK_IDLER, []{ msu.idler_select_filament_nbr(-1); });
  //LOOP_L_N(i, 5) ACTION_ITEM_N(i, MSG_MSU_IDLER_POSITION_N, []{ msu.idler_select_filament_nbr(MenuItemBase::itemIndex); });
  for (int i = 0; i < 5; ++i){
    ACTION_ITEM_N(i, MSG_MSU_IDLER_POSITION_N, []{ msu.idler_select_filament_nbr(MenuItemBase::itemIndex); });
  }
  END_MENU();
}

void menu_msu() {
  START_MENU();
  BACK_ITEM(MSG_MAIN);
  ACTION_ITEM(MSG_MSU_IDLER_PARK_IDLER, []{ msu.idler_select_filament_nbr(-1); });
  SUBMENU(MSG_MSU_SELECT_FILAMENT, menu_msu_change_filament);
  SUBMENU(MSG_MSU_SET_IDLER_POSITION, menu_msu_set_idler_position);
  ACTION_ITEM(MSG_MSU_CUT_FILAMENT, []{ msu.cut_filament(1); });
  END_MENU();
}

#endif // MSU_MENU
