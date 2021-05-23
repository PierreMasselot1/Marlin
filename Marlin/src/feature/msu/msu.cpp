#include "../../inc/MarlinConfig.h"

#if ENABLED(MSU)

#include "msu.h"
#include "../../module/servo.h"
#include "../../module/planner.h"

float selected_filament_nbr = -1;
float idler_first_filament_pos = 30;
float idler_angle_between_bearing = 26;
float bowdenTubeLength = MSU_BOWDEN_TUBE_LENGTH;

bool idler_engaged = false;

xyze_pos_t position;

float steps_per_mm_correction_factor = 1;
#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
steps_per_mm_correction_factor = MSU_EXTRUDER_STEPS_PER_MM/static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
#endif
void MSUMP::tool_change(uint8_t index)
{
  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    move_extruder(-MSU_GEAR_LENGTH,MSU_SPEED,true);
  #endif

  #if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
    move_extruder(-MSU_GEAR_LENGTH,MSU_SPEED);
  #endif

  idler_select_filament_nbr(selected_filament_nbr);
  move_extruder(-MSU_BOWDEN_TUBE_LENGTH*steps_per_mm_correction_factor,MSU_SPEED);
  idler_select_filament_nbr(index);
  selected_filament_nbr=index;
  move_extruder(MSU_BOWDEN_TUBE_LENGTH*steps_per_mm_correction_factor,MSU_SPEED);

  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    move_extruder(4,MSU_SPEED,true);
    idler_select_filament_nbr(-1);
    move_extruder(MSU_GEAR_LENGTH,MSU_SPEED,true);
  #endif

  #if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
    move_extruder(4,MSU_SPEED);
    idler_select_filament_nbr(-1);
    move_extruder(MSU_GEAR_LENGTH,MSU_SPEED);
  #endif
}

void MSUMP::move_extruder(float dist, const_feedRate_t speed, bool moveBothExtruders)
{
  const float old = current_position.e;
  current_position.e += dist;
  planner.buffer_line(current_position, speed, MSU_EXTRUDER_NBR);
  current_position.e = old;
  planner.set_e_position_mm(old);

  #if ENABLED(MSU_DIRECT_DRIVE_SETUP)
    if (moveBothExtruders)
    {
      const float old = current_position.e;
      current_position.e += dist;
      planner.buffer_line(current_position, speed, MSU_ORIGINAL_EXTRUDER_NBR);
      current_position.e = old;
      planner.set_e_position_mm(old);
    }
  #endif
  
  planner.synchronize();
}

// move idler to specific filament selection, -1 to park the idler
void MSUMP::idler_select_filament_nbr(int index)
{
  if (index == -1)
    MOVE_SERVO(MSU_SERVO_IDLER_NBR, MSU_PARKING_POSITION);
  else
    MOVE_SERVO(MSU_SERVO_IDLER_NBR, MSU_SERVO_OFFSET + (index + 1) * MSU_BEARING_ANGLES);
}

#endif