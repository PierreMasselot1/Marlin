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
steps_per_mm_correction_factor = MSU_EXTRUDER_STEPS_PER_MM / static_cast<float>(planner.settings.axis_steps_per_mm[E_AXIS]);
#endif
void MSUMP::tool_change(uint8_t index)
{

#if ENABLED(MSU_DIRECT_DRIVE_SETUP)
  move_extruder(-MSU_GEAR_LENGTH, MSU_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
#endif

#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  move_extruder(-MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
#endif

  idler_select_filament_nbr(selected_filament_nbr);
  move_extruder(-MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);
  idler_select_filament_nbr(index);
  selected_filament_nbr = index;
  move_extruder(MSU_BOWDEN_TUBE_LENGTH * steps_per_mm_correction_factor, MSU_SPEED, MSU_EXTRUDER_NBR);

#if ENABLED(MSU_DIRECT_DRIVE_SETUP)
  move_both_extruders(4, MSU_SPEED);
  idler_select_filament_nbr(-1);
  move_extruder(MSU_GEAR_LENGTH - 4, MSU_SPEED, MSU_ORIGINAL_EXTRUDER_NBR);
#endif

#if ENABLED(MSU_DIRECT_DRIVE_LINKED_EXTRUDER_SETUP)
  idler_select_filament_nbr(-1);
  move_extruder(MSU_GEAR_LENGTH, MSU_SPEED, MSU_EXTRUDER_NBR);
#endif
}

#if ENABLED(MSU_DIRECT_DRIVE_SETUP)
void MSUMP::move_both_extruders(float dist, const_feedRate_t speed)
{
  // split the dist in 1mm chunks and move one extruder at a time
  for (int i = 0; i < dist; i++)
  {
    move_extruder(1, speed, MSU_EXTRUDER_NBR);
    move_extruder(1, speed, MSU_ORIGINAL_EXTRUDER_NBR);
  }
}
#endif

void MSUMP::move_extruder(float dist, const_feedRate_t speed, int extruder_nbr)
{
  SERIAL_ECHO_MSG("MSU: move_extruder: ", dist, " ", speed, "extruder_nbr: ", extruder_nbr);
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
    servo[MSU_SERVO_IDLER_NBR].move(270);
  else
    servo[MSU_SERVO_IDLER_NBR].move(MSU_SERVO_OFFSET + (index + 1) * MSU_BEARING_ANGLES);
}

#endif