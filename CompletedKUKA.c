/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */

#include <webots/keyboard.h>
#include <webots/robot.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TIME_STEP 32

static void step() {
  if (wb_robot_step(TIME_STEP) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void passive_wait(double sec) {
  double start_time = wb_robot_get_time();
  do {
    step();
  } while (start_time + sec > wb_robot_get_time());
}

static void automatic_behavior() {
  //This code controls the KUKA Youbot to remove the obstacle
  base_forwards();
  passive_wait(1.0);
  gripper_release();
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
  passive_wait(4.0);
  gripper_grip();
  passive_wait(1.0);
  base_backwards();
  passive_wait(4.0);
  gripper_release();
  //Grip it again for better grip
  base_forwards();
  passive_wait(0.1);
  gripper_release();
  arm_set_height(ARM_FRONT_CARDBOARD_BOX);
  passive_wait(0.5);
  gripper_grip();
  passive_wait(1.0);
  base_backwards();
  passive_wait(4.0);
  
}



int main(int argc, char **argv) {
  wb_robot_init();

  base_init();
  arm_init();
  gripper_init();
  passive_wait(2.0);

  if (argc > 1 && strcmp(argv[1], "demo") == 0)
    automatic_behavior();
    
  wb_robot_cleanup();

  return 0;
}
