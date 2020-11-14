/*
 * Simple Swarm Robot Controller
 * Maryam and Pi
 * 

 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/gps.h>

#define PRECISION .3

#define PI 3.141592
#define RADIUS 2

#define DELTA 5

#define MAX_SPEED 20

#define TIME_STEP 64

static int get_time_step()
{
  static int time_step = -1;
  if (time_step == -1)
    time_step = (int)wb_robot_get_basic_time_step();
  return time_step;
}

static void step()
{
  if (wb_robot_step(get_time_step()) == -1)
  {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

/* get x coordinate destination*/
double get_xDestination(int N, int n)
{
  double result;
  result = RADIUS * cos((2 * PI * n) / (N));
  return result;
}
/* get x coordinate destination*/
double get_yDestination(int N, int n)
{
  double result;
  result = RADIUS * sin((2 * PI * n) / (N));
  return result;
}

int main(int argc, char *argv[])
{
  /* define variables */

  WbDeviceTag distance_sensor[8];
  WbDeviceTag left_motor, right_motor;
  int i;
  double speed_left = 0;
  double speed_right = 0;
  double sensors_value[8];

  /*Variables for destination determination*/
  int N, n, delta;
  double xD, yD, y0, x0, x, y, initial_distance, final_distance;
  double direction_gradient = 0;
  double destination_gradient = 0;
  double distance_difference = 0;
  delta = 0;
  N = 0;
  n = 1;
  xD = 0;
  yD = 0;
  y0 = 0;
  x0 = 0;
  x = 0;
  y = 0;
  initial_distance = 100;
  final_distance = 100;

  bool simulation_started = false;
  bool right_direction = false;
  /* initialize Webots */
  wb_robot_init();

  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  for (i = 0; i < 8; i++)
  {
    char device_name[4];
    /* get distance sensors */
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i], TIME_STEP * 4);
  }

  // wait until first distance sensor values are available
  if (wb_robot_step(TIME_STEP * 4) == -1)
  {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }

  /* get and enable GPS device */
  WbDeviceTag gps = wb_robot_get_device("gps");
  wb_gps_enable(gps, TIME_STEP);

  /* get key presses from keyboard */
  wb_keyboard_enable(TIME_STEP);

  /* main loop */
  while (true)
  {

    if (delta > DELTA)
    {
      delta = 0;
    }

    /*Get input from user*/
    switch (wb_keyboard_get_key())
    {
    case '3':
    {
      N = 3;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }
    case '4':
    {
      N = 4;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }
    case '5':
    {
      N = 5;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }
    case '6':
    {
      N = 6;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }
    case '7':
    {
      N = 7;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }
    case '8':
    {
      N = 8;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }
    case '9':
    {
      N = 9;
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);
      simulation_started = true;
      break;
    }

    default:
      break;
    }

    if (!simulation_started && N != 0)
    {
      printf("Forming %d-sided polygon. \n", N);
    }

    if (simulation_started)
    {
    
      xD = get_xDestination(N, n);
      yD = get_yDestination(N, n);

      /* get sensors values */
      for (i = 0; i < 8; i++)
      {
        sensors_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
      }

      /*printf("prox values = %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f\n",
     sensors_value[0], sensors_value[1], sensors_value[2], sensors_value[3],
     sensors_value[4], sensors_value[5], sensors_value[6], sensors_value[7]);
    */

      const double *gps_values = wb_gps_get_values(gps);
      x = gps_values[0];
      y = gps_values[2];

      
      /*Movement Code Begins Here*/
      final_distance = sqrt(((y - yD) * (y - yD)) + ((x - xD) * (x - xD)));
      
      distance_difference = initial_distance-final_distance;
      
      direction_gradient = (y-y0)/(x-x0);
      destination_gradient = (y-yD)/(x-xD);
      
      
      if(fabs(direction_gradient-destination_gradient)<0.1){
        right_direction = true;
      }
      
      if(!right_direction){
        speed_left = MAX_SPEED;
        speed_right = 0.1*MAX_SPEED;
      }else{
      
        speed_left = MAX_SPEED;
        speed_right = MAX_SPEED;
        
      }
      
      
      
      if(final_distance < 0.07){
        speed_left = 0;
        speed_right = 0;
      }
      
      
      

      printf("n, (xD,yD)(x,y)(x0,y0),dirG, desG, dist: %d (%.3f , %.3f) (%.5f , %.5f) (%.5f , %.5f) %.5f : %.5f : %.5f\n",n, xD,yD,x, y,x0, y0,direction_gradient,destination_gradient,final_distance);


      

      /* set speed values */
      wb_motor_set_velocity(left_motor, speed_left);
      wb_motor_set_velocity(right_motor, speed_right);
    }

    step();

    if (delta == DELTA)
    {

      /*record the old coordinates to determine movements and directions*/
      x0 = x;
      y0 = y;

      initial_distance = sqrt(((y0 - yD) * (y0 - yD)) + ((x0 - xD) * (x0 - xD)));
    }

    delta++;
  }

  wb_robot_cleanup();

  return 0;
}
