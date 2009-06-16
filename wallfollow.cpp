// URG Laser wall following example
// URG has 682 steps (240 degrees FOV)
// Index starts on the right sight of FOV
//
#include <iostream>
#include <libplayerc++/playerc++.h>
//#include <ctime>

using namespace PlayerCc;

// Global Robot proxy
PlayerClient    robot("localhost");
LaserProxy      lp(&robot,0);
//SonarProxy      sp(&robot,0);
Position2dProxy pp(&robot,0);

#define DEBUG

enum StateType
{
  WALL_FOLLOWING,
  COLLISION_AVOIDANCE,
  WALL_SEARCHING
};

// parameters
const double VEL       = 0.3; // normal_advance_speed
const double K_P       = 1000; // kp_wall_following SRO: TODO What's this?
const double TURN_RATE = 30; // maximal_wall_following_turnrate
const int    FOV       = 45; // collision_avoidance_fov
const int    LSRANGE   = 240; // Range of the Laser sensor
const int    LFOV      = LSRANGE/2 + FOV; // Left limit of the FOV
const int    RFOV      = LSRANGE/2 - FOV; // Right limit of the FOV
const int    MFOV      = LSRANGE/2; // Straight front FOV
const double MINWALLDIST = 0.7; // preferred_wall_following_distance
const double STOP_MINWALLDIST = 0.2; // stop_distance
const double SHAPE_DIST = 0.3; // Min Radius from sensor for robot shape
const double STOP_ROT  = 50; // stop_rotation_speed
const double DEGSTEP   = 360./1024.; // Degree per step
// Laserpointer values
const double LPMIN     = 0.02; // meters
const double LPMAX     = 5.0;  // meters
const time_t MAXCOUNT = 5;

// Calculates the turnrate from range measurement and minimum wall follow
// distance
// meas: range measurement in meters
// minwalldist: wall follow distance in meters
// turnrate: rotation rate
double wallfollow (double meas, double minwalldist)
{
    double turnrate=0;
    int theta = 0;

    // do simple (left) wall following
    turnrate = dtor(K_P * (meas - SHAPE_DIST - minwalldist));

    // Normalize rate
    if (turnrate > dtor(TURN_RATE))
    {
      turnrate = dtor(TURN_RATE);
    }
#ifdef DEBUG
    std::cout << "Distance Left: " << meas << std::endl;
#endif

    // Go straight if no wall is in distance (left and left front)
    theta = MFOV + 90;
    if (meas  >= 3.0                          &&
        lp[(uint32_t)(theta/DEGSTEP)] >= 5.0  &&
        lp[(uint32_t)(239/DEGSTEP)]   >= 3.0     )
    {
        turnrate = 0;
#ifdef DEBUG
        std::cout << "LOSTWALL" << std::endl;
#endif
    }

    return turnrate;
}

void scanfov (double * right_min, double * left_min)
{
    // Scan FOV for Walls
    for (int theta=RFOV; theta<LFOV; theta++)
    {
      // Right side
      if (theta < MFOV)
      {
        *right_min = min(*right_min, lp[(uint32_t)(theta/DEGSTEP)]);
      }
      // Left side
      else
      {
        *left_min  = min(*left_min, lp[(uint32_t)(theta/DEGSTEP)]);
      }
    }
    *left_min -= SHAPE_DIST;
    *right_min -= SHAPE_DIST;
}

void pathplan ( double * speed,
                double * turnrate,
                double * right_min,
                double * left_min,
                bool   * escape_direction,
                uint32_t * previous_mode)
{
    if ((*left_min < STOP_MINWALLDIST) ||
        (*right_min < STOP_MINWALLDIST)   )
    {
      *speed = 0;
      // selection of escape direction (done once for each object encounter)
      if (*previous_mode == WALL_FOLLOWING)
      {
        // go towards direction with most open space
        *escape_direction = left_min < right_min;
        // change this so that we know we have chosen the escape direction
        *previous_mode = COLLISION_AVOIDANCE;
      }

      if (*escape_direction) // right turn
        *turnrate = dtor(STOP_ROT);
      else // left turn
        *turnrate = dtor(-STOP_ROT);
    }
    else
    {
      *previous_mode = WALL_FOLLOWING;
    }
#ifdef DEBUG
    std::cout << "mode: " << *previous_mode << std::endl;
#endif
}

double calcspeed (double mindist)
{
    if (mindist < MINWALLDIST)
        return VEL * (mindist/MINWALLDIST);
    else
        return VEL;
}
double scanCriticalArea( void )
{
    double min_dist = 0;

    // Scan Safety Area for Walls
    // Right side
    for (int theta=0; theta<RFOV; theta++)
    {
        min_dist = min(min_dist, lp[(uint32_t)(theta/DEGSTEP)]);
    }
    // Left side
    for (int theta=LFOV; theta < LSRANGE; theta++)
    {
        min_dist = min(min_dist, lp[(uint32_t)(theta/DEGSTEP)]);
    }

    return min_dist;
}

void checkrotate (double * turnrate)
{
    double min_dist = 0;

    min_dist = scanCriticalArea();

    //if (lp[0]   < SHAPE_DIST ||
        //lp[240] < SHAPE_DIST   )
    //{
        //*turnrate = 0;
    //}
    if (min_dist < SHAPE_DIST)
        *turnrate = 0;
}

int
main(int argc, char *argv[])
{
  double speed = VEL;
  double tmp_speed = VEL;
  double turnrate = 0;
  double tmp_turnrate = 0;
  bool escape_direction = FALSE;
  bool goalachieved     = FALSE;
  double left_min = LPMAX;
  double right_min = LPMAX;
  uint32_t previous_mode = WALL_FOLLOWING;

  while (goalachieved == FALSE)
  {

    // Read from the proxies
    robot.Read();

    // (Left) Wall following
    turnrate = tmp_turnrate =
        wallfollow(lp[(uint32_t)(LFOV/DEGSTEP)], MINWALLDIST);

#ifdef DEBUG
    std::cout << "turnrate: " << turnrate << std::endl;
    std::cout << "speed: " << speed << std::endl;
#endif

    // Scan FOV for Walls
    scanfov(&right_min, &left_min);

    // Decide what to do
    pathplan(&tmp_speed,
            &tmp_turnrate,
            &right_min,
            &left_min,
            &escape_direction,
            &previous_mode);

    // Fusion of the vectors
    speed = (tmp_speed + speed) / 2;
    turnrate = (tmp_turnrate + turnrate) / 2;

    // Set speed dependend on the wall distance
    tmp_speed = calcspeed(min(left_min, right_min));

    // Fusion of the vectors
    speed = (tmp_speed + speed) / 2;

    // Check if rotating is safe
    checkrotate(&turnrate);

    // Fusion of the vectors
    turnrate = (tmp_turnrate + turnrate) / 2;

#ifdef DEBUG
    std::cout << "turnrate: " << turnrate << std::endl;
    std::cout << "speed: " << speed << std::endl;
#endif

    // Command the motors
    pp.SetSpeed(speed, turnrate);

    // Reset distance values
    left_min = LPMAX;
    right_min = LPMAX;
  }
}
