// URG Laser wall following example
// URG has 682 steps (240 degrees FOV)
// Index starts on the right sight of FOV

#ifdef DEBUG
#include <iostream>
#endif

#include <cmath>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

// Global Robot proxy
PlayerClient    robot("localhost");
LaserProxy      lp(&robot,0);
SonarProxy      sp(&robot,0);
Position2dProxy pp(&robot,0);

#define DEBUG_NO// Has to be set if any debug output wanted !!!
#define DEBUG_STATE_NO
#define DEBUG_CRIT_NO
#define DEBUG_SONAR_NO
#define DEBUG_LSONAR_NO
#define DEBUG_LASER_NO
#define DEBUG_DIST_NO

enum StateType {      // Current behaviour of the robot
  WALL_FOLLOWING,
  COLLISION_AVOIDANCE,
  WALL_SEARCHING
};
enum viewDirectType {   // Used for simple range area distinction
  LEFTFRONT,
  RIGHTFRONT,
  LEFTREAR,
  RIGHTREAR,
  ALL
};

// Parameters
const double VEL       = 0.3; // normal_advance_speed
const double K_P       = 1000; // kp_wall_following SRO: TODO What's this?
const double TURN_RATE = 40; // maximal_wall_following_turnrate (deg per sec)
                             // low values: smoother trajectory but more
                             // restricted
const int    FOV       = 45; // collision_avoidance_fov
const int    LSRANGE   = 240; // Range of the Laser sensor
const int    LFOV      = LSRANGE/2 + FOV; // Left limit of the FOV
const int    RFOV      = LSRANGE/2 - FOV; // Right limit of the FOV
const int    MFOV      = LSRANGE/2; // Straight front FOV
const double MINWALLDIST = 0.7; // preferred_wall_following_distance
const double STOP_MINWALLDIST = 0.2; // stop_distance
const double SHAPE_DIST = 0.4; // Min Radius from sensor for robot shape
const double STOP_ROT  = 30; // stop_rotation_speed
                            // low values increase manauverablility in narrow
                            // edges, high values let the robot sometimes be
                            // stuck
const double DEGSTEP   = 360./1024.; // Degree per step
// Laserranger
const double LPMIN     = 0.02; // meters
const double LPMAX     = 5.0;  // meters

// Measure angle to left wall and give appropriate turnrate back
inline double smoothTurnrate (double DistLFov)
{
  double turnrate = 0;

  // Calculate turn angle
  turnrate = atan( ( sp[0] - sp[15] ) / 0.23 );
  // Keep wall following distance to speed up
  (DistLFov-SHAPE_DIST)<MINWALLDIST ? turnrate-=0.2 : turnrate;

#ifdef DEBUG_LSONAR
  std::cout << "sp[0],sp[15],turnrate: " << sp[0] << "\t" << sp[15] << "\t" << rtod(turnrate) << std::endl;
#endif

  return turnrate;
}

// Calculates the turnrate from range measurement and minimum wall follow
// distance
// meas: range measurement in meters
// minwalldist: wall follow distance in meters
// turnrate: rotation rate
inline double wallfollow (double minwalldist, StateType * currentState)
{
  double turnrate=0;
  const double WallLostDist = 1.5;
  double DistLFov  = 0;
  double DistL     = 0;
  double DistLRear = 0;
  double DistFront = 0;

  // As long global goal is WF set it by default
  // Will potentially be overridden by higher prior behaviours
  *currentState = WALL_FOLLOWING;

  DistLFov  = lp[(uint32_t)(LFOV/DEGSTEP)];
  DistL     = lp[(uint32_t)(210/DEGSTEP)];
  DistLRear = lp[(uint32_t)(239/DEGSTEP)];
  DistFront = lp[(uint32_t)(MFOV/DEGSTEP)];

  // do simple (left) wall following
  //  Take Sonars into account
  DistLFov>sp[2] ? DistLFov=sp[2] : DistLFov;

  // Check conditions for smooth wall following
  // up to ~45 degrees to wall
  if ( ( std::abs(sp[15] - sp[0]) < 0.2 )          &&
      ( DistLFov < MINWALLDIST + SHAPE_DIST + 0.2) &&
      ( DistLFov > STOP_MINWALLDIST + SHAPE_DIST ) &&
      ( DistFront > 1.0 + SHAPE_DIST)                )
  {
#ifdef DEBUG_STATE
    std::cout << "OPTIMIZED FOLLOW\t" << DistLFov << std::endl;
#endif
    turnrate = smoothTurnrate(DistLFov);
  } else {
    // do naiv calculus for turnrate
    turnrate = dtor(K_P * (DistLFov - SHAPE_DIST - minwalldist));
  }
#ifdef DEBUG_STATE
  std::cout << "WALLFOLLOW" << std::endl;
#endif

  // Normalize turnrate
  if ( std::abs(turnrate) > dtor(TURN_RATE) )
    turnrate<0 ? turnrate=-dtor(TURN_RATE) : turnrate=dtor(TURN_RATE);

  // Go straight if no wall is in distance (front, left and left front)
  if (DistLFov  >= WallLostDist  &&
      DistL     >= WallLostDist  &&
      DistLRear >= WallLostDist     )
  {
    turnrate = 0;
    *currentState = WALL_SEARCHING;
#ifdef DEBUG_STATE
    std::cout << "WALL_SEARCHING" << std::endl;
#endif
  }

  return turnrate;
}

inline void scanfov (double * right_min, double * left_min)
{
  // Scan FOV for Walls
  for (int theta=RFOV; theta<LFOV; theta++)
  {
    if (theta < MFOV) // Right side
      *right_min = min(*right_min, lp[(uint32_t)(theta/DEGSTEP)]);
    else              // Left side
      *left_min  = min(*left_min, lp[(uint32_t)(theta/DEGSTEP)]);
  }
  // Get 2 front sonars into account
  *left_min  > sp[3] ? *left_min=sp[3] : *left_min;
  *right_min > sp[4] ? *left_min=sp[4] : *right_min;

  // Correct obstacles distances by robot shape (circle)
  *left_min  -= SHAPE_DIST;
  *right_min -= SHAPE_DIST;
}

// Biased by left wall following
inline void collisionAvoid ( double * speed,
                             double * turnrate,
                             double * right_min,
                             double * left_min,
                             StateType * currentState)
{
  if ((*left_min < STOP_MINWALLDIST) ||
      (*right_min < STOP_MINWALLDIST)   )
  {
    *currentState = COLLISION_AVOIDANCE;
    *speed = 0;
    // Turn right as long we want left wall following
    *turnrate = -dtor(STOP_ROT);
#ifdef DEBUG_STATE
    std::cout << "COLLISION_AVOIDANCE" << std::endl;
#endif
  }
}

// TODO consider median or mean for more robust laser range
// Returns minimum distance of requested viewDirection
// Robot shape shall be considered here !
inline double criticalDist( viewDirectType viewDirection )
{
  double LfMinDist = LPMAX;
  double RfMinDist = LPMAX;

  // Scan safety areas for walls
  switch (viewDirection) {
    case RIGHTFRONT: {
                       for (int theta=60; theta<75; theta++) // Check small laser range
                         RfMinDist = std::min(RfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       RfMinDist -= SHAPE_DIST;
                       return std::min(RfMinDist, sp[6]); // Check also sonar
                     }
    case LEFTFRONT : {
                       for (int theta=165; theta<180; theta++) // Check small laser range
                         LfMinDist = std::min(LfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       LfMinDist -= SHAPE_DIST;
                       return std::min(LfMinDist, sp[1]); // Check also sonar
                     }
    case LEFTREAR  : return sp[14]; // Sorry, only sonar at rear
    case RIGHTREAR : return sp[9];  // Sorry, only sonar at rear
    case ALL       : return std::min(criticalDist(RIGHTFRONT),
                               std::min(criticalDist(LEFTFRONT),
                                 std::min(criticalDist(LEFTREAR), criticalDist(RIGHTREAR) )));
    default: return 0.; // Should be recognized if happens
  }
}

//TODO Slow down if turning backwards into a wall
inline double calcspeed ( double min_dist )
{
  if (min_dist < MINWALLDIST)
    return VEL * (min_dist/MINWALLDIST);
  else
    return VEL;
}

// Implements more or less a rotation policy which decides depending on
// obstacles at the 4 robot edge surounding spots
// To not interfere to heavy to overall behaviour turnrate is only inverted (or
// set to zero)
inline void checkrotate (double * turnrate)
{
  // Check robot front
  if (criticalDist(LEFTFRONT) < SHAPE_DIST)
    if (criticalDist(RIGHTFRONT) < SHAPE_DIST)
      *turnrate = 0; else *turnrate>0 ? *turnrate=-*turnrate : *turnrate; // Turn right if not already
  else if (criticalDist(RIGHTFRONT) < SHAPE_DIST)
    *turnrate<0 ? *turnrate=-*turnrate : *turnrate; // Turn left; Sandwich case already considered above

  // Check robot back
  if (criticalDist(LEFTREAR) < SHAPE_DIST)
    if (criticalDist(RIGHTREAR) < SHAPE_DIST)
      *turnrate = 0; else *turnrate<0 ? *turnrate=-*turnrate : *turnrate; // Turn left if not already
  else if (criticalDist(RIGHTREAR) < SHAPE_DIST)
    *turnrate>0 ? *turnrate=-*turnrate : *turnrate; // Turn left; Sandwich case already considered above
}

int main( void )
{
  try {

  double speed = VEL;
  double turnrate = 0;
  double tmp_turnrate = 0;
  bool goalachieved = FALSE;
  double left_min = LPMAX;
  double right_min = LPMAX;
  StateType currentState = WALL_FOLLOWING;

  std::cout.precision(2);

  while (goalachieved == FALSE)
  {
    // Read from the proxies
    robot.Read();

#ifdef DEBUG_SONAR
    std::cout << std::endl;
    for(int i=0; i< 16; i++)
      std::cout << "Sonar " << i << ": " << sp[i] << std::endl;
#endif

    // (Left) Wall following
    turnrate = wallfollow(MINWALLDIST, &currentState);

    // Scan FOV for Walls
    scanfov(&right_min, &left_min);

    // Collision avoidance overrides wall follow turnrate if neccessary!
    collisionAvoid(&speed,
                   &turnrate,
                   &right_min,
                   &left_min,
                   &currentState);

    // Set speed dependend on the wall distance
    // This overrides collision avoidances but!
    // Nevertheless this conservative approach should be sufficient
    speed = calcspeed(std::min(left_min, right_min));
    //speed = calcspeed();

    // Check if rotating is safe
    checkrotate(&tmp_turnrate);

    // Fusion of the vectors makes a smoother trajectory
    turnrate = (tmp_turnrate + turnrate) / 2;

#ifdef DEBUG_STATE
    std::cout << "turnrate: " << turnrate << std::endl;
    std::cout << "speed: " << speed << std::endl;
    std::cout << currentState << std::endl;
#endif
#ifdef DEBUG_DIST
    std::cout << "Dist (lf/rf/rr/lr):\t" << criticalDist(LEFTFRONT) << "\t" <<
                                          criticalDist(RIGHTFRONT) << "\t" <<
                                          criticalDist(RIGHTREAR) << "\t" <<
                                          criticalDist(LEFTREAR) << std::endl;
#endif

    // Command the motors
    pp.SetSpeed(speed, turnrate);

    // Reset distance values
    left_min  = LPMAX; right_min = LPMAX;
  }

  } catch (PlayerCc::PlayerError e) {
    std::cerr << e << std::endl; // let's output the error
    return -1;
  }
  return 1;
}
