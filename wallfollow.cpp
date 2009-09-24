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
  LEFT,
  RIGHT,
  FRONT,
  BACK,
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
const double MINWALLDIST = 0.6; // preferred_wall_following_distance
const double STOP_MINWALLDIST = 0.2; // stop_distance
const double WALLLOSTDIST  = 1.5; // Wall attractor
const double SHAPE_DIST = 0.25; // Min Radius from sensor for robot shape
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
  (DistLFov)<MINWALLDIST ? turnrate-=0.2 : turnrate;

#ifdef DEBUG_LSONAR
  std::cout << "sp[0],sp[15],turnrate: " << sp[0] << "\t" << sp[15] << "\t" << rtod(turnrate) << std::endl;
#endif

  return turnrate;
}

// TODO consider median or mean for more robust laser range
// Returns minimum distance of requested viewDirection
// Robot shape shall be considered here !
// NOTE: ALL might be slow due recursion, use it only for debugging!
inline double getDistance( viewDirectType viewDirection )
{
  double LfMinDist = LPMAX;
  double RfMinDist = LPMAX;

  // Scan safety areas for walls
  switch (viewDirection) {
    case LEFT      : {
                       for (int theta=205; theta<240; theta++) // Check small laser range
                         LfMinDist = std::min(LfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       return std::min(LfMinDist-SHAPE_DIST*2, std::min(sp[0], sp[15])); // Check also sonar
                     }
    case RIGHT     : {
                       for (int theta=0; theta<35; theta++) // Check small laser range
                         LfMinDist = std::min(LfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       return std::min(LfMinDist-SHAPE_DIST*2, std::min(sp[7], sp[8])); // Check also sonar
                     }
    case FRONT     : {
                       for (int theta=115; theta<125; theta++) // Check small laser range
                         LfMinDist = std::min(LfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       return std::min(LfMinDist-SHAPE_DIST, std::min(sp[3], sp[4])-SHAPE_DIST); // Check also sonar
                     }
    case RIGHTFRONT: {
                       for (int theta=70; theta<80; theta++) // Check small laser range
                         RfMinDist = std::min(RfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       return std::min(RfMinDist-SHAPE_DIST*1.3, std::min(sp[5], sp[6])-SHAPE_DIST); // Check also sonar
                     }
    case LEFTFRONT : {
                       for (int theta=160; theta<170; theta++) // Check small laser range
                         LfMinDist = std::min(LfMinDist, lp[(uint32_t)(theta/DEGSTEP)]);
                       return std::min(LfMinDist-SHAPE_DIST*1.3, std::min(sp[1], sp[2])-SHAPE_DIST); // Check also sonar
                     }
    case BACK      : return std::min(sp[11], sp[12])-SHAPE_DIST; // Sorry, only sonar at rear
    case LEFTREAR  : return std::min(sp[13], sp[14])-SHAPE_DIST; // Sorry, only sonar at rear
    case RIGHTREAR : return std::min(sp[9] , sp[10])-SHAPE_DIST; // Sorry, only sonar at rear
    case ALL       : return std::min(getDistance(LEFT),
                             std::min(getDistance(RIGHT),
                               std::min(getDistance(FRONT),
                                 std::min(getDistance(BACK),
                                   std::min(getDistance(RIGHTFRONT),
                                     std::min(getDistance(LEFTFRONT),
                                       std::min(getDistance(LEFTREAR), getDistance(RIGHTREAR) )))))));
    default: return 0.; // Should be recognized if happens
  }
}

// Calculates the turnrate from range measurement and minimum wall follow
// distance
// meas: range measurement in meters
// minwalldist: wall follow distance in meters
// turnrate: rotation rate
inline double wallfollow( StateType * currentState )
{
  double turnrate  = 0;
  double DistLFov  = 0;
  double DistL     = 0;
  double DistLRear = 0;
  double DistFront = 0;

  // As long global goal is WF set it by default
  // Will potentially be overridden by higher prior behaviours
  *currentState = WALL_FOLLOWING;

  DistLFov  = getDistance(LEFTFRONT);
  DistL     = getDistance(LEFT);
  DistLRear = getDistance(LEFTREAR);
  DistFront = getDistance(FRONT);

  // do simple (left) wall following

  // Check conditions for smooth wall following
  // up to ~45 degrees to wall
  if ( ( std::abs(sp[15] - sp[0]) < 0.2 )          &&
      ( DistLFov < MINWALLDIST + 0.5) &&
      ( DistLFov > STOP_MINWALLDIST ) &&
      ( DistFront > 1.0 )                )
  {
#ifdef DEBUG_STATE
    std::cout << "OPTIMIZED FOLLOW\t" << DistLFov << std::endl;
#endif
    turnrate = smoothTurnrate(DistLFov);
  } else {
    // do naiv calculus for turnrate
    turnrate = dtor(K_P * (DistLFov*cos(dtor(45)) - MINWALLDIST));
  }
#ifdef DEBUG_STATE
  std::cout << "WALLFOLLOW" << std::endl;
#endif

  // Normalize turnrate
  if ( std::abs(turnrate) > dtor(TURN_RATE) )
    turnrate<0 ? turnrate=-dtor(TURN_RATE) : turnrate=dtor(TURN_RATE);

  // Go straight if no wall is in distance (front, left and left front)
  if (DistLFov  >= WALLLOSTDIST  &&
      DistL     >= WALLLOSTDIST  &&
      DistLRear >= WALLLOSTDIST     )
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
inline void collisionAvoid ( double * turnrate,
                             StateType * currentState)
{
  double left_min = LPMAX;
  double right_min = LPMAX;

  // Scan FOV for Walls
  scanfov(&right_min, &left_min);

  if ((left_min < STOP_MINWALLDIST) ||
      (right_min < STOP_MINWALLDIST)   )
  {
    *currentState = COLLISION_AVOIDANCE;
    // Turn right as long we want left wall following
    *turnrate = -dtor(STOP_ROT);
#ifdef DEBUG_STATE
    std::cout << "COLLISION_AVOIDANCE" << std::endl;
#endif
  }
}

//TODO Slow down if turning backwards into a wall
inline double calcspeed ( void )
{
  double tmpMinDistFront = std::min(getDistance(LEFTFRONT), std::min(getDistance(FRONT), getDistance(RIGHTFRONT)));
  //double tmpMinDistBack = std::min(getDistance(LEFTREAR), std::min(getDistance(BACK), getDistance(RIGHTREAR)));
  double speed = VEL;

  if (tmpMinDistFront < MINWALLDIST)
    speed = VEL * (tmpMinDistFront/MINWALLDIST);
  //if (tmpMinDistBack*2 < MINWALLDIST)
    //speed = -VEL * (tmpMinDistBack/MINWALLDIST);

  return speed;
}

// Implements more or less a rotation policy which decides depending on
// obstacles at the 4 robot edge surounding spots
// To not interfere to heavy to overall behaviour turnrate is only inverted (or
// set to zero)
inline void checkrotate (double * turnrate)
{
  // Check robot front
  if (getDistance(LEFTFRONT) < SHAPE_DIST)
    if (getDistance(RIGHTFRONT) < SHAPE_DIST)
      *turnrate = 0; else *turnrate>0 ? *turnrate=-*turnrate : *turnrate; // Turn right if not already
  else if (getDistance(RIGHTFRONT) < SHAPE_DIST)
    *turnrate<0 ? *turnrate=-*turnrate : *turnrate; // Turn left; Sandwich case already considered above

  // Check robot back
  if (getDistance(LEFTREAR) < SHAPE_DIST)
    if (getDistance(RIGHTREAR) < SHAPE_DIST)
      *turnrate = 0; else *turnrate<0 ? *turnrate=-*turnrate : *turnrate; // Turn left if not already
  else if (getDistance(RIGHTREAR) < SHAPE_DIST)
    *turnrate>0 ? *turnrate=-*turnrate : *turnrate; // Turn left; Sandwich case already considered above
}

int main( void )
{
try {

  double speed = VEL;
  double turnrate = 0;
  double tmp_turnrate = 0;
  StateType currentState = WALL_FOLLOWING;

  std::cout.precision(2);

  while (true)
  {
    // Read from the proxies
    robot.Read();

#ifdef DEBUG_SONAR
    std::cout << std::endl;
    for(int i=0; i< 16; i++)
      std::cout << "Sonar " << i << ": " << sp[i] << std::endl;
#endif

    // (Left) Wall following
    turnrate = wallfollow(&currentState);

    // Collision avoidance overrides wall follow turnrate if neccessary!
    collisionAvoid(&turnrate,
                   &currentState);

    // Set speed dependend on the wall distance
    speed = calcspeed();

    // Check if rotating is safe
    checkrotate(&tmp_turnrate);

    // Fusion of the vectors makes a smoother trajectory
    turnrate = (tmp_turnrate + turnrate) / 2;

#ifdef DEBUG_STATE
    std::cout << "turnrate/speed/state:\t" << turnrate << "\t" << speed << "\t"
      << currentState << std::endl;
#endif
#ifdef DEBUG_DIST
    std::cout << "Dist (l/lf/f/rf/r/rb/b/lb):\t" << getDistance(LEFT)
      << getDistance(LEFTFRONT)  << "\t"
      << getDistance(FRONT)      << "\t"
      << getDistance(RIGHTFRONT) << "\t"
      << getDistance(RIGHT)      << "\t"
      << getDistance(RIGHTREAR)  << "\t"
      << getDistance(BACK)       << "\t"
      << getDistance(LEFTREAR)   << std::endl;
#endif

    // Command the motors
    pp.SetSpeed(speed, turnrate);
  }

} catch (PlayerCc::PlayerError e) {
  std::cerr << e << std::endl; // let's output the error
  return -1;
}
return 1;
}
