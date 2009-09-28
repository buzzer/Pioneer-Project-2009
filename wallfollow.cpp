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
#define DEBUG_POSITION_NO

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
const double VEL       = 0.3; // normal_advance_speed in meters per sec
const double K_P       = 1000; // kp_wall_following SRO: TODO What's this?
const double TURN_RATE = 40; // maximal_wall_following_turnrate (deg per sec)
                             // low values: smoother trajectory but more
                             // restricted
const double STOP_ROT  = 30; // stop_rotation_speed
                             // low values increase manauverablility in narrow
                             // edges, high values let the robot sometimes be
                             // stuck
const double WALLFOLLOWDIST = 0.5; // preferred_wall_following_distance
const double STOP_WALLFOLLOWDIST = 0.2; // stop_distance
const double WALLLOSTDIST  = 1.5; // Wall attractor
const double SHAPE_DIST = 0.3; // Min Radius from sensor for robot shape
// Laserranger
const double DEGSTEP   = 0.3515625; // 360./1024. in degree per laserbeam
const int    LSRANGE   = 240; // Arc range of the Laser sensor in degrees
const double LPMAX     = 5.0;  // max laser range in meters
const double COS45     = 0.83867056795; // cos(33);
const double INV_COS45 = 1.19236329284; // 1/COS45
const double DIAGOFFSET  = 0.1;  // laser to sonar diagonal offset in meters
const double HORZOFFSET  = 0.15; // laser to sonar horizontal offset in meters
const double MOUNTOFFSET = 0.1;  // sonar vertical offset at back for laptop mount
const int LMIN  = 175; const int LMAX  = 240; // LEFT
const int LFMIN = 140; const int LFMAX = 175; // LEFTFRONT
const int FMIN  = 100; const int FMAX  = 140; // FRONT
const int RFMIN = 65;  const int RFMAX = 100; // RIGHTFRONT
const int RMIN  = 0;   const int RMAX  = 65;  // RIGHT

// Measure angle to left wall and give appropriate turnrate back
inline double smoothTurnrate (double DistLFov)
{
  double turnrate = 0;

  // Calculate turn angle
  turnrate = atan( ( sp[0] - sp[15] ) / 0.23 );
  // Keep wall following distance to speed up
  DistLFov<(INV_COS45*WALLFOLLOWDIST) ? turnrate-=0.1 : turnrate+=0.1;

#ifdef DEBUG_LSONAR
  std::cout << "sp[0],sp[15],turnrate: " << sp[0] << "\t" << sp[15] << "\t" << rtod(turnrate) << std::endl;
#endif

  return turnrate;
}

// Input: Range of angle (degrees)
// Output: Minimum distance in range
// Algorithm calculates the average of 2 or 3 beams
// to define a minimum value per degree
inline double getDistanceArc ( int minAngle, int maxAngle )
{
  double sumDist     = 0.;
  double averageDist = 0.;
  double minDist     = LPMAX;

  if ( !(minAngle<0 || maxAngle<0 || minAngle>=maxAngle || minAngle>=LSRANGE || maxAngle>LSRANGE) )
  {
    // Measure minimum distance of each degree
    for (int arc=minAngle, rIndex=0, rIndexOld=rIndex, beamCount=1;      // Init per degree values
        arc < maxAngle;
        arc++, sumDist=0.) // Reset per degree values
    {
      // Measure average distance of beams belonging to one degree
      for (rIndex=(int)((double)arc/DEGSTEP), rIndexOld=rIndex;
          rIndex<(int)((double)(arc+1)/DEGSTEP);
          rIndex++)
      {
        sumDist += lp[rIndex];
      }
      beamCount = rIndex-rIndexOld;
      // Calculate the mean distance per degree
      averageDist = (double)sumDist/beamCount;
      // Calculate the minimum distance for the arc
      averageDist<minDist ? minDist=averageDist : minDist;
#ifdef DEBUG_LASER
      std::cout << sumDist << "\t" << rIndex << "\t" << rIndexOld << "\t" << beamCount << "\t" << averageDist << std::endl;
#endif
    }
  }

  return minDist;
}

// Input: Robot view direction
// Output: Minimum distance of requested viewDirection
// Robot shape shall be considered here by weighted SHAPE_DIST.
// Derived arcs, sonars and weights from graphic "PioneerShape.fig".
// NOTE: ALL might be slow due recursion, use it only for debugging!
inline double getDistance( viewDirectType viewDirection )
{
  // Scan safety areas for walls
  switch (viewDirection) {
    case LEFT      : return min(getDistanceArc(LMIN,  LMAX) -HORZOFFSET-SHAPE_DIST, min(sp[0], sp[15])-SHAPE_DIST);
    case RIGHT     : return min(getDistanceArc(RMIN,  RMAX) -HORZOFFSET-SHAPE_DIST, min(sp[7], sp[8]) -SHAPE_DIST);
    case FRONT     : return min(getDistanceArc(FMIN,  FMAX)            -SHAPE_DIST, min(sp[3], sp[4]) -SHAPE_DIST);
    case RIGHTFRONT: return min(getDistanceArc(RFMIN, RFMAX)-DIAGOFFSET-SHAPE_DIST, min(sp[5], sp[6]) -SHAPE_DIST);
    case LEFTFRONT : return min(getDistanceArc(LFMIN, LFMAX)-DIAGOFFSET-SHAPE_DIST, min(sp[1], sp[2]) -SHAPE_DIST);
    case BACK      : return min(sp[11], sp[12])-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
    case LEFTREAR  : return min(sp[13], sp[14])-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
    case RIGHTREAR : return min(sp[9] , sp[10])-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
    case ALL       : return min(getDistance(LEFT),
                             min(getDistance(RIGHT),
                               min(getDistance(FRONT),
                                 min(getDistance(BACK),
                                   min(getDistance(RIGHTFRONT),
                                     min(getDistance(LEFTFRONT),
                                       min(getDistance(LEFTREAR), getDistance(RIGHTREAR) )))))));
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

  DistFront = getDistance(FRONT);
  DistLFov  = getDistance(LEFTFRONT);
  DistL     = getDistance(LEFT);
  DistLRear = getDistance(LEFTREAR);

  // do simple (left) wall following

  // Check conditions for smooth wall following
  // up to ~45 degrees to wall
  if ( ( std::abs(sp[15] - sp[0]) < 0.2 ) &&
       ( DistLFov < WALLFOLLOWDIST + 0.5) &&
       ( DistLFov > STOP_WALLFOLLOWDIST ) &&
       ( DistFront > 1.0 )                   )
  {
#ifdef DEBUG_STATE
    std::cout << "OPTIMIZED FOLLOW\t" << DistLFov << std::endl;
#endif
    turnrate = smoothTurnrate(DistLFov);
  } else {
    //do naiv calculus for turnrate; weight dist vector
    turnrate = dtor(K_P * (COS45*DistLFov - WALLFOLLOWDIST));
    //turnrate = M_PI/-2 + acos( (DistL - COS45*(DistLFov+0.15) /
        //sqrt(DistL*DistL+(DistLFov+0.15)*(DistLFov+0.15)-TWO_COS45*DistL*(DistLFov+0.15))));
  }
#ifdef DEBUG_STATE
  std::cout << "WALLFOLLOW" << std::endl;
#endif

  // Normalize turnrate
  turnrate = limit(turnrate, -dtor(TURN_RATE), dtor(TURN_RATE));

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
  double distLeftFront  = getDistance(LEFTFRONT);
  double distFront      = getDistance(FRONT);
  double distRightFront = getDistance(RIGHTFRONT);

  *left_min  = (distFront + distLeftFront)  / 2;
  *right_min = (distFront + distRightFront) / 2;
}

// Biased by left wall following
inline void collisionAvoid ( double * turnrate,
                             StateType * currentState)
{
  double left_min  = LPMAX;
  double right_min = LPMAX;

  // Scan FOV for Walls
  scanfov(&right_min, &left_min);

  if ((left_min  < STOP_WALLFOLLOWDIST) ||
      (right_min < STOP_WALLFOLLOWDIST)   )
  {
    *currentState = COLLISION_AVOIDANCE;
    // Turn right as long we want left wall following
    *turnrate = -dtor(STOP_ROT);
#ifdef DEBUG_STATE
    std::cout << "COLLISION_AVOIDANCE" << std::endl;
#endif
  }
}

//TODO Code review
//TODO Consider turnrate for calculation
inline double calcspeed ( void )
{
  double tmpMinDistFront = min(getDistance(LEFTFRONT), min(getDistance(FRONT), getDistance(RIGHTFRONT)));
  double tmpMinDistBack  = min(getDistance(LEFTREAR), min(getDistance(BACK), getDistance(RIGHTREAR)));
  double speed = VEL;

  if (tmpMinDistFront < WALLFOLLOWDIST) {
    speed = VEL * (tmpMinDistFront/WALLFOLLOWDIST);

    // Do not turn back if there is a wall!
    if (tmpMinDistFront<0 && tmpMinDistBack<0)
      tmpMinDistBack<tmpMinDistFront ? speed=(VEL*tmpMinDistFront)/(tmpMinDistFront+tmpMinDistBack) : speed;
  }

  return speed;
}

//TODO Code review
// Implements more or less a rotation policy which decides depending on
// obstacles at the 4 robot edge surounding spots
// To not interfere to heavy to overall behaviour turnrate is only inverted (or
// set to zero)
inline void checkrotate (double * turnrate)
{
  // Check robot front
  if (getDistance(LEFTFRONT) < 0)
    if (getDistance(RIGHTFRONT) < 0)
      *turnrate = 0;
    else
      *turnrate>0 ? *turnrate*=(-1) : *turnrate; // Turn right if not already
  else if (getDistance(RIGHTFRONT) < 0)
    *turnrate<0 ? *turnrate*=(-1) : *turnrate; // Turn left; Sandwich case already considered above

  // Check robot back
  if (getDistance(LEFTREAR) < 0)
    if (getDistance(RIGHTREAR) < 0)
      *turnrate = 0;
    else
      *turnrate<0 ? *turnrate*=(-1) : *turnrate; // Turn left if not already
  else if (getDistance(RIGHTREAR) < 0)
    *turnrate>0 ? *turnrate*=(-1) : *turnrate; // Turn left; Sandwich case already considered above
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
    std::cout << "Laser (l/lf/f/rf/r/rb/b/lb):\t" << getDistanceArc(LMIN, LMAX)-HORZOFFSET << "\t"
      << getDistanceArc(LFMIN, LFMAX)-DIAGOFFSET  << "\t"
      << getDistanceArc(FMIN, FMAX)               << "\t"
      << getDistanceArc(RFMIN, RFMAX)-DIAGOFFSET  << "\t"
      << getDistanceArc(RMIN, RMAX)  -HORZOFFSET  << "\t"
      << "XXX"                                    << "\t"
      << "XXX"                                    << "\t"
      << "XXX"                                    << std::endl;
    std::cout << "Sonar (l/lf/f/rf/r/rb/b/lb):\t" << min(sp[15],sp[0]) << "\t"
      << min(sp[1],sp[2])               << "\t"
      << min(sp[3],sp[4])               << "\t"
      << min(sp[5],sp[6])               << "\t"
      << min(sp[7],sp[8])               << "\t"
      << min(sp[9],sp[10]) -MOUNTOFFSET << "\t"
      << min(sp[11],sp[12])-MOUNTOFFSET << "\t"
      << min(sp[13],sp[14])-MOUNTOFFSET << std::endl;
    std::cout << "Shape (l/lf/f/rf/r/rb/b/lb):\t" << getDistance(LEFT) << "\t"
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

#ifdef DEBUG_POSITION
    std::cout << pp.GetXPos() << "\t" << pp.GetYPos() << "\t" << rtod(pp.GetYaw()) << std::endl;
#endif
  }

} catch (PlayerCc::PlayerError e) {
  std::cerr << e << std::endl; // let's output the error
  return -1;
}
return 1;
}
