// Wall following example for Pioneer 2DX robot.
// This is part of the robotics students project at Uni Hamburg in 2009.
// COPYRIGHT Sebastian Rockel 2009
//
#ifdef DEBUG
#include <iostream>
#endif
#include <cmath>
#include <libplayerc++/playerc++.h>

using namespace PlayerCc;

#define DEBUG_NO// Has to be set if any debug output wanted !!!
#define DEBUG_STATE_NO
#define DEBUG_CRIT_NO
#define DEBUG_SONAR_NO
#define DEBUG_LASER_NO
#define DEBUG_DIST_NO
#define DEBUG_POSITION_NO

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

class Robot
{
private:
  PlayerClient    *robot;
  LaserProxy      *lp;
  SonarProxy      *sp;
  Position2dProxy *pp;
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
  int       robotID;
  double    speed;
  double    turnrate, tmp_turnrate;
  StateType currentState;

  // Input: Range of angle (degrees)
  // Output: Minimum distance in range
  // Algorithm calculates the average of 2 or 3 beams
  // to define a minimum value per degree
  inline double getDistanceLas ( int minAngle, int maxAngle )
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
          sumDist += lp->GetRange(rIndex);
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
      case LEFT      : return min(getDistanceLas(LMIN,  LMAX) -HORZOFFSET-SHAPE_DIST, min(sp->GetScan(0), sp->GetScan(15))-SHAPE_DIST);
      case RIGHT     : return min(getDistanceLas(RMIN,  RMAX) -HORZOFFSET-SHAPE_DIST, min(sp->GetScan(7), sp->GetScan(8)) -SHAPE_DIST);
      case FRONT     : return min(getDistanceLas(FMIN,  FMAX)            -SHAPE_DIST, min(sp->GetScan(3), sp->GetScan(4)) -SHAPE_DIST);
      case RIGHTFRONT: return min(getDistanceLas(RFMIN, RFMAX)-DIAGOFFSET-SHAPE_DIST, min(sp->GetScan(5), sp->GetScan(6)) -SHAPE_DIST);
      case LEFTFRONT : return min(getDistanceLas(LFMIN, LFMAX)-DIAGOFFSET-SHAPE_DIST, min(sp->GetScan(1), sp->GetScan(2)) -SHAPE_DIST);
      case BACK      : return min(sp->GetScan(11), sp->GetScan(12))-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
      case LEFTREAR  : return min(sp->GetScan(13), sp->GetScan(14))-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
      case RIGHTREAR : return min(sp->GetScan(9) , sp->GetScan(10))-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
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
    //do naiv calculus for turnrate; weight dist vector
    turnrate = atan( (COS45*DistLFov - WALLFOLLOWDIST ) * 4 );
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

  inline void getDistanceFOV (double * right_min, double * left_min)
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
    getDistanceFOV(&right_min, &left_min);

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
      //speed=(VEL*(tmpMinDistBack-tmpMinDistFront))/SHAPE_DIST;
      //tmpMinDistBack<tmpMinDistFront ? speed=(VEL*(tmpMinDistFront-tmpMinDistBack))/WALLFOLLOWDIST : speed;
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
    if (*turnrate < 0) { // Right turn
      getDistance(LEFTREAR)<0 ? *turnrate=0 : *turnrate;
      getDistance(RIGHT)   <0 ? *turnrate=0 : *turnrate;
    } else { // Left turn
      getDistance(RIGHTREAR)<0 ? *turnrate=0 : *turnrate;
      getDistance(LEFT)     <0 ? *turnrate=0 : *turnrate;
    }
  }

public:
  Robot(std::string name, int address, int id) {
    robot = new PlayerClient(name, address);
    pp    = new Position2dProxy(robot, id);
    lp    = new LaserProxy(robot, id);
    sp    = new SonarProxy(robot, id);
    robotID      = id;
    currentState = WALL_FOLLOWING;
  }

  void update ( void ) { robot->Read(); }
  void plan ( void) {
#ifdef DEBUG_SONAR
    std::cout << std::endl;
    for(int i=0; i< 16; i++)
      std::cout << "Sonar " << i << ": " << sp->GetScan(i) << std::endl;
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
    std::cout << "Laser (l/lf/f/rf/r/rb/b/lb):\t" << getDistanceLas(LMIN, LMAX)-HORZOFFSET << "\t"
      << getDistanceLas(LFMIN, LFMAX)-DIAGOFFSET  << "\t"
      << getDistanceLas(FMIN,  FMAX)              << "\t"
      << getDistanceLas(RFMIN, RFMAX)-DIAGOFFSET  << "\t"
      << getDistanceLas(RMIN,  RMAX) -HORZOFFSET  << "\t"
      << "XXX"                                    << "\t"
      << "XXX"                                    << "\t"
      << "XXX"                                    << std::endl;
    std::cout << "Sonar (l/lf/f/rf/r/rb/b/lb):\t" << min(sp->GetScan(15),sp->GetScan(0)) << "\t"
      << min(sp->GetScan(1), sp->GetScan(2))              << "\t"
      << min(sp->GetScan(3), sp->GetScan(4))              << "\t"
      << min(sp->GetScan(5), sp->GetScan(6))              << "\t"
      << min(sp->GetScan(7), sp->GetScan(8))              << "\t"
      << min(sp->GetScan(9), sp->GetScan(10))-MOUNTOFFSET << "\t"
      << min(sp->GetScan(11),sp->GetScan(12))-MOUNTOFFSET << "\t"
      << min(sp->GetScan(13),sp->GetScan(14))-MOUNTOFFSET << std::endl;
    std::cout << "Shape (l/lf/f/rf/r/rb/b/lb):\t" << getDistance(LEFT) << "\t"
      << getDistance(LEFTFRONT)  << "\t"
      << getDistance(FRONT)      << "\t"
      << getDistance(RIGHTFRONT) << "\t"
      << getDistance(RIGHT)      << "\t"
      << getDistance(RIGHTREAR)  << "\t"
      << getDistance(BACK)       << "\t"
      << getDistance(LEFTREAR)   << std::endl;
#endif
#ifdef DEBUG_POSITION
    std::cout << pp->GetXPos() << "\t" << pp->GetYPos() << "\t" << rtod(pp->GetYaw()) << std::endl;
#endif
  }
  // Command the motors
  void execute() { pp->SetSpeed(speed, turnrate); }
};



int main ( void ) {
  try {
    Robot pioneer("localhost", 6665, 0);
    std::cout.precision(2);
    while (true) {
      pioneer.update();
      pioneer.plan();
      pioneer.execute();
    }
  } catch (PlayerCc::PlayerError e) {
    std::cerr << e << std::endl; // let's output the error
    return -1;
  }
  return 1;
}
