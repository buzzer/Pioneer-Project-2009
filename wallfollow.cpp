/// @file wallfollowing.cpp
/// @author Sebastian Rockel
///
/// @mainpage Robotic Project 2009
///
/// @par Copyright
/// Copyright (C) 2009 Sebastian Rockel.
/// This program can be distributed and modified under the condition mentioning
/// the @ref author.
///
/// @par Description
/// Wall following example for Pioneer 2DX robot.
/// This is part of the robotics students project at Uni Hamburg in 2009.
///
/// @par Sensor fusion
/// The program depends on sensor fusion of 16 sonar rangers and a 240 degree
/// urg laser ranger.
/// @image html PioneerShape.png "Overview about how laser and sonar sensors are fused"
/// @image html AngleDefinition.png "Calculating turning angle via the atan function"
#ifdef DEBUG  // {{{
#include <iostream>
#endif  // }}}
#include <cmath>
#include <libplayerc++/playerc++.h>
#include "wallfollow.h"
// For timer services
#include <sys/time.h>

#ifdef OPENCV //{{{
# include "cc_camera1394.h"
# include "cc_ballfinder.h"
#endif //}}}

using namespace PlayerCc;

// {{{ DEBUG COMPILATION FLAGS
#define DEBUG///< Has to be set if any debug output wanted !!!
#define DEBUG_STATE///< Output of the robot's internal state
#define DEBUG_SONAR_NO///< Output of sonar readings
#define DEBUG_DIST_NO///< Output of all (fused) ranger readings
#define DEBUG_POSITION///< Output of the robot's odometry readings
#define DEBUG_LASER_NO///< Output of laser readings
#define DEBUG_CAM///< Output camera debug information
#define LASER///< Uses sonar + laser if defined
//#define OPENCV///< Uses omni vision camera via opencv library(Don't change-> makefile magic!)
// }}}

// Parameters {{{
const double VEL       = 0.2;///< Normal_advance_speed in meters per sec.
const double TURN_RATE = 30; ///< Max wall following turnrate in deg per sec.
                             /// Low values: Smoother trajectory but more
                             /// restricted
const double STOP_ROT  = 40; ///< Stop rotation speed.
                             /// Low values increase manauverablility in narrow
                             /// edges, high values let the robot sometimes be
                             /// stuck.
const double TRACK_ROT =  30; /// Goal tracking rotation speed in degrees per sec.
const double YAW_TOLERANCE = 20;///< Yaw tolerance for ball tracking in deg
const double DIST_TOLERANCE = 0.5;///< Distance tolerance before stopping in meters
const time_t BALLTIMEOUT = 10;/// Goal tracking time out in seconds.
const time_t BALLREQINT  = 1;/// Goal position request interval, i.e. driver
                               /// call, in seconds.
const double WALLFOLLOWDIST = 0.5; ///< Preferred wall following distance in meters.
const double STOP_WALLFOLLOWDIST = 0.2; ///< Stop distance in meters.
const double WALLLOSTDIST  = 1.5; ///< Wall attractor in meters before loosing walls.
const double SHAPE_DIST = 0.3; ///< Min Radius from sensor for robot shape.
// Laserranger
const double LMAXANGLE = 240; ///< Laser max angle in degree
const int BEAMCOUNT = 2; ///< Number of laser beams taken for one average distance measurement
const double DEGPROBEAM   = 0.3515625; ///< 360./1024. in degree per laserbeam
const double LPMAX     = 5.0;  ///< max laser range in meters
const double COS45     = 0.83867056795; ///< Cos(33);
const double INV_COS45 = 1.19236329284; ///< 1/COS45
const double DIAGOFFSET  = 0.1;  ///< Laser to sonar diagonal offset in meters.
const double HORZOFFSET  = 0.15; ///< Laser to sonar horizontal offset in meters.
const double MOUNTOFFSET = 0.1;  ///< Sonar vertical offset at back for laptop mount.
const int LMIN  = 175;/**< LEFT min angle.       */ const int LMAX  = 240; ///< LEFT max angle.
const int LFMIN = 140;/**< LEFTFRONT min angle.  */ const int LFMAX = 175; ///< LEFTFRONT max angle.
const int FMIN  = 100;/**< FRONT min angle.      */ const int FMAX  = 140; ///< FRONT max angle.
const int RFMIN = 65; /**< RIGHTFRONT min angle. */ const int RFMAX = 100; ///< RIGHTFRONT max angle.
const int RMIN  = 0;  /**< RIGHT min angle.      */ const int RMAX  = 65;  ///< RIGHT max angle.
// }}} Parameters

/// This class represents a robot.
/// The robot object provides wall following behaviour.
class Robot {
private:
  PlayerClient    *robot;
#ifdef LASER
  RangerProxy     *lp; ///< New in Player-3.x: hukoyo laser only via ranger IF
  //LaserProxy     *lp; ///< New in Player-3.x: hukoyo laser only via ranger IF
#endif
  SonarProxy      *sp;
  Position2dProxy *pp;
  /// Current behaviour of the robot.
  enum StateType {  // {{{
    WALL_FOLLOWING,
    COLLISION_AVOIDANCE,
    WALL_SEARCHING,
    BALL_TRACKING
  };  // }}}
  /// Used for simple range area distinction.
  enum viewDirectType { // {{{
    LEFT,
    RIGHT,
    FRONT,
    BACK,
    LEFTFRONT,
    RIGHTFRONT,
    LEFTREAR,
    RIGHTREAR,
    ALL
  };  // }}}
  int       robotID; ///< Global robot identifier
  double    speed; ///< Current robot speed
  double    turnrate; ///< Current robot turnrate
  double    tmp_turnrate; ///< Used for behavior turnrate fusion
  double    trackTurnrate; ///< Zero or tracking the ball turnrate
  double    trackSpeed; ///< Tracking ball speed
  StateType currentState; ///< Current robot state

  /// Returns the minimum distance of the given arc.
  /// Algorithm calculates the average of BEAMCOUNT beams
  /// to define a minimum value per degree.
  /// @param Range of angle (degrees)
  /// @return Minimum distance in range
  inline double getDistanceLas ( int minAngle, int maxAngle )
  {
    double minDist         = LPMAX; ///< Min distance in the arc.
    double distCurr        = LPMAX; ///< Current distance of a laser beam
#ifdef LASER
    if ( !(minAngle<0 || maxAngle<0 || minAngle>=maxAngle || minAngle>=LMAXANGLE || maxAngle>LMAXANGLE ) ) {

      const int minBIndex = (int)(minAngle/DEGPROBEAM); ///< Beam index of min deg.
      const int maxBIndex = (int)(maxAngle/DEGPROBEAM); ///< Beam index of max deg.
      double sumDist     = 0.; ///< Sum of BEAMCOUNT beam's distance.
      double averageDist = LPMAX; ///< Average of BEAMCOUNT beam's distance.

      for (int beamIndex=minBIndex; beamIndex<maxBIndex; beamIndex++) {
        distCurr = lp->GetRange(beamIndex);
        distCurr<0.02 ? sumDist+=LPMAX : sumDist+=distCurr;
        //sumDist += lp->GetRange(beamIndex);
        if((beamIndex-minBIndex) % BEAMCOUNT == 1) { ///< On each BEAMCOUNT's beam..
          averageDist = sumDist/BEAMCOUNT; ///< Calculate the average distance.
          sumDist = 0.; ///< Reset sum of beam average distance
          // Calculate the minimum distance for the arc
          averageDist<minDist ? minDist=averageDist : minDist;
        }
#ifdef DEBUG_LASER // {{{
        std::cout << "beamInd: " << beamIndex
          << "\tsumDist: " << sumDist
          << "\taveDist: " << averageDist
          << "\tminDist: " << minDist << std::endl;
#endif // }}}
      }
    }
#endif
  return minDist;
  }

  /// Returns the minimum distance of the given view direction.
  /// Robot shape shall be considered here by weighted SHAPE_DIST.
  /// Derived arcs, sonars and weights from graphic "PioneerShape.fig".
  /// NOTE: ALL might be slow due recursion, use it only for debugging!
  /// @param Robot view direction
  /// @return Minimum distance of requested view Direction
  inline double getDistance( viewDirectType viewDirection )
  {
    // Scan safety areas for walls
    switch (viewDirection) {
      case LEFT      : return PlayerCc::min(getDistanceLas(LMIN,  LMAX) -HORZOFFSET-SHAPE_DIST, PlayerCc::min(sp->GetScan(0), sp->GetScan(15))-SHAPE_DIST);
      case RIGHT     : return PlayerCc::min(getDistanceLas(RMIN,  RMAX) -HORZOFFSET-SHAPE_DIST, PlayerCc::min(sp->GetScan(7), sp->GetScan(8)) -SHAPE_DIST);
      case FRONT     : return PlayerCc::min(getDistanceLas(FMIN,  FMAX)            -SHAPE_DIST, PlayerCc::min(sp->GetScan(3), sp->GetScan(4)) -SHAPE_DIST);
      case RIGHTFRONT: return PlayerCc::min(getDistanceLas(RFMIN, RFMAX)-DIAGOFFSET-SHAPE_DIST, PlayerCc::min(sp->GetScan(5), sp->GetScan(6)) -SHAPE_DIST);
      case LEFTFRONT : return PlayerCc::min(getDistanceLas(LFMIN, LFMAX)-DIAGOFFSET-SHAPE_DIST, PlayerCc::min(sp->GetScan(1), sp->GetScan(2)) -SHAPE_DIST);
      case BACK      : return PlayerCc::min(sp->GetScan(11), sp->GetScan(12))-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
      case LEFTREAR  : return PlayerCc::min(sp->GetScan(13), sp->GetScan(14))-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
      case RIGHTREAR : return PlayerCc::min(sp->GetScan(9) , sp->GetScan(10))-MOUNTOFFSET-SHAPE_DIST; // Sorry, only sonar at rear
      case ALL       : return PlayerCc::min(getDistance(LEFT),
                           PlayerCc::min(getDistance(RIGHT),
                             PlayerCc::min(getDistance(FRONT),
                               PlayerCc::min(getDistance(BACK),
                                 PlayerCc::min(getDistance(RIGHTFRONT),
                                   PlayerCc::min(getDistance(LEFTFRONT),
                                     PlayerCc::min(getDistance(LEFTREAR), getDistance(RIGHTREAR) )))))));
      default: return 0.; // Should be recognized if happens
    }
  }

  /// Calculates the turnrate from range measurement and minimum wall follow
  /// distance.
  /// @param Current state of the robot.
  /// @returns Turnrate to follow wall.
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
#ifdef DEBUG_STATE  // {{{
    std::cout << "WALLFOLLOW" << std::endl;
#endif  // }}}

    // Normalize turnrate
    turnrate = limit(turnrate, -dtor(TURN_RATE), dtor(TURN_RATE));

    // Go straight if no wall is in distance (front, left and left front)
    if (DistLFov  >= WALLLOSTDIST  &&
        DistL     >= WALLLOSTDIST  &&
        DistLRear >= WALLLOSTDIST     )
    {
      turnrate = 0;
      *currentState = WALL_SEARCHING;
#ifdef DEBUG_STATE  // {{{
      std::cout << "WALL_SEARCHING" << std::endl;
#endif  // }}}
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
  inline void collisionAvoid ( double * turnrate, StateType * currentState)
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
#ifdef DEBUG_STATE  // {{{
      std::cout << "COLLISION_AVOIDANCE" << std::endl;
#endif  // }}}
    }
  }

  /// @todo Code review
  inline double calcspeed ( void )
  {
    double tmpMinDistFront = PlayerCc::min(getDistance(LEFTFRONT), PlayerCc::min(getDistance(FRONT), getDistance(RIGHTFRONT)));
    double tmpMinDistBack  = PlayerCc::min(getDistance(LEFTREAR), PlayerCc::min(getDistance(BACK), getDistance(RIGHTREAR)));
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

  /// Checks if turning the robot is not causing collisions.
  /// Implements more or less a rotation policy which decides depending on
  /// obstacles at the 4 robot edge surounding spots
  /// To not interfere to heavy to overall behaviour turnrate is only inverted (or
  /// set to zero)
  /// @param Turnrate
  /// @todo Code review
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
#ifdef LASER
    lp = new RangerProxy(robot, id);
    //lp    = new LaserProxy(robot, id);
#endif
    sp    = new SonarProxy(robot, id);
    robotID      = id;
    currentState = WALL_FOLLOWING;
    pp->SetMotorEnable(true);
    trackTurnrate = TRACKING_NO; // Disable tracking camera targets be default
  }

  inline void update ( void ) {
      robot->Read(); ///< This blocks until new data comes; 10Hz by default
  }
  inline void plan ( void ) {
#ifdef DEBUG_SONAR  // {{{
    std::cout << std::endl;
    for(int i=0; i< 16; i++)
      std::cout << "Sonar " << i << ": " << sp->GetScan(i) << std::endl;
#endif  // }}}
    if ( trackTurnrate == TRACKING_NO ) { ///< Check if ball is not detected in camera FOV

      // (Left) Wall following
      turnrate = wallfollow(&currentState);
      // Collision avoidance overrides other turnrate if neccessary!
      collisionAvoid(&turnrate, &currentState);

    } else {
      // Track the ball
      currentState = BALL_TRACKING;
      std::cout << "BALL_TRACKING" << std::endl;
      turnrate = trackTurnrate;
      speed    = trackSpeed;
    }

    // Set speed dependend on the wall distance
    speed = calcspeed();

    // Fusion speed with ball tracking: lower wins
    speed>trackSpeed ? speed=trackSpeed : speed;

    // Check if rotating is safe
    checkrotate(&tmp_turnrate);

    // Fusion of the vectors makes a smoother trajectory
    turnrate = (tmp_turnrate + turnrate) / 2;
#ifdef DEBUG_STATE  // {{{
    std::cout << "turnrate/speed/state:\t" << turnrate << "\t" << speed << "\t"
      << currentState << std::endl;
#endif  // }}}
#ifdef DEBUG_DIST // {{{
    std::cout << "Laser (l/lf/f/rf/r/rb/b/lb):\t" << getDistanceLas(LMIN, LMAX)-HORZOFFSET << "\t"
      << getDistanceLas(LFMIN, LFMAX)-DIAGOFFSET  << "\t"
      << getDistanceLas(FMIN,  FMAX)              << "\t"
      << getDistanceLas(RFMIN, RFMAX)-DIAGOFFSET  << "\t"
      << getDistanceLas(RMIN,  RMAX) -HORZOFFSET  << "\t"
      << "XXX"                                    << "\t"
      << "XXX"                                    << "\t"
      << "XXX"                                    << std::endl;
    std::cout << "Sonar (l/lf/f/rf/r/rb/b/lb):\t" << PlayerCc::min(sp->GetScan(15),sp->GetScan(0)) << "\t"
      << PlayerCc::min(sp->GetScan(1), sp->GetScan(2))              << "\t"
      << PlayerCc::min(sp->GetScan(3), sp->GetScan(4))              << "\t"
      << PlayerCc::min(sp->GetScan(5), sp->GetScan(6))              << "\t"
      << PlayerCc::min(sp->GetScan(7), sp->GetScan(8))              << "\t"
      << PlayerCc::min(sp->GetScan(9), sp->GetScan(10))-MOUNTOFFSET << "\t"
      << PlayerCc::min(sp->GetScan(11),sp->GetScan(12))-MOUNTOFFSET << "\t"
      << PlayerCc::min(sp->GetScan(13),sp->GetScan(14))-MOUNTOFFSET << std::endl;
    std::cout << "Shape (l/lf/f/rf/r/rb/b/lb):\t" << getDistance(LEFT) << "\t"
      << getDistance(LEFTFRONT)  << "\t"
      << getDistance(FRONT)      << "\t"
      << getDistance(RIGHTFRONT) << "\t"
      << getDistance(RIGHT)      << "\t"
      << getDistance(RIGHTREAR)  << "\t"
      << getDistance(BACK)       << "\t"
      << getDistance(LEFTREAR)   << std::endl;
#endif // }}}
#ifdef DEBUG_POSITION // {{{
    std::cout << pp->GetXPos() << "\t" << pp->GetYPos() << "\t" << rtod(pp->GetYaw()) << std::endl;
#endif  // }}}
  }
  /// Command the motors
  inline void execute() {
    std::cout << "SET SPEED/TURN:\t" << speed << "\t" << turnrate << std::endl;
    pp->SetSpeed(speed, turnrate); }
  void go() {
    this->update();
    this->plan();
    this->execute();
  }
  /// Set turnrate in radians
  /// @param Turnrate in radians, '0' will do wall follow
  /// @todo Make thread safe
  void setTurnrate( double vl_turnrate ) { trackTurnrate = vl_turnrate; }
  /// Set Robot speed in meters per sec
  /// @param vl_speed Robot speed in meters per sec
  /// @todo Make thread safe
  void setSpeed ( double vl_speed ) { trackSpeed = vl_speed; }
  /// Get global robot orientation in radians
  double getOrientation ( void ) { return pp->GetYaw(); }
}; // Class Robot
//=================
#ifndef OPENCV //{{{ Dummy for compilation w/o opencv
  struct Ball
  {
    int num;
    double* dist;
    double* angle;
  };
#endif //}}}
// Simulation of the camera's driver call
const int width=1280; ///< Camera width resolution definition
const int height=960; ///< Camera height resolution definition
#ifdef OPENCV //{{{
  Single1394 c1394;
  BallFinder fb;
#endif //}}}

/// Read the camera driver's ball tracking information
/// Call of the camera driver may take some time (~1sec)!
/// @return Pointer to dynamic ball information object.
ts_Ball * getBallInfo ( void ) {
  static ts_Ball ballInfo;
  Ball *balls;

#ifdef OPENCV //{{{
  c1394.captureImage();
  balls=fb.DetectBall(c1394.captureBuf);
#endif //}}}
  if ( balls->num > 0 ) {
     ballInfo.angle = balls->angle[0];
     ballInfo.dist  = balls->dist[0];
     ballInfo.num   = balls->num;
     delete []balls->angle;
     delete []balls->dist;
  } else {
    ballInfo.num = 0;
  }
  delete balls;

  assert( abs(ballInfo.angle) <= M_PI);
  assert( ballInfo.dist >= 0 );
  assert( ballInfo.num >= 0 );

  return &ballInfo;
}
/// Calculates a relative turnrate towards goal angle
/// @param curOrientation Current robot global orientation angle
/// @param goalAngle Relative goal orientation angle
/// @param newGoalFlag Marks current goal as new or not
/// @return An estimated goal angle.
/// Caution returned turnrate is only +/- M_PI bound!!
/// Expects new goal flag to be set the first call!
/// Will falsify it immediately and work on that goal until flag is (re-) set
/// again
double approxTurnrate(double curOrientation, double goalAngle, bool * newGoalFlag) {

  static double goalYaw = 0.;
  static double oldGoalAngle = 0.;
  double approxTurnrate = 0.;

  assert( abs(curOrientation) <= 2*M_PI );
  assert( abs(goalAngle) <= M_PI );

  // MARK NEW GOAL
  if (*newGoalFlag == true) { // New goal defined
    *newGoalFlag = false; // Mark current goal being tracked
    // Normalized absolute goal angle
    goalYaw = fmod((curOrientation + goalAngle), 2*M_PI);
    oldGoalAngle = goalAngle;
  }
  std::cout << "Goal YAW delta:\t" << fabs(curOrientation - goalYaw) << std::endl;
  // Normalized absolute goal orientation
  if (fabs(curOrientation - goalYaw) < dtor(YAW_TOLERANCE)){
    approxTurnrate = 0; // Heading to the goal
  } else { // Turning towards goal
    (oldGoalAngle<0) ? (approxTurnrate=-dtor(TRACK_ROT)) : (approxTurnrate=dtor(TRACK_ROT));
  }
  //if (oldGoalAngle < 0) { // to the right/*{{{*/
    //approxTurnrate = -dtor(TRACK_ROT);
  //} else { // to the left
    //approxTurnrate = dtor(TRACK_ROT);
  //}/*}}}*/

  std::cout << "CurPos/GoalPos/Turnrate:\t"
    << rtod(curOrientation) << "\t"
    << rtod(goalYaw) << "\t"
    << rtod(approxTurnrate) << std::endl;

  assert( abs(approxTurnrate) <= dtor(TRACK_ROT) );

  return approxTurnrate;
  //return -dtor(10);
}
/// Abstraction layer between robot and camera.
/// Gets goal coordinates from camera device and directs the robot to it
/// accordingly.
/// Camera functions are called in here.
/// @param Pointer to robot of type @ref Robot to command.
void trackBall (Robot * robot)
{
  ts_Ball * ballInfo; // Pointer to the ball coordinates from camera
  double vl_turnrate = 0; // Local calculated robot write turnrate
  static double robPrevTurnrate = 0.; // Last turnrate before this one
  double goalAngle = 0.; // Relative turnrate to goal
  double goalDist  = LPMAX; // Relative distance to goal
  timeval curTime; // Current system time
  double curTimeSec = 0.; // Current time in seconds
  double curOrientation = 0.; // Robot current global orientation
  static double lastFound = 0.; // Time when ball was last found
  static double lastBallReq = 0.; // Time when ball was last searched
  bool newGoalFlag = false;

  // Get current time
  gettimeofday(&curTime, 0);
  curTimeSec = curTime.tv_sec + curTime.tv_usec/1e6;
  assert( curTimeSec >= 1 );

  // Current global orientation of robot
  curOrientation = robot->getOrientation();
  assert( abs(curOrientation) <= 2*M_PI );

  assert( curTimeSec >= lastBallReq );
  // Call driver only once each BALLREQINT
  if(curTimeSec-lastBallReq >= BALLREQINT) {

    ballInfo = getBallInfo(); // Call the camera driver
#ifdef DEBUG_CAM //{{{
   std::cout << "Ball ctime/dist./angle/num:\t"
     << curTimeSec << "\t"
     << ballInfo->dist << "\t"
     << ballInfo->angle << "\t"
     << ballInfo->num << std::endl;
#endif //}}}
    lastBallReq = curTimeSec; // Reset request time

    if ( ballInfo->num == 0 ) { // When no balls have been found
#ifdef DEBUG_CAM //{{{
      std::cout << "NO BALL FOUND" << std::endl;
#endif //}}}
      assert( curTimeSec >= lastFound );
      if(curTimeSec-lastFound > BALLTIMEOUT) { // When beyond the time out
#ifdef DEBUG_CAM //{{{
        std::cout << "  BALLTRACKING TIMEOUT (sec)\t" << BALLTIMEOUT << std::endl;
#endif //}}}
        vl_turnrate = TRACKING_NO; // Robot will do another task
      }
    } else { // A ball has been found
      lastFound = curTimeSec; // Reset found time
      goalAngle = ballInfo->angle;
      goalDist  = ballInfo->dist;
      if (goalDist < DIST_TOLERANCE) {
        robot->setSpeed(0); // stop
      } else {
        robot->setSpeed(VEL); // cruise
      }
      newGoalFlag = true; // Mark as new goal angle
#ifdef DEBUG_CAM //{{{
      std::cout << "BALL FOUND at angle/time:\t"
        << goalAngle << "\t"
        << curTimeSec << std::endl;
#endif //}}}
    }
  } else { // else estimate to old goal position
    std::cout << "Not yet ball request" << std::endl;
  }

  // Calculate track turnrate always except when not tracking the goal
  //if (vl_turnrate != TRACKING_NO) {
    vl_turnrate = limit(approxTurnrate(curOrientation, goalAngle, &newGoalFlag),
        -dtor(TRACK_ROT), dtor(TRACK_ROT));
  //}
  robPrevTurnrate = vl_turnrate; // Remember turnrate for next cycle
  // Give the robot a new target, '0' for doing default task
  robot->setTurnrate(vl_turnrate);
#ifdef DEBUG_CAM //{{{
  std::cout << "SET TURNRATE: " << vl_turnrate << std::endl;
#endif //}}}
}
//=================
int main ( void ) {
  try {
#ifdef OPENCV //{{{
    if (!c1394.initCam(width,height)) {
      printf("Initializing Camera failed.\n");
      return 0;
    }
#endif //}}}

    Robot r0("localhost", 6665, 0);
    std::cout.precision(2);

#ifdef OPENCV //{{{
    c1394.initFocus();
    fb.Init(width,height);
#endif //}}}

    while (true) {
      r0.go();
#ifdef OPENCV //{{{
      trackBall(&r0); ///< Let the robot trace the ball if any
#endif //}}}
    }
  } catch (PlayerCc::PlayerError e) {
    std::cerr << e << std::endl; // let's output the error
#ifdef OPENCV //{{{
    fb.Over();
    c1394.cleanup();
#endif //}}}
    return -1;
  }

  return 1;
}
