#include <iostream>
#include <cmath>
#include <cstdlib>   // for srand and rand
#include <ctime>     // for time

#define DEBUG_LASER

using namespace std;

int main (void)
{
  const int maxAngle = 240;
  const int minAngle =   0;
  const double LPMAX = 5.0;
  const double DEGPROBEAM = 0.3515625; // 360./1024. in degree perrIndexaserbeam
  const int UPIND = 682;
  double lp[UPIND];

  std::cout.precision(2);

  srand(time(0));  // initialize seed "randomly"

  for (int i=0; i<UPIND; i++) {
    lp[i] = (rand() % 500 + 60)*0.01;  // fill the array in order
    cout << i << "\t" << lp[i] << endl;
  }

  {
    const int minBIndex = (int)(minAngle/DEGPROBEAM); ///< Beam index of min deg.
    const int maxBIndex = (int)(maxAngle/DEGPROBEAM); ///< Beam index of max deg.
    const int BEAMCOUNT = 2; ///< Number of laser beams taken for one average distance measurement
    double minDist     = LPMAX; ///< Min distance in the arc.
    double sumDist     = 0.; ///< Sum of BEAMCOUNT beam's distance.
    double averageDist = LPMAX; ///< Average of BEAMCOUNT beam's distance.

    for (int beamIndex=minBIndex; beamIndex<maxBIndex; beamIndex++) {
      sumDist += lp[beamIndex];
      if((beamIndex-minBIndex) % BEAMCOUNT == 1) { ///< On each BEAMCOUNT's beam..
        averageDist = sumDist/BEAMCOUNT; ///< Calculate the average distance.
        sumDist = 0.; ///< Reset sum of beam average distance
        // Calculate the minimum distance for the arc
        averageDist<minDist ? minDist=averageDist : minDist;
      }
#ifdef DEBUG_LASER
      cout
        << BEAMCOUNT << "\t"
        << beamIndex << "\t"
        << lp[beamIndex] << "\t"
        << sumDist << "\t"
        << averageDist << "\t"
        << minDist << endl;
#endif
    }
  }

return 0;
}
