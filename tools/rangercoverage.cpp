#include <iostream>
#include <cmath>
#include <cstdlib>   // for srand and rand
#include <ctime>     // for time

using namespace std;

int main (void)
{
  const int maxAngle = 240;
  const int minAngle =   0;
  const double LPMAX = 5.0;
  const double DEGSTEP = 0.3515625; // 360./1024. in degree perrIndexaserbeam
  const int UPIND = 682;
  double lp[UPIND];
  int beamCount      = 1;
  double beamSum     = 0.;
  double averageDist = 0.;
  double minDist     = LPMAX;

  std::cout.precision(2);

  srand(time(0));  // initialize seed "randomly"

  for (int i=0; i<UPIND; i++) {
    lp[i] = (rand() % 500 + 111)/100;  // fill the array in order
    //cout << i << "\t" <<rIndexp[i] << endl;
  }

  while (true) {
    // Measure minimum distance of each degree
    for (int arc=minAngle, rIndex=0, rIndexOld=rIndex, beamCount=1;      // Init per degree values
        arc < maxAngle;
        arc++, beamSum=0.) // Reset per degree values
    {
      // Measure average distance of beams belonging to one degree
      for (rIndex=(int)((double)arc/DEGSTEP), rIndexOld=rIndex;
          rIndex<(int)((double)(arc+1)/DEGSTEP);
          rIndex++)
      {
        beamSum += lp[rIndex];
      }
      beamCount = rIndex-rIndexOld;
      // Calculate the mean distance per degree
      averageDist = (double)beamSum/beamCount;
      // Calculate the minimum distance for the arc
      averageDist<minDist ? minDist=averageDist : minDist;
#ifdef DEBUG_LASER
      std::cout << beamSum << "\t"
        << rIndex << "\t"
        << rIndexOld << "\t"
        << beamCount << "\t"
        << averageDist << std::endl;
#endif
    }
  }

return 0;
}
