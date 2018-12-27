#include "point.h"

using namespace geodesy;

int main()
{
  // Tests:
  Point p1{ 52.205, 0.119 };
  Point p2{ 48.857, 2.351 };
  auto d = p1.sphericalDistanceTo(p2); // 404.3 km

  auto b1 = p1.sphericalBearingTo(p2);      // 156.2°
  auto b2 = p1.sphericalFinalBearingTo(p2); // 157.9°
  auto pMid = p1.sphericalMidpointTo(p2);   // 50.5363°N, 001.2746°E

  return 0;
}
