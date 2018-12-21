#include "point.h"

using namespace geodesy;

int main()
{
  // Tests:
  Point p1{ 52.205, 0.119 };
  Point p2{ 48.857, 2.351 };
  auto d = p1.sphericalDistanceTo(p2); // 404.3 km

  auto b1 = p1.sphericalBearingTo(p2); // 156.2°

  return 0;
}
