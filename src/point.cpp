#include <cmath>
#include "point.h"

namespace geodesy
{

Point::Point(const Latitude &latitude, const Longitude &longitude)
  :
    m_latitude(latitude),
    m_longitude(longitude)
{
}

const Latitude &Point::latitude() const
{
  return m_latitude;
}

const Longitude &Point::longitude() const
{
  return m_longitude;
}

double Point::sphericalDistanceTo(const Point &point, double radius) const
{
  // see mathforum.org/library/drmath/view/51879.html for derivation

  auto phi1 = m_latitude.radians();
  auto lambda1 = m_longitude.radians();
  auto phi2 = point.latitude().radians();
  auto lambda2 = point.longitude().radians();
  auto deltaPhi = phi2 - phi1;
  auto deltaLambda = lambda2 - lambda1;

  auto a = std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
           std::cos(phi1) * std::cos(phi2) *
           std::sin(deltaLambda / 2.0) * std::sin(deltaLambda / 2.0);
  auto c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  return radius * c;
}

double Point::sphericalBearingTo(const Point &point) const
{
  // see mathforum.org/library/drmath/view/55417.html for derivation

  auto phi1 = m_latitude.radians();
  auto phi2 = point.latitude().radians();
  auto deltaLambda = point.longitude().radians() - longitude().radians();
  auto y = std::sin(deltaLambda) * std::cos(phi2);
  auto x = std::cos(phi1) * std::sin(phi2) -
           std::sin(phi1) * std::cos(phi2) * std::cos(deltaLambda);
  auto theta = std::atan2(y, x);

  return fmod(Coordinate::toDegrees(theta) + 360.0, 360.0);
}

double Point::sphericalFinalBearingTo(const Point &point) const
{
  // Get initial bearing from destination point to this point & reverse it by adding 180°
  return fmod(point.sphericalBearingTo(*this) + 180.0, 360.0);
}

Point Point::sphericalMidpointTo(const Point &point) const
{
  // see mathforum.org/library/drmath/view/51822.html for derivation

  auto phi1 = m_latitude.radians();
  auto lambda1 = m_longitude.radians();
  auto phi2 = point.latitude().radians();
  auto deltaLambda = point.longitude().radians() - m_longitude.radians();

  auto Bx = std::cos(phi2) * std::cos(deltaLambda);
  auto By = std::cos(phi2) * std::sin(deltaLambda);

  auto x = std::sqrt((std::cos(phi1) + Bx) * (std::cos(phi1) + Bx) + By * By);
  auto y = std::sin(phi1) + std::sin(phi2);
  auto phi3 = std::atan2(y, x);

  auto lambda3 = lambda1 + std::atan2(By, std::cos(phi1) + Bx);

  return Point(Coordinate::toDegrees(phi3),
               fmod(Coordinate::toDegrees(lambda3) + 540.0, 360.0) - 180.0); // normalise to -180..+180°
}

Point Point::sphericalIntermediatePointTo(const Point &point, double fraction) const
{
  auto phi1 = m_latitude.radians();
  auto lambda1 = m_longitude.radians();
  auto phi2 = point.latitude().radians();
  auto lambda2 = point.longitude().radians();
  auto sinPhi1 = std::sin(phi1);
  auto cosPhi1 = std::cos(phi1);
  auto sinlambda1 = std::sin(lambda1);
  auto coslambda1 = std::cos(lambda1);
  auto sinPhi2 = std::sin(phi2);
  auto cosPhi2 = std::cos(phi2);
  auto sinLambda2 = std::sin(lambda2);
  auto cosLambda2 = std::cos(lambda2);

  // Distance between points
  auto deltaPhi = phi2 - phi1;
  auto deltaLambda = lambda2 - lambda1;
  auto a = std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
           std::cos(phi1) * std::cos(phi2) * std::sin(deltaLambda / 2.0) * std::sin(deltaLambda / 2.0);
  auto sigma = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  auto A = std::sin((1.0 - fraction) * sigma) / std::sin(sigma);
  auto B = std::sin(fraction * sigma) / std::sin(sigma);

  auto x = A * cosPhi1 * coslambda1 + B * cosPhi2 * cosLambda2;
  auto y = A * cosPhi1 * sinlambda1 + B * cosPhi2 * sinLambda2;
  auto z = A * sinPhi1 + B * sinPhi2;

  auto phi3 = std::atan2(z, std::sqrt(x * x + y * y));
  auto lambda3 = std::atan2(y, x);

  return Point(Coordinate::toDegrees(phi3),
               fmod(Coordinate::toDegrees(lambda3) + 540.0, 360.0) - 180.0); // normalise lon to -180..+180°
}

}

