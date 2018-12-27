/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018 Vahan Aghajanyan <vahancho@gmail.com>                       *
*                                                                                 *
*  Permission is hereby granted, free of charge, to any person obtaining a copy   *
*  of this software and associated documentation files (the "Software"), to deal  *
*  in the Software without restriction, including without limitation the rights   *
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell      *
*  copies of the Software, and to permit persons to whom the Software is          *
*  furnished to do so, subject to the following conditions:                       *
*                                                                                 *
*  The above copyright notice and this permission notice shall be included in all *
*  copies or substantial portions of the Software.                                *
*                                                                                 *
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR     *
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,       *
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE    *
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER         *
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,  *
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE  *
*  SOFTWARE.                                                                      *
***********************************************************************************/

#include <cmath>
#include <algorithm>

#include "point.h"

namespace geodesy
{

Point::Point()
  :
    m_latitude(0.0),
    m_longitude(0.0),
    m_isValid(true)
{}

Point::Point(const Latitude &latitude, const Longitude &longitude)
  :
    m_latitude(latitude),
    m_longitude(longitude),
    m_isValid(true)
{}

const Latitude &Point::latitude() const
{
  return m_latitude;
}

const Longitude &Point::longitude() const
{
  return m_longitude;
}

bool Point::isValid() const
{
  return m_isValid;
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

Point Point::sphericalDestinationPoint(double distance, double bearing, double radius) const
{
  // see mathforum.org/library/drmath/view/52049.html for derivation

  auto sigma = distance / radius; // angular distance in radians
  auto theta = Coordinate::toRadians(bearing);

  auto phi1 = m_latitude.radians();
  auto lambda1 = m_longitude.radians();

  auto sinPhi1 = std::sin(phi1);
  auto cosPhi1 = std::cos(phi1);
  auto sinSigma = std::sin(sigma);
  auto cosSigma = std::cos(sigma);
  auto sinTheta = std::sin(theta);
  auto cosTheta = std::cos(theta);

  auto sinPhi2 = sinPhi1 * cosSigma + cosPhi1 * sinSigma * cosTheta;
  auto phi2 = std::asin(sinPhi2);
  auto y = sinTheta * sinSigma * cosPhi1;
  auto x = cosSigma - sinPhi1 * sinPhi2;
  auto lambda2 = lambda1 + std::atan2(y, x);

  return Point(Coordinate::toDegrees(phi2),
               fmod(Coordinate::toDegrees(lambda2) + 540.0, 360.0) - 180.0); // normalise to -180..+180°
}

Point Point::sphericalIntersection(const Point &p1, double brng1,
                                   const Point &p2, double brng2)
{
  // see www.edwilliams.org/avform.htm#Intersection

  auto phi1 = p1.latitude().radians();
  auto lambda1 = p1.longitude().radians();
  auto phi2 = p2.latitude().radians();
  auto lambda2 = p2.longitude().radians();
  auto theta13 = Coordinate::toRadians(brng1);
  auto theta23 = Coordinate::toRadians(brng2);
  auto deltaPhi = phi2 - phi1;
  auto deltaLambda = lambda2 - lambda1;

  // angular distance p1-p2
  auto sigma12 = 2.0 * std::asin(std::sqrt(std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
                                 std::cos(phi1) * std::cos(phi2) * std::sin(deltaLambda / 2.0) * std::sin(deltaLambda / 2.0)));
  if (sigma12 == 0) {
    return Point();
  }

  // initial/final bearings between points
  auto costhetaa = (std::sin(phi2) - std::sin(phi1)*std::cos(sigma12)) / (std::sin(sigma12) * std::cos(phi1));
  auto costhetab = (std::sin(phi1) - std::sin(phi2)*std::cos(sigma12)) / (std::sin(sigma12) * std::cos(phi2));
  auto thetaA = std::acos(std::min(std::max(costhetaa, -1.0), 1.0)); // protect against rounding errors
  auto thetaB = std::acos(std::min(std::max(costhetab, -1.0), 1.0)); // protect against rounding errors

  auto theta12 = std::sin(lambda2 - lambda1) > 0.0 ? thetaA : 2.0 * Coordinate::pi() - thetaA;
  auto theta21 = std::sin(lambda2 - lambda1) > 0.0 ? 2.0 * Coordinate::pi() - thetaB : thetaB;

  auto alpha1 = theta13 - theta12; // angle 2-1-3
  auto alpha2 = theta21 - theta23; // angle 1-2-3

  if (std::sin(alpha1) == 0.0 && std::sin(alpha2) == 0.0) {
    return Point(); // Infinite intersections
  }
  if (std::sin(alpha1) * std::sin(alpha2) < 0.0) {
    return Point(); // Ambiguous intersection
  }

  auto alpha3 = std::acos(-std::cos(alpha1) * std::cos(alpha2) + std::sin(alpha1) * std::sin(alpha2) * std::cos(sigma12));
  auto sigma13 = std::atan2(std::sin(sigma12) * std::sin(alpha1) * std::sin(alpha2), std::cos(alpha2) + std::cos(alpha1) * std::cos(alpha3));
  auto phi3 = std::asin(std::sin(phi1) * std::cos(sigma13) + std::cos(phi1) * std::sin(sigma13) * std::cos(theta13));
  auto deltaLambda13 = std::atan2(std::sin(theta13) * std::sin(sigma13) * std::cos(phi1), std::cos(sigma13) - std::sin(phi1) * std::sin(phi3));
  auto lambda3 = lambda1 + deltaLambda13;

  return Point(Coordinate::toDegrees(phi3),
               fmod(Coordinate::toDegrees(lambda3) + 540.0, 360.0) - 180); // normalise to -180..+180°
}

double Point::sphericalCrossTrackDistanceTo(const Point &pathStart, const Point &pathEnd,
                                            double radius) const
{
  auto d13 = pathStart.sphericalDistanceTo(*this, radius) / radius;
  auto theta13 = Coordinate::toRadians(pathStart.sphericalBearingTo(*this));
  auto theta12 = Coordinate::toRadians(pathStart.sphericalBearingTo(pathEnd));

  auto xt = std::asin(std::sin(d13) * std::sin(theta13 - theta12));

  return xt * radius;
}

double Point::sphericalAlongTrackDistanceTo(const Point &pathStart, const Point &pathEnd,
                                            double radius) const
{
  auto d13 = pathStart.sphericalDistanceTo(*this, radius) / radius;
  auto theta13 = Coordinate::toRadians(pathStart.sphericalBearingTo(*this));
  auto theta12 = Coordinate::toRadians(pathStart.sphericalBearingTo(pathEnd));

  auto xt = std::asin(std::sin(d13) * std::sin(theta13 - theta12));

  auto at = std::acos(std::cos(d13) / std::abs(std::cos(xt)));

  auto cosTheta = std::cos(theta12 - theta13);
  if (cosTheta == 0.0)
  {
    return 0.0;
  }

  auto dist = at * radius;
  return cosTheta > 0 ? dist : -dist;
}

}

