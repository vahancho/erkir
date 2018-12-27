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

}

