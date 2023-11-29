// clang-format off
/**
 * MIT License
 *
 * Copyright (c) 2018-2023 Vahan Aghajanyan
 * Copyright (c) 2002-2018 Chris Veness (Latitude/Longitude spherical geodesy tools | https://www.movable-type.co.uk/scripts/latlong.html)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
// clang-format on

#include "erkir/sphericalpoint.h"

#include <algorithm>
#include <cmath>

namespace erkir {
namespace spherical {

/// Normalise angle to -180..+180°

/**
 * @brief Normalise angle to -180..+180°.
 *
 * @param degree Decimal degree.
 * @return double
 */
static double normalizeAngle(double degree) {
  return std::fmod(degree + 540.0, 360.0) - 180.0;
}

Point::Point(const Latitude &latitude, const Longitude &longitude)
    : erkir::Point(latitude, longitude) {}

Point::Point() : erkir::Point() {}

double Point::distanceTo(const Point &point, double radius) const {
  // see mathforum.org/library/drmath/view/51879.html for derivation

  auto phi1 = latitude().radians();
  auto lambda1 = longitude().radians();
  auto phi2 = point.latitude().radians();
  auto lambda2 = point.longitude().radians();
  auto deltaPhi = phi2 - phi1;
  auto deltaLambda = lambda2 - lambda1;

  auto a = std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
           std::cos(phi1) * std::cos(phi2) * std::sin(deltaLambda / 2.0) *
               std::sin(deltaLambda / 2.0);
  auto c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  return radius * c;
}

double Point::bearingTo(const Point &point) const {
  // see mathforum.org/library/drmath/view/55417.html for derivation

  auto phi1 = latitude().radians();
  auto phi2 = point.latitude().radians();
  auto deltaLambda = point.longitude().radians() - longitude().radians();
  auto y = std::sin(deltaLambda) * std::cos(phi2);
  auto x = std::cos(phi1) * std::sin(phi2) -
           std::sin(phi1) * std::cos(phi2) * std::cos(deltaLambda);
  auto theta = std::atan2(y, x);

  return fmod(Coordinate::toDegrees(theta) + 360.0, 360.0);
}

double Point::finalBearingTo(const Point &point) const {
  // Get initial bearing from destination point to this point & reverse it by
  // adding 180°
  return fmod(point.bearingTo(*this) + 180.0, 360.0);
}

Point Point::midpointTo(const Point &point) const {
  // see mathforum.org/library/drmath/view/51822.html for derivation

  auto phi1 = latitude().radians();
  auto lambda1 = longitude().radians();
  auto phi2 = point.latitude().radians();
  auto deltaLambda = point.longitude().radians() - longitude().radians();

  auto Bx = std::cos(phi2) * std::cos(deltaLambda);
  auto By = std::cos(phi2) * std::sin(deltaLambda);

  auto x = std::sqrt((std::cos(phi1) + Bx) * (std::cos(phi1) + Bx) + By * By);
  auto y = std::sin(phi1) + std::sin(phi2);
  auto phi3 = std::atan2(y, x);

  auto lambda3 = lambda1 + std::atan2(By, std::cos(phi1) + Bx);

  return Point(Coordinate::toDegrees(phi3),
               normalizeAngle(Coordinate::toDegrees(
                   lambda3)));  // normalise to -180..+180°
}

Point Point::intermediatePointTo(const Point &point, double fraction) const {
  auto phi1 = latitude().radians();
  auto lambda1 = longitude().radians();
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
           std::cos(phi1) * std::cos(phi2) * std::sin(deltaLambda / 2.0) *
               std::sin(deltaLambda / 2.0);
  auto sigma = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));

  auto A = std::sin((1.0 - fraction) * sigma) / std::sin(sigma);
  auto B = std::sin(fraction * sigma) / std::sin(sigma);

  auto x = A * cosPhi1 * coslambda1 + B * cosPhi2 * cosLambda2;
  auto y = A * cosPhi1 * sinlambda1 + B * cosPhi2 * sinLambda2;
  auto z = A * sinPhi1 + B * sinPhi2;

  auto phi3 = std::atan2(z, std::sqrt(x * x + y * y));
  auto lambda3 = std::atan2(y, x);

  return Point(Coordinate::toDegrees(phi3),
               fmod(Coordinate::toDegrees(lambda3) + 540.0, 360.0) -
                   180.0);  // normalise lon to -180..+180°
}

Point Point::destinationPoint(double distance, double bearing,
                              double radius) const {
  // see mathforum.org/library/drmath/view/52049.html for derivation

  auto sigma = distance / radius;  // angular distance in radians
  auto theta = Coordinate::toRadians(bearing);

  auto phi1 = latitude().radians();
  auto lambda1 = longitude().radians();

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
               normalizeAngle(Coordinate::toDegrees(
                   lambda2)));  // normalise to -180..+180°
}

Point Point::intersection(const Point &p1, double brng1, const Point &p2,
                          double brng2) {
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
  auto sigma12 =
      2.0 * std::asin(std::sqrt(
                std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
                std::cos(phi1) * std::cos(phi2) * std::sin(deltaLambda / 2.0) *
                    std::sin(deltaLambda / 2.0)));
  if (sigma12 == 0) {
    return Point();
  }

  // initial/final bearings between points
  auto costhetaa = (std::sin(phi2) - std::sin(phi1) * std::cos(sigma12)) /
                   (std::sin(sigma12) * std::cos(phi1));
  auto costhetab = (std::sin(phi1) - std::sin(phi2) * std::cos(sigma12)) /
                   (std::sin(sigma12) * std::cos(phi2));
  auto thetaA = std::acos(std::min(std::max(costhetaa, -1.0),
                                   1.0));  // protect against rounding errors
  auto thetaB = std::acos(std::min(std::max(costhetab, -1.0),
                                   1.0));  // protect against rounding errors

  auto theta12 = std::sin(lambda2 - lambda1) > 0.0
                     ? thetaA
                     : 2.0 * Coordinate::pi() - thetaA;
  auto theta21 = std::sin(lambda2 - lambda1) > 0.0
                     ? 2.0 * Coordinate::pi() - thetaB
                     : thetaB;

  auto alpha1 = theta13 - theta12;  // angle 2-1-3
  auto alpha2 = theta21 - theta23;  // angle 1-2-3

  if (std::sin(alpha1) == 0.0 && std::sin(alpha2) == 0.0) {
    return Point();  // Infinite intersections
  }
  if (std::sin(alpha1) * std::sin(alpha2) < 0.0) {
    return Point();  // Ambiguous intersection
  }

  auto alpha3 =
      std::acos(-std::cos(alpha1) * std::cos(alpha2) +
                std::sin(alpha1) * std::sin(alpha2) * std::cos(sigma12));
  auto sigma13 =
      std::atan2(std::sin(sigma12) * std::sin(alpha1) * std::sin(alpha2),
                 std::cos(alpha2) + std::cos(alpha1) * std::cos(alpha3));
  auto phi3 = std::asin(std::sin(phi1) * std::cos(sigma13) +
                        std::cos(phi1) * std::sin(sigma13) * std::cos(theta13));
  auto deltaLambda13 =
      std::atan2(std::sin(theta13) * std::sin(sigma13) * std::cos(phi1),
                 std::cos(sigma13) - std::sin(phi1) * std::sin(phi3));
  auto lambda3 = lambda1 + deltaLambda13;

  return Point(Coordinate::toDegrees(phi3),
               normalizeAngle(Coordinate::toDegrees(
                   lambda3)));  // normalise to -180..+180°
}

double Point::crossTrackDistanceTo(const Point &pathStart, const Point &pathEnd,
                                   double radius) const {
  auto d13 = pathStart.distanceTo(*this, radius) / radius;
  auto theta13 = Coordinate::toRadians(pathStart.bearingTo(*this));
  auto theta12 = Coordinate::toRadians(pathStart.bearingTo(pathEnd));

  auto xt = std::asin(std::sin(d13) * std::sin(theta13 - theta12));

  return xt * radius;
}

double Point::alongTrackDistanceTo(const Point &pathStart, const Point &pathEnd,
                                   double radius) const {
  auto d13 = pathStart.distanceTo(*this, radius) / radius;
  auto theta13 = Coordinate::toRadians(pathStart.bearingTo(*this));
  auto theta12 = Coordinate::toRadians(pathStart.bearingTo(pathEnd));

  auto xt = std::asin(std::sin(d13) * std::sin(theta13 - theta12));

  auto at = std::acos(std::cos(d13) / std::abs(std::cos(xt)));

  auto cosTheta = std::cos(theta12 - theta13);
  if (cosTheta == 0.0) {
    return 0.0;
  }

  auto dist = at * radius;
  return cosTheta > 0 ? dist : -dist;
}

double Point::maxLatitude(double bearing) const {
  auto theta = Coordinate::toRadians(bearing);
  auto phi = latitude().radians();
  auto phiMax = std::acos(std::abs(std::sin(theta) * std::cos(phi)));

  return Coordinate::toDegrees(phiMax);
}

double Point::rhumbDistanceTo(const Point &point, double radius) const {
  // see www.edwilliams.org/avform.htm#Rhumb

  auto phi1 = latitude().radians();
  auto phi2 = point.latitude().radians();
  auto deltaPhi = phi2 - phi1;
  auto deltaLambda =
      std::abs(point.longitude().radians() - longitude().radians());

  // If dLon over 180° take shorter rhumb line across the anti-meridian:
  if (deltaLambda > Coordinate::pi()) {
    deltaLambda -= 2.0 * Coordinate::pi();
  }

  // On Mercator projection, longitude distances shrink by latitude; q is the
  // 'stretch factor' q becomes ill-conditioned along E-W line (0/0); use
  // empirical tolerance to avoid it
  auto deltaPsi = std::log(std::tan(phi2 / 2.0 + Coordinate::pi() / 4.0) /
                           std::tan(phi1 / 2.0 + Coordinate::pi() / 4.0));
  auto q = std::abs(deltaPsi) > 10e-12 ? deltaPhi / deltaPsi : std::cos(phi1);

  // Distance is Pythagoras on 'stretched' Mercator projection
  auto sigma = std::sqrt(deltaPhi * deltaPhi +
                         q * q * deltaLambda *
                             deltaLambda);  // angular distance in radians
  return sigma * radius;
}

double Point::rhumbBearingTo(const Point &point) const {
  auto phi1 = latitude().radians();
  auto phi2 = point.latitude().radians();
  auto deltaLambda = point.longitude().radians() - longitude().radians();
  // If dLon over 180° take shorter rhumb line across the anti-meridian:
  if (deltaLambda > Coordinate::pi()) {
    deltaLambda -= 2.0 * Coordinate::pi();
  }
  if (deltaLambda < -Coordinate::pi()) {
    deltaLambda += 2.0 * Coordinate::pi();
  }

  auto deltaPsi = std::log(std::tan(phi2 / 2.0 + Coordinate::pi() / 4.0) /
                           std::tan(phi1 / 2.0 + Coordinate::pi() / 4.0));

  auto theta = std::atan2(deltaLambda, deltaPsi);

  return Coordinate::wrap360(Coordinate::toDegrees(theta));
}

Point Point::rhumbDestinationPoint(double distance, double bearing,
                                   double radius) const {
  auto sigma = distance / radius;  // angular distance in radians
  auto phi1 = latitude().radians();
  auto lambda1 = longitude().radians();
  auto theta = Coordinate::toRadians(bearing);

  auto deltaPhi = sigma * std::cos(theta);
  auto phi2 = phi1 + deltaPhi;

  // check for some daft bugger going past the pole, normalise latitude if so
  if (std::abs(phi2) > Coordinate::pi() / 2.0) {
    phi2 = phi2 > 0 ? Coordinate::pi() - phi2 : -Coordinate::pi() - phi2;
  }

  auto deltaPsi = std::log(std::tan(phi2 / 2.0 + Coordinate::pi() / 4.0) /
                           std::tan(phi1 / 2.0 + Coordinate::pi() / 4.0));
  // E-W course becomes ill-conditioned with 0/0
  auto q = std::abs(deltaPsi) > 10e-12 ? deltaPhi / deltaPsi : std::cos(phi1);

  auto deltaLambda = sigma * std::sin(theta) / q;
  auto lambda2 = lambda1 + deltaLambda;

  return Point(Coordinate::toDegrees(phi2),
               normalizeAngle(Coordinate::toDegrees(
                   lambda2)));  // normalise to -180..+180°
}

Point Point::rhumbMidpointTo(const Point &point) const {
  // see mathforum.org/kb/message.jspa?messageID=148837

  auto phi1 = latitude().radians();
  auto lambda1 = longitude().radians();
  auto phi2 = point.latitude().radians();
  auto lambda2 = point.longitude().radians();

  if (std::abs(lambda2 - lambda1) > Coordinate::pi())
    lambda1 += 2 * Coordinate::pi();  // crossing anti-meridian

  auto phi3 = (phi1 + phi2) / 2;
  auto f1 = std::tan(Coordinate::pi() / 4.0 + phi1 / 2.0);
  auto f2 = std::tan(Coordinate::pi() / 4.0 + phi2 / 2.0);
  auto f3 = std::tan(Coordinate::pi() / 4.0 + phi3 / 2.0);
  auto lambda3 = ((lambda2 - lambda1) * std::log(f3) + lambda1 * std::log(f2) -
                  lambda2 * std::log(f1)) /
                 std::log(f2 / f1);

  if (!std::isfinite(lambda3)) {
    lambda3 = (lambda1 + lambda2) / 2.0;  // parallel of latitude
  }

  return Point(Coordinate::toDegrees(phi3),
               normalizeAngle(Coordinate::toDegrees(
                   lambda3)));  // normalise to -180..+180°
}

double Point::areaOf(const std::vector<Point> &polygon, double radius) {
  // Uses method due to Karney:
  // osgeo-org.1560.x6.nabble.com/Area-of-a-spherical-polygon-td3841625.html;
  // For each edge of the polygon, tan(E/2) = tan(deltaLambda/2)·(tan(phi1/2) +
  // tan(phi2/2)) / (1 + tan(phi1/2)·tan(phi2/2)) where E is the spherical
  // excess of the trapezium obtained by extending the edge to the equator

  if (polygon.size() < 3) {
    return 0.0;
  }

  auto tmpPolygon = polygon;

  // Close polygon so that last point equals first point
  bool closed = (tmpPolygon[0] == tmpPolygon[tmpPolygon.size() - 1]);
  if (!closed) {
    tmpPolygon.emplace_back(polygon[0]);
  }

  auto S = 0.0;  // spherical excess in steradians
  for (size_t v = 0; v < tmpPolygon.size() - 1; v++) {
    auto phi1 = tmpPolygon[v].latitude().radians();
    auto phi2 = tmpPolygon[v + 1].latitude().radians();
    auto deltaLambda = tmpPolygon[v + 1].longitude().radians() -
                       tmpPolygon[v].longitude().radians();
    auto E =
        2.0 * std::atan2(std::tan(deltaLambda / 2.0) *
                             (std::tan(phi1 / 2.0) + std::tan(phi2 / 2.0)),
                         1.0 + std::tan(phi1 / 2.0) * std::tan(phi2 / 2.0));
    S += E;
  }

  // Whether polygon encloses pole: sum of course deltas around pole is 0°
  // rather than normal ±360°:
  // blog.element84.com/determining-if-a-spherical-polygon-contains-a-pole.html
  // TODO(vahancho) any better test than this?
  auto sigmaDelta = 0.0;
  auto prevBrng = tmpPolygon[0].bearingTo(tmpPolygon[1]);
  for (size_t v = 0; v < tmpPolygon.size() - 1; v++) {
    auto initBrng = tmpPolygon[v].bearingTo(tmpPolygon[v + 1]);
    auto finalBrng = tmpPolygon[v].finalBearingTo(tmpPolygon[v + 1]);
    sigmaDelta += normalizeAngle(initBrng - prevBrng);
    sigmaDelta += normalizeAngle(finalBrng - initBrng);
    prevBrng = finalBrng;
  }

  auto initBrng = tmpPolygon[0].bearingTo(tmpPolygon[1]);
  sigmaDelta += normalizeAngle(initBrng - prevBrng);

  // TODO(vahancho) fix (intermittent) edge crossing pole - eg (85,90), (85,0),
  // (85,-90)
  if (std::abs(sigmaDelta) < 90.0) {  // 0°-ish
    S = std::abs(S) - 2.0 * Coordinate::pi();
  }

  return std::abs(S * radius * radius);  // area in units of radius
}

}  // namespace spherical
}  // namespace erkir
