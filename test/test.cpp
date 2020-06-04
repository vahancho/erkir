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

#include <random>
#include <chrono>
#include <iostream>

#include "sphericalpoint.h"
#include "sphericalpoint.cpp"
#include "ellipsoidalpoint.h"
#include "ellipsoidalpoint.cpp"
#include "point.cpp"
#include "coordinate.cpp"
#include "vector3d.cpp"
#include "datum.cpp"
#include "cartesianpoint.cpp"

using namespace erkir;

static int s_passed = 0;
static int s_failed = 0;

#define STRINGIFY_HELPER(x) #x
#define STRINGIFY(x) STRINGIFY_HELPER(x)

#define LOCATION std::string(__FILE__ ":" STRINGIFY(__LINE__))

static std::string boolToString(bool value)
{
  return value ? "true" : "false";
}

static bool verifyDouble(double value, double expected, const std::string &location)
{
  static const double epsilon = 0.001;
  if (std::abs(value - expected) > epsilon) {
    fprintf(stderr, "%s VALUE ERROR: actual=%.3f, expected=%.3f\n",
            location.c_str(), value, expected);
    ++s_failed;
    return false;
  }
  ++s_passed;
  return true;
}

static bool verifyPoint(const Point &point, const Point &expectedPoint,
                        const std::string &location)
{
  if (point.isValid() != expectedPoint.isValid()) {
    fprintf(stderr, "%s POINT ERROR: isValid: %s, but expected: %s\n",
            location.c_str(),
            boolToString(point.isValid()).c_str(), boolToString(expectedPoint.isValid()).c_str());
    ++s_failed;
    return false;
  }

  if (point != expectedPoint) {
    fprintf(stderr, "%s POINT ERROR: actual=(%.3f, %.3f), expected=(%.3f, %.3f)\n",
            location.c_str(),
            point.latitude().degrees(), point.longitude().degrees(),
            expectedPoint.latitude().degrees(), expectedPoint.longitude().degrees());
    ++s_failed;
    return false;
  }
  ++s_passed;
  return true;
}

static std::vector<spherical::Point> randomPoints(int count)
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> latitudeDistr{ -90.0, 90.0 };
  std::uniform_real_distribution<double> longitudeDistr{ -180.0, 180.0 };

  std::vector<spherical::Point> points;
  for (int i = 0; i < count; ++i) {
    points.emplace_back(latitudeDistr(gen), longitudeDistr(gen));
  }

  return points;
}

static void reportTime(std::chrono::time_point<std::chrono::high_resolution_clock> start,
                       std::chrono::time_point<std::chrono::high_resolution_clock> end,
                       int count, const std::string &title)
{
  auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
  std::cout << title << ": " << duration / count << " nanoseconds per calculation\n";
}

int main()
{
  {
    Point p{};
    p.isValid() ? ++s_failed : ++s_passed;
    verifyDouble(0.0, p.latitude().degrees(), LOCATION);
    verifyDouble(0.0, p.longitude().degrees(), LOCATION);
  }
  {
    spherical::Point p{};
    p.isValid() ? ++s_failed : ++s_passed;
    verifyDouble(0.0, p.latitude().degrees(), LOCATION);
    verifyDouble(0.0, p.longitude().degrees(), LOCATION);
  }
  {
    spherical::Point p1{ 52.205, 0.119 };
    spherical::Point p2{ 48.857, 2.351 };
    verifyDouble(404279.164, p1.distanceTo(p2), LOCATION);  // 404.3 km
    verifyDouble(156.167, p1.bearingTo(p2), LOCATION);      // 156.2°
    verifyDouble(157.890, p1.finalBearingTo(p2), LOCATION); // 157.9°
  }
  {
    spherical::Point p1{ 52.205, 0.119 };
    spherical::Point p2{ 48.857, 2.351 };
    auto pMid1 = p1.midpointTo(p2); // 50.5363°N, 001.2746°E
    verifyPoint(pMid1, { 50.5363, 1.2746 }, LOCATION);
  }
  {
    spherical::Point p1{ 52.205, 0.119 };
    spherical::Point p2{ 48.857, 2.351 };
    auto pMid = p1.intermediatePointTo(p2, 0.25); // 51.3721°N, 000.7073°E
    verifyPoint(pMid, { 51.3721, 0.7073 }, LOCATION);
  }
  {
    spherical::Point p3{ 51.4778, -0.0015 };
    auto dest = p3.destinationPoint(7794.0, 300.7); // 51.5135°N, 000.0983°W
    verifyPoint(dest, { 51.5135, -0.0983 /*W*/}, LOCATION);
  }
  {
    auto intersect = spherical::Point::intersection({ 51.8853, 0.2545 }, 108.547,
                                         { 49.0034, 2.5735 }, 32.435); // 50.9078°N, 004.5084°
    verifyPoint(intersect, { 50.9078, 4.5084 }, LOCATION);
  }

  {
    spherical::Point p{ 53.2611, -0.7972 };
    auto dist = p.crossTrackDistanceTo({ 53.3206, -1.7297 },
                                       { 53.1887,  0.1334 }); // -307.5 m
    verifyDouble(-307.550, dist, LOCATION);
  }
  {
    spherical::Point p{ 53.2611, -0.7972 };
    auto dist = p.alongTrackDistanceTo({ 53.3206, -1.7297 },
                                       { 53.1887,  0.1334 }); // 62.331 km
    verifyDouble(62331.493, dist, LOCATION);
  }
  {
    spherical::Point p{ 53.2611, -0.7972 };
    auto dist = p.alongTrackDistanceTo({ 53.2611, -0.7972 },
                                       { 53.1887,  0.1334 }); // 0.0 km
    verifyDouble(0.0, dist, LOCATION);
  }

  // Rhumb functions.
  {
    spherical::Point p1{ 51.127, 1.338 };
    spherical::Point p2{ 50.964, 1.853 };
    verifyDouble(40307.745, p1.rhumbDistanceTo(p2), LOCATION); // 40.31 km
  }
  {
    spherical::Point p1{ 51.127, 1.338 };
    spherical::Point p2{ 50.964, 1.853 };
    verifyDouble(116.722, p1.rhumbBearingTo(p2), LOCATION); // 116.7°
  }
  {
    spherical::Point p1{ 51.127, 1.338 };
    auto p2 = p1.rhumbDestinationPoint(40300, 116.7); // 50.9642°N, 001.8530°E
    verifyPoint(p2, { 50.9642, 001.8530 }, LOCATION);
  }
  {
    spherical::Point p1{ 51.127, 1.338 };
    spherical::Point p2{ 50.964, 1.853 };
    auto pMid = p1.rhumbMidpointTo(p2); // 51.0455°N, 001.5957°E
    verifyPoint(pMid, { 51.0455, 001.5957 }, LOCATION);
  }

  // Area
  {
    std::vector<spherical::Point> polygon = { {0, 0}, {1, 0}, {0, 1} };
    verifyDouble(6182469722.731, spherical::Point::areaOf(polygon), LOCATION); // 6.18e9 mІ
  }

  //////////////////////////////////////////////////////////////////////////////

  // Datum conversion.
  {
    ellipsoidal::Point greenwichWGS84( 51.47788, -0.00147, 0.0, ellipsoidal::Datum::Type::WGS84 );
    auto greenwichOSGB36 = greenwichWGS84.toDatum( ellipsoidal::Datum::Type::OSGB36 ); // 51.4773°N, 000.0000°E
    // TODO: huh? should be 0°E? out by c. 10 metres / 0.5"! am I missing something?
    // TODO: This result corresponds to one Chris Veness gets in his tests.
    verifyPoint( greenwichOSGB36, { 51.4773, 0.0001 }, LOCATION );

    // Should return to the same coordinates.
    greenwichWGS84 = greenwichOSGB36.toDatum( ellipsoidal::Datum::Type::WGS84 );
    verifyPoint( greenwichWGS84, { 51.478, 0.0001 }, LOCATION );
  }
  {
    ellipsoidal::Point pWGS84( 53.0, 1.0, 50.0 );
    auto pOSGB36 = pWGS84.toDatum( ellipsoidal::Datum::Type::OSGB36 );
    verifyPoint( pOSGB36, { 52.9996, 1.0018 }, LOCATION );
    verifyDouble( pOSGB36.height(), 3.987, LOCATION );
  }
  {
    ellipsoidal::Point pWGS84( 53.0, 1.0, 50.0 );
    auto pED50 = pWGS84.toDatum(ellipsoidal::Datum::Type::ED50);
    verifyPoint(pED50, {53.0008, 1.0014}, LOCATION);
    verifyDouble(pED50.height(), 2.721, LOCATION);
  }

  // Vector3D
  {
    Vector3d v{};
    v.isValid() ? ++s_failed : ++s_passed;
  }
  {
    Vector3d v{3.0, 4.0, 0.0};
    verifyDouble(v.length(), 5.0, LOCATION);
  }
  {
    Vector3d v{ 0.0, 0.0, 0.0 };
    verifyDouble(v.length(), 0.0, LOCATION);
  }
  {
    Vector3d v1{ 0.0, 0.0, 0.0 };
    Vector3d v2{ 1.0, 2.0, 3.0 };
    verifyDouble(v1.dot(v2), 0.0, LOCATION);
  }
  {
    Vector3d v1{ 2.0, 2.0, 2.0 };
    Vector3d v2{ 2.0, 2.0, 2.0 };
    verifyDouble(v1.dot(v2), 12.0, LOCATION);
  }
  {
    // Orthogonal vectors.
    Vector3d v1{ 1.0, 0.0, 0.0 };
    Vector3d v2{ 0.0, 1.0, 0.0 };
    auto v3 = v1.cross(v2);
    verifyDouble(v3.x(), 0.0, LOCATION);
    verifyDouble(v3.y(), 0.0, LOCATION);
    verifyDouble(v3.z(), 1.0, LOCATION);
  }
  {
    Vector3d v1{ 3.0, 0.0, 0.0 };
    Vector3d v2{ 0.0, 4.0, 0.0 };
    auto v3 = v1.cross(v2);
    verifyDouble(v3.x(), 0.0, LOCATION);
    verifyDouble(v3.y(), 0.0, LOCATION);
    verifyDouble(v3.z(), 12.0, LOCATION);
  }
  {
    Vector3d v1{ 1.0, 2.0, 3.0 };
    Vector3d v2{ 4.0, 5.0, 6.0 };
    auto v3 = v1.cross(v2);
    verifyDouble(v3.x(), -3.0, LOCATION);
    verifyDouble(v3.y(), 6.0, LOCATION);
    verifyDouble(v3.z(), -3.0, LOCATION);
  }

  // Cartesian points
  {
    auto p = ellipsoidal::Point(45.0, 45.0);
    auto cartesian = p.toCartesianPoint();
    verifyDouble(cartesian->x(), 3194419.145, LOCATION);
    verifyDouble(cartesian->y(), 3194419.145, LOCATION);
    verifyDouble(cartesian->z(), 4487348.409, LOCATION);
  }
  // Cartesian point to geodetic point.
  {
    auto cartesian = cartesian::Point(3194419.0, 3194419.0, 4487348.0);
    auto p = cartesian.toGeoPoint();
    verifyPoint(*p, {45.0, 45.0}, LOCATION);
  }

  //////////////////////////////////////////////////////////////////////////////

  std::cout << "Total tests: " << s_passed + s_failed << ", passed: " << s_passed << ", failed: " << s_failed << '\n';

  //////////////////////////////////////////////////////////////////////////////

  // Some performance tests.
  std::cout << "\nSome performance tests:\n\n";

  const int count = 5000;
  auto points1 = randomPoints(count);
  auto points2 = randomPoints(count);

  // Distance (haversine).
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    points1[i].distanceTo(points2[i]);
  }
  auto end = std::chrono::high_resolution_clock::now();
  reportTime(start, end, count, "Distance");

  // Initial bearing.
  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    points1[i].bearingTo(points2[i]);
  }
  end = std::chrono::high_resolution_clock::now();
  reportTime(start, end, count, "Initial bearing");

  // Destination point.
  start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < count; ++i) {
    points1[i].destinationPoint(7794.0, 300.7);
  }
  end = std::chrono::high_resolution_clock::now();
  reportTime(start, end, count, "Destination point");

  return s_failed == 0 ? 0 : 1;
}
