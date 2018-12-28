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

#include "point.h"
#include "point.cpp"
#include "coordinate.cpp"

using namespace erkir;

static int s_passed = 0;
static int s_failed = 0;

static std::string boolToString(bool value)
{
  return value ? "true" : "false";
}

static bool verifyDouble(double value, double expected, const char *file, int line)
{
  static const double epsilon = 0.001;
  if (std::abs(value - expected) > epsilon)
  {
    fprintf(stderr, "%s:%d VALUE ERROR: actual=%.3f, expected=%.3f\n", file, line, value, expected);
    ++s_failed;
    return false;
  }
  ++s_passed;
  return true;
}

static bool verifyPoint(const Point &point, const Point &expectedPoint, const char *file, int line)
{
  if (point.isValid() != expectedPoint.isValid())
  {
    fprintf(stderr, "%s:%d POINT ERROR: isValid: %s, but expected: %s\n",
            file, line,
            boolToString(point.isValid()).c_str(), boolToString(expectedPoint.isValid()).c_str());
    ++s_failed;
    return false;
  }

  if (point != expectedPoint)
  {
    fprintf(stderr, "%s:%d POINT ERROR: actual=(%.3f, %.3f), expected=(%.3f, %.3f)\n",
            file, line,
            point.latitude().degrees(), point.longitude().degrees(),
            expectedPoint.latitude().degrees(), expectedPoint.longitude().degrees());
    ++s_failed;
    return false;
  }
  ++s_passed;
  return true;
}

int main()
{
  {
    Point p1{ 52.205, 0.119 };
    Point p2{ 48.857, 2.351 };
    verifyDouble(404279.164, p1.distanceTo(p2), __FILE__, __LINE__);  // 404.3 km
    verifyDouble(156.167, p1.bearingTo(p2), __FILE__, __LINE__);      // 156.2°
    verifyDouble(157.890, p1.finalBearingTo(p2), __FILE__, __LINE__); // 157.9°
  }
  {
    Point p1{ 52.205, 0.119 };
    Point p2{ 48.857, 2.351 };
    auto pMid1 = p1.midpointTo(p2); // 50.5363°N, 001.2746°E
    verifyPoint(pMid1, { 50.5363, 1.2746 }, __FILE__, __LINE__);
  }
  {
    Point p1{ 52.205, 0.119 };
    Point p2{ 48.857, 2.351 };
    auto pMid = p1.intermediatePointTo(p2, 0.25); // 51.3721°N, 000.7073°E
    verifyPoint(pMid, { 51.3721, 0.7073 }, __FILE__, __LINE__);
  }
  {
    Point p3{ 51.4778, -0.0015 };
    auto dest = p3.destinationPoint(7794.0, 300.7); // 51.5135°N, 000.0983°W
    verifyPoint(dest, { 51.5135, -0.0983 /*W*/}, __FILE__, __LINE__);
  }
  {
    auto intersect = Point::intersection({ 51.8853, 0.2545 }, 108.547,
                                         { 49.0034, 2.5735 }, 32.435); // 50.9078°N, 004.5084°
    verifyPoint(intersect, { 50.9078, 4.5084 }, __FILE__, __LINE__);
  }

  {
    Point p{ 53.2611, -0.7972 };
    auto dist = p.crossTrackDistanceTo({ 53.3206, -1.7297 },
                                       { 53.1887,  0.1334 }); // -307.5 m
    verifyDouble(-307.550, dist, __FILE__, __LINE__);
  }
  {
    Point p{ 53.2611, -0.7972 };
    auto dist = p.alongTrackDistanceTo({ 53.3206, -1.7297 },
                                       { 53.1887,  0.1334 }); // 62.331 km
    verifyDouble(62331.493, dist, __FILE__, __LINE__);
  }
  {
    Point p{ 53.2611, -0.7972 };
    auto dist = p.alongTrackDistanceTo({ 53.2611, -0.7972 },
                                       { 53.1887,  0.1334 }); // 0.0 km
    verifyDouble(0.0, dist, __FILE__, __LINE__);
  }

  // Rhumb functions.
  {
    Point p1{ 51.127, 1.338 };
    Point p2{ 50.964, 1.853 };
    verifyDouble(40307.745, p1.rhumbDistanceTo(p2), __FILE__, __LINE__); // 40.31 km
  }
  {
    Point p1{ 51.127, 1.338 };
    Point p2{ 50.964, 1.853 };
    verifyDouble(116.722, p1.rhumbBearingTo(p2), __FILE__, __LINE__); // 116.7°
  }
  {
    Point p1{ 51.127, 1.338 };
    auto p2 = p1.rhumbDestinationPoint(40300, 116.7); // 50.9642°N, 001.8530°E
    verifyPoint(p2, { 50.9642, 001.8530 }, __FILE__, __LINE__);
  }
  {
    Point p1{ 51.127, 1.338 };
    Point p2{ 50.964, 1.853 };
    auto pMid = p1.rhumbMidpointTo(p2); // 51.0455°N, 001.5957°E
    verifyPoint(pMid, { 51.0455, 001.5957 }, __FILE__, __LINE__);
  }

  // Area
  {
    std::vector<Point> polygon = { {0, 0}, {1, 0}, {0, 1} };
    verifyDouble(6182469722.731, Point::areaOf(polygon), __FILE__, __LINE__); // 6.18e9 m²
  }

  printf("Total tests: %d, passed: %d, failed: %d\n", s_passed + s_failed, s_passed, s_failed);

  return s_failed == 0 ? 0 : 1;
}
