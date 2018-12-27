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

using namespace geodesy;

int main()
{
  // Tests:
  Point p1{ 52.205, 0.119 };
  Point p2{ 48.857, 2.351 };
  auto d = p1.sphericalDistanceTo(p2); // 404.3 km

  auto b1 = p1.sphericalBearingTo(p2);      // 156.2°
  auto b2 = p1.sphericalFinalBearingTo(p2); // 157.9°
  auto pMid1 = p1.sphericalMidpointTo(p2);   // 50.5363°N, 001.2746°E
  auto pMid2 = p1.sphericalIntermediatePointTo(p2, 0.25); // 51.3721°N, 000.7073°E

  Point p3{ 51.4778, -0.0015 };
  auto dest = p3.sphericalDestinationPoint(7794.0, 300.7); // 51.5135°N, 000.0983°W

  return 0;
}
