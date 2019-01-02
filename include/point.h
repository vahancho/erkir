/**********************************************************************************
*  MIT License                                                                    *
*                                                                                 *
*  Copyright (c) 2018-2019 Vahan Aghajanyan <vahancho@gmail.com>                  *
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

#ifndef POINT_H
#define POINT_H

#include <vector>
#include "coordinate.h"

namespace erkir
{

/// Base class for all types of geodetic points.
class Point
{
public:
  //! Constructs an invalid point object.
  Point();

  //! Constructs a point with the given \p latitude and \p longitude.
  Point(const Latitude &latitude, const Longitude &longitude);

  //! Returns the latitude of this point.
  const Latitude &latitude() const;

  //! Returns the longitude of this point.
  const Longitude &longitude() const;

  //! Returns true if this point is a valid one and false otherwise.
  bool isValid() const;

  /// Returns true if two points are equal and false otherwise.
  bool operator==(const Point &other) const;

  /// Returns true if two points are different and false otherwise.
  bool operator!=(const Point &other) const;

private:
  Latitude m_latitude;
  Longitude m_longitude;
  bool m_isValid;
};

} // erkir

#endif // POINT_H

