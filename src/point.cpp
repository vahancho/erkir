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

#include "point.h"
#include <cmath>

namespace erkir
{

Point::Point()
  :
    m_latitude(0.0),
    m_longitude(0.0),
    m_isValid(false)
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

bool Point::operator==(const Point &other) const
{
  static const double epsilon = 0.0001;
  return std::abs(latitude().degrees() - other.latitude().degrees()) < epsilon &&
         std::abs(longitude().degrees() - other.longitude().degrees()) < epsilon;
}

bool Point::operator!=(const Point &other) const
{
  return !(*this == other);
}

} // erkir

