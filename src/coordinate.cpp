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

#define _USE_MATH_DEFINES

#include "coordinate.h"

#include <cassert>
#include <codecvt>
#include <math.h>
#include <regex>
#include <stdexcept>
#include <unordered_map>

namespace erkir
{

using RxList = std::unordered_map<Coordinate::Format, std::wregex>;

constexpr double radiansInDegree = M_PI / 180.0;

static const std::wstring degreeSigns = LR"([\u00B0\u00BA])";
static const std::wstring minSigns    = LR"(['\u2019\u2032\u02B9])";
static const std::wstring secSigns    = LR"(["\u201D\u2033])";
static const std::wstring floatNumber = LR"([0-9]*[.,]?[0-9]*)";

static std::wstring dms(const std::wstring &directions)
{
  return { LR"(\s*([0-9]+)\s*)"
           + degreeSigns +
           LR"(\s*([0-5]?[0-9])\s*)" + minSigns + // Minutes [0, 60)
           LR"(\s*()"
           + floatNumber +
           LR"()\s*)"
           + secSigns +
           LR"(\s*()"
           + directions +
           LR"()?\s*|\s*([0-9]+)\s+([0-5]?[0-9]+)\s+()"
           + floatNumber +
           LR"()\s+()"
           + directions +
           LR"()?\s*)" };
}

static std::wstring ddm(const std::wstring &directions)
{
  return { LR"(\s*([0-9]+)\s*)"
           + degreeSigns +
           LR"(\s*()" + floatNumber + LR"()\s*)" + minSigns +
           LR"(\s*()"
           + directions +
           LR"()?\s*|\s*([0-9]+)\s+()" +
           floatNumber +
           LR"()\s+()"
           + directions +
           LR"()?\s*)" };
}

static std::wstring dd(const std::wstring &directions)
{
  return { LR"(\s*(-?)"
           + floatNumber +
           LR"()\s*)"
           + degreeSigns +
           LR"(?\s*()"
           + directions +
           LR"()?\s*)" };
}

static double parseRx(const std::string &coord, const RxList &rxList, char hemisphere)
{
  static std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
  auto wcoord = converter.from_bytes(coord.c_str());

  auto parseRx = [] (const std::wstring &ws, const std::wregex &rx) -> std::vector<std::wstring> {
    std::vector<std::wstring> ret;
    std::match_results<std::wstring::const_iterator> match;
    if (std::regex_match(ws, match, rx)) {
      for (std::size_t i = 1; i < match.size(); ++i) {
        const auto &sub_match = match[i];
        auto piece = sub_match.str();

        if (!piece.empty()) {
          ret.emplace_back(piece);
        }
      }
    }
    return ret;
  };

  auto toDouble = [] (const std::wstring &value, double max) {
    static constexpr const double epsilon = 0.001;
    const auto d = std::stof(value);
    if (d - max > epsilon) {
      throw std::runtime_error("Invalid or unsupported coordinates format");
    }

    return d;
  };

  double dd = 0.0;

  for (auto && rx : rxList) {
    auto v = parseRx(wcoord, rx.second);
    if (!v.empty()) {
      switch (rx.first) {
      case Coordinate::DD:
      {
        assert(v.size() >= 1);
        dd = std::stof(v.at(0));
        break;
      };
      case Coordinate::DDM:
      {
        assert(v.size() >= 2);
        auto d = std::stof(v.at(0));
        auto m = toDouble(v.at(1), 60.0);
        dd = d + m / 60.0;
        break;
      }
      case Coordinate::DMS:
      {
        assert(v.size() >= 3);
        auto d = std::stof(v.at(0));
        auto m = toDouble(v.at(1), 60.0);
        auto s = toDouble(v.at(2), 60.0);
        dd = d + m / 60.0 + s / 3600.0;
        break;
      }
      default:
        assert(false);
        break;
      }

      if (std::toupper(v.back().at(0)) == hemisphere && dd > 0.0) {
        dd = -dd;
      }

      return dd;
    }
  }

  throw std::runtime_error("Invalid or unsupported coordinates format");
}

Coordinate::Coordinate(double degrees)
  :
    m_degrees(degrees)
{}

double Coordinate::degrees() const
{
  return m_degrees;
}

double Coordinate::radians() const
{
  return toRadians(m_degrees);
}

double Coordinate::toDegrees(double radians)
{
  return radians / radiansInDegree;
}

double Coordinate::toRadians(double degrees)
{
  return degrees * radiansInDegree;
}

double Coordinate::pi()
{
  return M_PI;
}

double Coordinate::wrap360(double degrees)
{
  return fmod(degrees + 360.0, 360.0);
}

////////////////////////////////////////////////////////////////////////////////

Latitude::Latitude(double degree)
  :
    Coordinate(degree)
{
  if (degree > 90.0 || degree < -90.0)
  {
    throw std::out_of_range("Latitude measurements range from 0° to (+/–)90°.");
  }
}

Latitude Latitude::fromString(const std::string &coord)
{
  static const std::wstring directions = LR"([NSns])";
  static const std::wregex rxDMS{ dms(directions) };
  static const std::wregex rxDDM{ ddm(directions) };
  static const std::wregex rxDD { dd (directions) };

  static const RxList rxList =
  {
    { DMS, rxDMS },
    { DDM, rxDDM },
    { DD , rxDD  }
  };

  return { parseRx(coord, rxList, 'S')};
}

////////////////////////////////////////////////////////////////////////////////

Longitude::Longitude(double degree)
  :
    Coordinate(degree)
{
  if (degree > 180.0 || degree < -180.0)
  {
    throw std::out_of_range("Longitude measurements range from 0° to (+/–)180°.");
  }
}

Longitude Longitude::fromString(const std::string &coord)
{
  static const std::wstring directions = LR"([EWew])";
  static const std::wregex rxDMS{ dms(directions) };
  static const std::wregex rxDDM{ ddm(directions) };
  static const std::wregex rxDD { dd (directions) };

  static const RxList rxList =
  {
    { DMS, rxDMS },
    { DDM, rxDDM },
    { DD , rxDD  }
  };

  return { parseRx(coord, rxList, 'W')};
}

} // namespace erkir

