# A C++ library for geodesic and trigonometric calculations

[Erkir (armenian: Երկիր)](https://github.com/vahancho/erkir) - is inspired by and based on the great work of [Chris Veness](https://github.com/chrisveness), the owner of the [Geodesy functions](https://github.com/chrisveness/geodesy) project - provides a set of comprehensive API for geodesic and trigonometric calculations. I would call it a C++ port of JavaScript functions provided by the mentioned Chris Veness' project, however I designed the library to be more object oriented. Thus the code is organized a little bit differently, but the implementation itself is preserved.

[![Build Status](https://travis-ci.org/vahancho/erkir.svg?branch=master)](https://travis-ci.org/vahancho/erkir)
[![Build status](https://ci.appveyor.com/api/projects/status/gh9v3ynrm1dt1w7t/branch/master?svg=true)](https://ci.appveyor.com/project/vahancho/erkir/branch/master)
[![Coverage Status](https://coveralls.io/repos/github/vahancho/erkir/badge.svg)](https://coveralls.io/github/vahancho/erkir)

### Prerequisites

There are no special requirements and dependencies except *C++11* compliant compiler. The class is tested with *gcc 4.8.4* and *MSVC 15.x* (Visual Studio 2017). The library is written with pure STL without any third party dependencies.
For more details see the CI badges (*Travis CI & AppVeyor CI*) above.

### Installation

No installation required. Just incorparate header files from the include/ and source files from src/ directories in your project and compile them. All library calsses are in *erkir* namespace.

### Usage Examples:

```cpp
#include "point.h"

int main(int argc, char **argv)
{
  // Calculate great-circle distance between two points.
  Point p1{ 52.205, 0.119 };
  Point p2{ 48.857, 2.351 };
  auto d = p1.distanceTo(p2); // 404.3 km
  
  // Get destination point by given distance (shortest) and bearing from start point.
  Point p3{ 51.4778, -0.0015 };
  auto dest = p3.destinationPoint(7794.0, 300.7); // 51.5135°N, 000.0983°W
  
  return 0;
}
```

## See Also

* [Movable Type Scripts Latitude/Longitude Calculations Reference](http://www.movable-type.co.uk/scripts/latlong.html)

