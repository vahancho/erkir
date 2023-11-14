# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

### Added

-   Changelog file ([`CHANGELOG.md`](./CHANGELOG.md))

### Changed

-   All includes must be preceded by the word _erkir_: `#include <erkir/FILE.h>`
-   Single include header file, `erkir.h`, exports all library's component
-   Tests implemented with [`GoogleTest`](https://github.com/google/googletest)
-   _CI_ enhancements
-   Overall code and repository structure have been reworked to improve maintainability

### Deprecated

### Removed

### Fixed

### Security

## 2.0.0 - 2022-09-21

### Added

-   Support shared library (_Windows_)

### Changed

-   Refactors the project's build system using [`CMake`](https://cmake.org)
-   Use _GitHub Actions_ for _CI_
-   Improve documentation

## 1.0.0 - 2021-03-10

### Added

-   Initial release
-   Class description
