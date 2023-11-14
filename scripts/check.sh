#!/usr/bin/env sh

# Fail on error
set -o errexit
# Disable wildcard character expansion
set -o noglob
# Disable uninitialized variable usage
set -o nounset
# Disable error masking in pipe
# shellcheck disable=SC3040
if (set -o pipefail > /dev/null 2>&1); then
    set -o pipefail
fi

# ================
# LOGGER
# ================
# Error log level. Cause exit failure
LOG_LEVEL_ERROR=100
# Warning log level
LOG_LEVEL_WARN=200
# Informational log level
LOG_LEVEL_INFO=300
# Debug log level
LOG_LEVEL_DEBUG=400
# Log level
LOG_LEVEL=$LOG_LEVEL_INFO
# Log color flag
LOG_COLOR_ENABLE=true

# Convert log level to equivalent name
# @param $1 Log level
log_level_name() {
    _log_level=${1:-LOG_LEVEL}
    _log_level_name=

    case $_log_level in
        "$LOG_LEVEL_ERROR") _log_level_name=error ;;
        "$LOG_LEVEL_WARN") _log_level_name=warn ;;
        "$LOG_LEVEL_INFO") _log_level_name=info ;;
        "$LOG_LEVEL_DEBUG") _log_level_name=debug ;;
        *) ERROR "Unknown log level '$_log_level'" ;;
    esac

    printf '%s\n' "$_log_level_name"
}

# Check if log level is enabled
# @param $1 Log level
log_is_enabled() {
    [ "$1" -le "$LOG_LEVEL" ]
}

# Print log message
# @param $1 Log level
# @param $2 Message
_log_print_message() {
    _log_level=${1:-LOG_LEVEL_ERROR}
    shift
    _log_level_name=
    _log_message=${*:-}
    _log_prefix=
    _log_suffix="\033[0m"

    # Check log level
    log_is_enabled "$_log_level" || return 0

    case $_log_level in
        "$LOG_LEVEL_ERROR")
            _log_level_name=ERROR
            _log_prefix="\033[1;31m"
            ;;
        "$LOG_LEVEL_WARN")
            _log_level_name=WARN
            _log_prefix="\033[1;33m"
            ;;
        "$LOG_LEVEL_INFO")
            _log_level_name=INFO
            _log_prefix="\033[1;37m"
            ;;
        "$LOG_LEVEL_DEBUG")
            _log_level_name=DEBUG
            _log_prefix="\033[1;34m"
            ;;
    esac

    # Check color flag
    if [ "$LOG_COLOR_ENABLE" = false ]; then
        _log_prefix=
        _log_suffix=
    fi

    # Log
    printf '%b[%-5s] %b%b\n' "$_log_prefix" "$_log_level_name" "$_log_message" "$_log_suffix"
}

# Error log message
# @param $1 Message
ERROR() {
    _log_print_message "$LOG_LEVEL_ERROR" "$1" >&2
    exit 1
}
# Warning log message
# @param $1 Message
WARN() { _log_print_message "$LOG_LEVEL_WARN" "$1" >&2; }
# Informational log message
# @param $1 Message
INFO() { _log_print_message "$LOG_LEVEL_INFO" "$1"; }
# Debug log message
# @param $1 Message
DEBUG() { _log_print_message "$LOG_LEVEL_DEBUG" "$1"; }

# ================
# FUNCTIONS
# ================
# Assert command is installed
# @param $1 Command name
assert_cmd() {
    check_cmd "$1" || ERROR "Command '$1' not found"
    DEBUG "Command '$1' found at '$(command -v "$1")'"
}

# Check command is installed
# @param $1 Command name
check_cmd() {
    command -v "$1" > /dev/null 2>&1
}

# Show help message
show_help() {
    cat << EOF
Usage: $(basename "$0")
         [--disable-color] [--help] [--log-level <LEVEL>] [--root-dir <DIR>]

DAG checker.

Options:
  --disable-color        Disable color

  --help                 Show this help message and exit

  --log-level <LEVEL>    Logger level
                         Default: $(log_level_name "$LOG_LEVEL")
                         Values:
                           error    Error level
                           warn     Warning level
                           info     Informational level
                           debug    Debug level

  --root-dir <DIR>       Root directory
                         Default: $ROOTDIR
EOF
}

# Assert argument has a value
# @param $1 Argument name
# @param $2 Argument value
_parse_args_assert_value() {
    [ -n "${2+x}" ] || ERROR "Argument '$1' requires a non-empty value"
}

# Parse command line arguments
# @param $@ Arguments
parse_args() {
    while [ $# -gt 0 ]; do
        case $1 in
            --disable-color)
                # Disable color
                LOG_COLOR_ENABLE=false
                shift
                ;;
            --help)
                # Display help message and exit
                show_help
                exit 0
                ;;
            --log-level)
                # Log level
                _parse_args_assert_value "$@"
                case $2 in
                    error) LOG_LEVEL=$LOG_LEVEL_ERROR ;;
                    warn) LOG_LEVEL=$LOG_LEVEL_WARN ;;
                    info) LOG_LEVEL=$LOG_LEVEL_INFO ;;
                    debug) LOG_LEVEL=$LOG_LEVEL_DEBUG ;;
                    *) ERROR "Value '$2' of argument '$1' is invalid" ;;
                esac
                shift
                shift
                ;;
            --root-dir)
                # Root directory
                _parse_args_assert_value "$@"
                ROOTDIR=$2
                shift
                shift
                ;;
            -*)
                # Unknown argument
                WARN "Unknown argument '$1' is ignored"
                shift
                ;;
            *)
                # No argument
                WARN "Skipping argument '$1'"
                shift
                ;;
        esac
    done
}

# Verify system
verify_system() {
    assert_cmd clang-format
    assert_cmd cmake-format
    assert_cmd cmake-lint

    [ -d "$ROOTDIR" ] || ERROR "Root directory '$ROOTDIR' does not exists"
}

# CMake Format
cmake_format() {
    INFO "CMake Format"
    find "$ROOTDIR" \
        -not -path "$ROOTDIR/build/*" \
        -not -path "$ROOTDIR/cmake/erkirConfig.cmake.in" \
        -not -path "$ROOTDIR/cmake/CPM.cmake" \
        -not -path "$ROOTDIR/cmake/CodeCoverage.cmake" \
        -type f \( -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.cmake.in' \) \
        -print0 \
        | xargs -0 -n 1 \
            cmake-format \
            --check
}

# CMake Lint
cmake_lint() {
    INFO "CMake Lint"
    find "$ROOTDIR" \
        -not -path "$ROOTDIR/build/*" \
        -not -path "$ROOTDIR/cmake/erkirConfig.cmake.in" \
        -not -path "$ROOTDIR/cmake/CPM.cmake" \
        -not -path "$ROOTDIR/cmake/CodeCoverage.cmake" \
        -type f \( -name 'CMakeLists.txt' -o -name '*.cmake' -o -name '*.cmake.in' \) \
        -print0 \
        | xargs -0 -n 1 \
            cmake-lint
}

# Clang Format
clang_format() {
    INFO "Clang Format"
    find "$ROOTDIR" \
        -not -path "$ROOTDIR/build/*" \
        -not -path "$ROOTDIR/include/erkir/export.h" \
        -type f \( -name '*.h' -o -name '*.h.in' -o -name '*.cpp' \) \
        -print0 \
        | xargs -0 -n 1 \
            clang-format \
            --Werror \
            --dry-run \
            --style 'file'
}

# Clang Tidy
clang_tidy() {
    INFO "Clang Tidy"
    # TODO(carlocorradini)
}

# CppLint
cpplint() {
    INFO "CppLint"
    # TODO(carlocorradini)
}

# CppCheck
cppcheck() {
    INFO "CppCheck"
    # TODO(carlocorradini)
}

# ================
# CONFIGURATION
# ================
# Log level
LOG_LEVEL=$LOG_LEVEL_INFO
# Log color flag
LOG_COLOR_ENABLE=true
# Root directory
# shellcheck disable=SC1007
ROOTDIR="$(readlink -f "$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)/..")"

# ================
# MAIN
# ================
{
    parse_args "$@"
    verify_system
    cmake_format
    cmake_lint
    clang_format
    clang_tidy
    cpplint
    cppcheck
}
