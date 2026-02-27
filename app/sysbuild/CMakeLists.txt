# Set BOARD_ROOT so sysbuild can find custom boards in app/boards/
# This must happen before find_package(Sysbuild) which triggers board discovery.
list(APPEND BOARD_ROOT ${APP_DIR})
set(BOARD_ROOT ${BOARD_ROOT} CACHE PATH "BOARD_ROOT with ZMK app boards" FORCE)

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sysbuild LANGUAGES)
