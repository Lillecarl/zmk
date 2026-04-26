# Add app/ to BOARD_ROOT for sysbuild orchestration (needed so SHIELD=daisy
# resolves during validation). sysbuild_root.cmake will FORCE-cache this so it
# would otherwise propagate to every image, including mcuboot. We strip it back
# out for non-main images via a sysbuild_cache override below — the main app
# build adds app/ to BOARD_ROOT separately in app/CMakeLists.txt.
list(APPEND BOARD_ROOT ${APP_DIR})

# Top-level -DSHIELD=daisy must apply to the zmk app only, not mcuboot.
set(mcuboot_SHIELD "" CACHE INTERNAL "" FORCE)

# Split sysbuild loading so we can override sysbuild_cache between the function
# being defined (sysbuild_extensions) and being called for each image
# (sysbuild_images). The list mirrors zephyr/share/sysbuild/cmake/modules/sysbuild_default.cmake.
find_package(Sysbuild REQUIRED COMPONENTS
  extensions
  sysbuild_extensions
  python
  west
  yaml
  sysbuild_root
  zephyr_module
  boards
  shields
  hwm_v2
  sysbuild_snippets
  sysbuild_kconfig
  native_simulator_sb_extensions
  HINTS $ENV{ZEPHYR_BASE}
)

# Override sysbuild_cache(CREATE) so non-main images (mcuboot) don't get
# BOARD_ROOT=app/ written into their per-image cache file. Without this,
# mcuboot's Kconfig parses every Kconfig.defconfig under app/boards/shields/
# (which references ZMK-only symbols) and aborts on "defined without a type"
# warnings. CMake renames the original to _sysbuild_cache when we redefine it.
function(sysbuild_cache)
  cmake_parse_arguments(SB_CACHE "CREATE;CMAKE_RERUN" "APPLICATION" "" ${ARGN})
  get_target_property(_main_app ${SB_CACHE_APPLICATION} MAIN_APP)
  if(NOT _main_app)
    set(_saved_board_root "$CACHE{BOARD_ROOT}")
    unset(BOARD_ROOT CACHE)
  endif()
  _sysbuild_cache(${ARGV})
  if(NOT _main_app)
    set(BOARD_ROOT "${_saved_board_root}" CACHE PATH "Sysbuild adjusted BOARD_ROOT" FORCE)
  endif()
endfunction()

# Now do the rest of what sysbuild_default would do.
find_package(Sysbuild REQUIRED COMPONENTS
  sysbuild_images
  sysbuild_merged_hex
  HINTS $ENV{ZEPHYR_BASE}
)

project(sysbuild LANGUAGES)
