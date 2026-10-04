# Probe the actual compiler target, including cross/universal configurations.
set(CP_AVX2_CONTACT_SOLVER 0)
if(MUNK2D_ENABLE_AVX2_CONTACT_SOLVER)
  include(CheckCSourceCompiles)
  set(_cp_saved_required_flags "${CMAKE_REQUIRED_FLAGS}")
  if(MSVC)
    set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} /arch:AVX2")
  else()
    set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -mavx2")
  endif()
  check_c_source_compiles("
#if !defined(__x86_64__) && !defined(_M_X64)
#error This backend requires an x86-64 target
#endif
#include <immintrin.h>
int main(void) { __m256d v = _mm256_setzero_pd(); return _mm256_movemask_pd(v); }
" CP_COMPILER_HAS_AVX2_CONTACT_SOLVER)
  set(CMAKE_REQUIRED_FLAGS "${_cp_saved_required_flags}")
  if(CP_COMPILER_HAS_AVX2_CONTACT_SOLVER)
    set(CP_AVX2_CONTACT_SOLVER 1)
    if(MSVC)
      set_source_files_properties(cpContactSolverAvx2.c PROPERTIES COMPILE_OPTIONS "/arch:AVX2;/fp:strict;/GL-")
      set_source_files_properties(cpContactSolverScalar.c PROPERTIES COMPILE_OPTIONS "/fp:strict")
    else()
      # Do not let IPO move AVX2 code across the generic dispatch boundary.
      set_source_files_properties(cpContactSolverAvx2.c PROPERTIES COMPILE_OPTIONS "-mavx2;-ffp-contract=off;-fno-lto")
      set_source_files_properties(cpContactSolverScalar.c PROPERTIES COMPILE_OPTIONS "-ffp-contract=off")
    endif()
  endif()
endif()
message(STATUS "Runtime-guarded AVX2 contact solver compiled: ${CP_AVX2_CONTACT_SOLVER}")

file(GLOB chipmunk_source_files "*.c")
file(GLOB chipmunk_public_header "${chipmunk_SOURCE_DIR}/include/chipmunk/*.h")

include_directories(${chipmunk_SOURCE_DIR}/include)

message("Configuring Munk2D version ${MUNK2D_VERSION}")


if(BUILD_SHARED)
  add_library(chipmunk SHARED
    ${chipmunk_source_files}
  )
  target_compile_definitions(chipmunk PRIVATE CP_AVX2_CONTACT_SOLVER=${CP_AVX2_CONTACT_SOLVER})
  # Tell MSVC to compile the code as C++.
  if(MSVC)
    set_source_files_properties(${chipmunk_source_files} PROPERTIES LANGUAGE CXX)
    set_target_properties(chipmunk PROPERTIES LINKER_LANGUAGE CXX)
  endif(MSVC)
  # set the lib's version number
  # But avoid on Android because symlinks to version numbered .so's don't work with Android's Java-side loadLibrary.
  if(NOT ANDROID)
    set_target_properties(chipmunk PROPERTIES
      SOVERSION ${MUNK2D_VERSION_MAJOR}
      VERSION ${MUNK2D_VERSION})
  endif(NOT ANDROID)
  if(ANDROID OR UNIX)
	  # need to explicitly link to the math library because the CMake/Android toolchains may not do it automatically
	  target_link_libraries(chipmunk m)
  endif(ANDROID OR UNIX)
  install(TARGETS chipmunk RUNTIME DESTINATION ${BIN_INSTALL_DIR}
                           LIBRARY DESTINATION ${LIB_INSTALL_DIR}
                           ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
endif(BUILD_SHARED)

if(BUILD_STATIC)
  add_library(chipmunk_static STATIC
    ${chipmunk_source_files}
  )
  target_compile_definitions(chipmunk_static PRIVATE CP_AVX2_CONTACT_SOLVER=${CP_AVX2_CONTACT_SOLVER})
  # Tell MSVC to compile the code as C++.
  if(MSVC)
    set_source_files_properties(${chipmunk_source_files} PROPERTIES LANGUAGE CXX)
    set_target_properties(chipmunk_static PROPERTIES LINKER_LANGUAGE CXX)
  endif(MSVC)
  # Sets chipmunk_static to output "libchipmunk.a" not "libchipmunk_static.a"
  set_target_properties(chipmunk_static PROPERTIES OUTPUT_NAME chipmunk)
  if(INSTALL_STATIC)
    install(TARGETS chipmunk_static ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
  endif(INSTALL_STATIC)
endif(BUILD_STATIC)

if(BUILD_SHARED OR INSTALL_STATIC)
  # FIXME: change to PUBLIC_HEADER to allow building frameworks
  install(FILES ${chipmunk_public_header} DESTINATION include/chipmunk)
  install(FILES ${chipmunk_constraint_header} DESTINATION include/chipmunk/constraints)
endif(BUILD_SHARED OR INSTALL_STATIC)
