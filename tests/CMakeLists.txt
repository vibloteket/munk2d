add_executable(chipmunk_basic_tests
  test_basic.c
)

add_executable(chipmunk_bbtree_tests
  test_bbtree.c
)

target_include_directories(chipmunk_basic_tests PRIVATE ${chipmunk_SOURCE_DIR}/include)
target_link_libraries(chipmunk_basic_tests PRIVATE chipmunk_static)

target_include_directories(chipmunk_bbtree_tests PRIVATE ${chipmunk_SOURCE_DIR}/include)
target_link_libraries(chipmunk_bbtree_tests PRIVATE chipmunk_static)

if(ANDROID OR UNIX)
  target_link_libraries(chipmunk_basic_tests PRIVATE m)
  target_link_libraries(chipmunk_bbtree_tests PRIVATE m)
endif()

if(MSVC)
  set_source_files_properties(test_basic.c PROPERTIES LANGUAGE CXX)
  set_target_properties(chipmunk_basic_tests PROPERTIES LINKER_LANGUAGE CXX)
  set_source_files_properties(test_bbtree.c PROPERTIES LANGUAGE CXX)
  set_target_properties(chipmunk_bbtree_tests PROPERTIES LINKER_LANGUAGE CXX)
endif()

add_executable(chipmunk_poly_point_tests test_poly_point_query.c)
target_include_directories(chipmunk_poly_point_tests PRIVATE ${chipmunk_SOURCE_DIR}/include)
target_link_libraries(chipmunk_poly_point_tests PRIVATE chipmunk_static)
if(ANDROID OR UNIX)
  target_link_libraries(chipmunk_poly_point_tests PRIVATE m)
endif()
if(MSVC)
  set_source_files_properties(test_poly_point_query.c PROPERTIES LANGUAGE CXX)
  set_target_properties(chipmunk_poly_point_tests PROPERTIES LINKER_LANGUAGE CXX)
endif()
add_test(NAME chipmunk_poly_point_tests COMMAND chipmunk_poly_point_tests)
set_tests_properties(chipmunk_poly_point_tests PROPERTIES TIMEOUT 30)

add_executable(chipmunk_constraint_filter_tests test_constraint_filter.c)
target_include_directories(chipmunk_constraint_filter_tests PRIVATE ${chipmunk_SOURCE_DIR}/include)
target_link_libraries(chipmunk_constraint_filter_tests PRIVATE chipmunk_static)
if(ANDROID OR UNIX)
  target_link_libraries(chipmunk_constraint_filter_tests PRIVATE m)
endif()
if(MSVC)
  set_source_files_properties(test_constraint_filter.c PROPERTIES LANGUAGE CXX)
  set_target_properties(chipmunk_constraint_filter_tests PROPERTIES LINKER_LANGUAGE CXX)
endif()
add_test(NAME chipmunk_constraint_filter_tests COMMAND chipmunk_constraint_filter_tests)
set_tests_properties(chipmunk_constraint_filter_tests PROPERTIES TIMEOUT 30)

add_test(NAME chipmunk_basic_tests COMMAND chipmunk_basic_tests)
add_test(NAME chipmunk_bbtree_tests COMMAND chipmunk_bbtree_tests)

add_executable(chipmunk_sleep_wake_tests test_sleep_wake.c)
target_include_directories(chipmunk_sleep_wake_tests PRIVATE ${chipmunk_SOURCE_DIR}/include)
find_package(Threads REQUIRED)
target_link_libraries(chipmunk_sleep_wake_tests PRIVATE chipmunk_static Threads::Threads)

if(ANDROID OR UNIX)
  target_link_libraries(chipmunk_sleep_wake_tests PRIVATE m)
endif()

if(MSVC)
  set_source_files_properties(test_sleep_wake.c PROPERTIES LANGUAGE CXX)
  set_target_properties(chipmunk_sleep_wake_tests PROPERTIES LINKER_LANGUAGE CXX)
endif()

add_test(NAME chipmunk_sleep_wake_tests COMMAND chipmunk_sleep_wake_tests)
add_test(NAME chipmunk_hasty_sleep_wake_tests COMMAND chipmunk_sleep_wake_tests --hasty)
set_tests_properties(chipmunk_sleep_wake_tests chipmunk_hasty_sleep_wake_tests PROPERTIES TIMEOUT 30)

add_executable(chipmunk_constraint_list_tests test_constraint_lists.c)
target_include_directories(chipmunk_constraint_list_tests PRIVATE ${chipmunk_SOURCE_DIR}/include)
target_link_libraries(chipmunk_constraint_list_tests PRIVATE chipmunk_static)
if(ANDROID OR UNIX)
  target_link_libraries(chipmunk_constraint_list_tests PRIVATE m)
endif()
if(MSVC)
  set_source_files_properties(test_constraint_lists.c PROPERTIES LANGUAGE CXX)
  set_target_properties(chipmunk_constraint_list_tests PROPERTIES LINKER_LANGUAGE CXX)
endif()
add_test(NAME chipmunk_constraint_list_tests COMMAND chipmunk_constraint_list_tests)
set_tests_properties(chipmunk_constraint_list_tests PROPERTIES TIMEOUT 30)
