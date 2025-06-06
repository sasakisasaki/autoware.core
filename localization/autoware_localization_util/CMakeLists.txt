cmake_minimum_required(VERSION 3.14)
project(autoware_localization_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/util_func.cpp
  src/smart_pose_buffer.cpp
  src/tree_structured_parzen_estimator.cpp
  src/covariance_ellipse.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_smart_pose_buffer
    test/test_smart_pose_buffer.cpp
    src/smart_pose_buffer.cpp
  )

  ament_auto_add_gtest(test_tpe
    test/test_tpe.cpp
    src/tree_structured_parzen_estimator.cpp
  )

  ament_auto_add_gtest(test_covariance_ellipse
    test/test_covariance_ellipse.cpp
    src/covariance_ellipse.cpp
  )

  ament_auto_add_gtest(test_util_func
    test/test_util_func.cpp
    src/util_func.cpp
  )

endif()

ament_auto_package(
  INSTALL_TO_SHARE
)
