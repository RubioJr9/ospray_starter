add_library(ospray_module_tensor_geometry SHARED)

ispc_include_directories(
    ${PROJECT_SOURCE_DIR}/ospray/include
    ${PROJECT_SOURCE_DIR}/ospray
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
    ${RKCOMMON_INCLUDE_DIRS}
    ${EMBREE_INCLUDE_DIRS}
)

ispc_target_add_sources(ospray_module_tensor_geometry
  EllipsoidIntersect.ih
  ellipsoid.cpp
  ellipsoid.ispc
  SuperquadricIntersect.ih
  superquadric.cpp
  superquadric.ispc
  SphericalHarmonicsIntersect.ih
  SphericalHarmonics.cpp
  spherical_harmonics.ispc
  module_init.cpp
)

target_link_libraries(ospray_module_tensor_geometry PUBLIC
    ospray_module_cpu)
