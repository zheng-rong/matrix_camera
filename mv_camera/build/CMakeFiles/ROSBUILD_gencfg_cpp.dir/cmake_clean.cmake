FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/mv_camera/srv"
  "CMakeFiles/ROSBUILD_gencfg_cpp"
  "../cfg/cpp/mv_camera/mv_cameraConfig.h"
  "../docs/mv_cameraConfig.dox"
  "../docs/mv_cameraConfig-usage.dox"
  "../src/mv_camera/cfg/mv_cameraConfig.py"
  "../docs/mv_cameraConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gencfg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
