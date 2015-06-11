FILE(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/mv_camera/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/PropertyMap.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_PropertyMap.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
