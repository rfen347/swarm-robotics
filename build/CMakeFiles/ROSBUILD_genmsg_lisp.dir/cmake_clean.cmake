FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/project1/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/move.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_move.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
