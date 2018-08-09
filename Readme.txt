ryan_ws

1. The directory, ryan_arms, contains model of ryan arms.
2. The directory, ryan_arms_plugin, contains plugin for ryan arms control


Running plugin
1. Open a terminal
2. Type belows
  cd ~/ryan_ws/ryan_arms_plugin/build
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:~/ryan_ws/ryan_arms_plugin/build
  gazebo ../ryan_arms.world

