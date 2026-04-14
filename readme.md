构建 image
docker build -t tip_obstacle_img .

运行doecker
./run_docker.sh

进入docker后，先执行编译消息包，然后再进行编译tip_obstacle功能包

colcon build --packages-skip tip_obstacle

source install/setup.bash

colcon build --packages-select tip_obstacle