#!/bin/bash

# source install_isolated/setup.bash

map_dir="/home/agrobot/Robot_ws/src/RobotCar/map"
map_name="0515"

# 检查文件夹是否存在, 如果不存在就创建文件夹
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 正在创建文件夹"
  mkdir -p $map_dir
fi

# finish slam 序列化
rosservice call /finish_trajectory 0

# 等待1秒，确保轨迹已经完成
sleep 1

# make pbstream
rosservice call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

# 等待2秒，确保pbstream文件已经写入
sleep 2

echo "尝试第一种方法转换地图..."
# pbstream to map 第一种方法
rosrun cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name -resolution=0.05 || {
  
  echo "第一种方法失败，尝试第二种方法..." 
  
  # 第二种方法：直接使用.pbstream文件
  echo "将pbstream文件复制到最终位置..."
  cp $map_dir/$map_name.pbstream $map_dir/$map_name.pbstream.backup
  
  echo "地图已保存在 $map_dir/$map_name.pbstream"
  echo "您可以使用以下命令加载地图进行导航："
  echo "roslaunch RobotCar nav08_cartographer_localization.launch map_file:=$map_dir/$map_name.pbstream"
}

echo "建图完成！"
