
cd unitree-a1-ros/catkin_ws/src/py_lecture/script 
sudo apt-get install python3-tk

##  Rosbag使う場合  ##

python3 rosbag_analisys_pose.py lcm_bag.bag 

##　lcm_bag.bag はRosbagデータ  ##


##  CSV使う場合  ##

python3 pyplot.py

##  中のCSVファイル名を任意のものに書き換える  ##


##  両方使う場合  ##

python3 mixplot.py lcm_bag.bag

