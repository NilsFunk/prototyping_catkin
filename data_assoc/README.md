# Data association

## Without image undistortion
```
rosrun data_association data_association -i "PATH/TO/ROS.BAG" -o "PATH/TO/OUTPUT/FOLDER/
```

## With image undistortion
The camera parameters are fx, fy, cx, cy

```
rosrun data_association data_association -i "PATH/TO/ROS.BAG" -o "PATH/TO/OUTPUT/FOLDER/ -d 0.139066,-0.349607,-0.000163,0.003016 -k 630.043222,630.362170,327.615743,238.386901
```