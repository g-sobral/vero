Launch on Pioneer:
$ roslaunch ransac_project pioneer.launch
$ roslaunch ransac_project ransac.launch
$ rosrun ransac_project dataplot (just for data visualization)

Launch on VERO:
$ ssh vero@EMBSYS0 (roscore start)
$ roslaunch sensors sensors.launch
$ roslaunch ransac_project ransac.launch
