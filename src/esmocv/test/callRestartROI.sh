# calls the restartROI service with a rect at 100,100 with size 100,100. just to make sure it is restarting
# the syntax sucks. spaces after the : are important. the commas are also needed. 
rosservice call /restartROI '{roi: {x_offset: 100, y_offset: 100, width: 100, height: 100, do_rectify: 0} }'

