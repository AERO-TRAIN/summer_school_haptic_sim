# AERO-TRAIN Summer School 

## Build Docker container
```
$ cd docker
$ docker build . -t summer_school_haptic:1.0
```
## Run container
```
$ xhost +
$ docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --net=host --privileged summer_school_haptic:1.0
```
## Run simulation
```
$ roslaunch summer_school_haptic_sim aerotrain_sim.launch
```
> **_NOTE:_**  The root of this repo includes a foxglove layout for data visualization
