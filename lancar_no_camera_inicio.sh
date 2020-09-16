#!/bin/bash

# Dormir um pouco para esperar iniciar
sleep 7s

# Buscar a fonte dos pacotes pra garantir
source /opt/ros/melodic/setup.bash
source /home/pepo/pepo_ws/devel/setup.bash

# Lancar a merda do no da camera que so da chateacao
roslaunch usb_cam usb_cam.launch
