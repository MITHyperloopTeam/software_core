xhost +local:root
docker run --name mithl -e DISPLAY -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw -v $(pwd):/mithl/ --rm --ipc=host -it gizatt/mithl /bin/bash
xhost -local:root
