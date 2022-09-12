# Docker

```terminal
wsl

docker build -t cpp-slam .
docker run -it --rm -v ${PWD}:/develop cpp-slam
docker run -it --rm -v /tmp/.X11-unix:/tmp/.X11-unix -v ${PWD}:/develop -e DISPLAY=$DISPLAY -p 8888:8888 -p 6006:6006 cpp-slam
```