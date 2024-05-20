#!/bin/bash
docker build -t rustmap3d --build-arg UID=$(id -u) .