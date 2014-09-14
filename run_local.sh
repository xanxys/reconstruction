#!/bin/sh

sudo docker run -ti -v (pwd):/root/local -v (pwd)/../capturer:/root/data 1c6bbe46e13e bash
