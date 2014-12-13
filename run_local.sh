#!/bin/sh

sudo docker run -ti -v (pwd):/root/local -v (pwd)/../capturer:/root/capturer xanxys/recon bash
