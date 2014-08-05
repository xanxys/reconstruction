#!/bin/sh

scons -j 8

./recon --convert ../capturer/scan-20140801-18:54-gakusei-small-1/ --convert ../capturer/scan-20140801-18:57-gakusei-small-2/
