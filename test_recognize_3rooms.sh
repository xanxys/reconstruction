#!/bin/sh

scons -j 8

# small
# * scan-20140801-18:54-gakusei-small-1
# * scan-20140801-18:57-gakusei-small-2
# * scan-20140801-19:00-gakusei-small-3
echo "== Gakusei-Small"
time ./recon --convert ../capturer/scan-20140801-18:54-gakusei-small-1/ ../capturer/scan-20140801-18:57-gakusei-small-2/

# ocha
# * scan-20140801-18:44
# * scan-20140801-18:47
# * scan-20140801-18:50-ocha-2
echo "== Ocha"
time ./recon --convert ../capturer/scan-20140801-18:44 ../capturer/scan-20140801-18:47 ../capturer/scan-20140801-18:50-ocha-2

# large
# * test-20140801-15:24-gakusei-table
# * scan-20140801-18:41-gakusei-large
echo "== Gakusei-Large"
time ./recon --convert ../capturer/scan-20140801-18:41-gakusei-large ../capturer/test-20140801-15:24-gakusei-table
