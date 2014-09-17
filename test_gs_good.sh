#!/bin/sh
# List of 15 scans
# scan-20140827-12:57-gakusei-small
# scan-20140827-13:05-gakusei-small
# scan-20140827-13:11-gakusei-small
# scan-20140827-13:15-gakusei-small
# scan-20140827-13:18-gakusei-small
# scan-20140827-13:23-gakusei-small
# scan-20140827-13:26-gakusei-small <- target
# scan-20140827-13:31-gakusei-small
# scan-20140827-13:34-gakusei-small
# scan-20140827-13:38-gakusei-small
# scan-20140827-13:42-gakusei-small
# scan-20140827-13:45-gakusei-small
# scan-20140827-13:49-gakusei-small
# scan-20140827-13:53-gakusei-small
# scan-20140827-13:57-gakusei-small

# All 15
time ./recon --convert \
	../capturer/scan-20140827-12:57-gakusei-small \
	../capturer/scan-20140827-13:05-gakusei-small \
	../capturer/scan-20140827-13:11-gakusei-small \
	../capturer/scan-20140827-13:15-gakusei-small \
	../capturer/scan-20140827-13:18-gakusei-small \
	../capturer/scan-20140827-13:23-gakusei-small \
	../capturer/scan-20140827-13:26-gakusei-small \
	../capturer/scan-20140827-13:31-gakusei-small \
	../capturer/scan-20140827-13:34-gakusei-small \
	../capturer/scan-20140827-13:38-gakusei-small \
	../capturer/scan-20140827-13:42-gakusei-small \
	../capturer/scan-20140827-13:45-gakusei-small \
	../capturer/scan-20140827-13:49-gakusei-small \
	../capturer/scan-20140827-13:53-gakusei-small \
	../capturer/scan-20140827-13:57-gakusei-small

# time ./recon --convert \
# 	../capturer/scan-20140827-13:23-gakusei-small \
# 	../capturer/scan-20140827-13:26-gakusei-small

# first half: mostly ok
# time ./recon --convert \
# 	../capturer/scan-20140827-12:57-gakusei-small \
# 	../capturer/scan-20140827-13:05-gakusei-small \
# 	../capturer/scan-20140827-13:11-gakusei-small \
# 	../capturer/scan-20140827-13:15-gakusei-small \
# 	../capturer/scan-20140827-13:18-gakusei-small \
# 	../capturer/scan-20140827-13:23-gakusei-small \
# 	../capturer/scan-20140827-13:26-gakusei-small

# second half:
# time ./recon --convert \
# 	../capturer/scan-20140827-13:26-gakusei-small \
# 	../capturer/scan-20140827-13:31-gakusei-small \
# 	../capturer/scan-20140827-13:34-gakusei-small \
# 	../capturer/scan-20140827-13:45-gakusei-small

# bad in 2nd half
# ../capturer/scan-20140827-13:38-gakusei-small
# ../capturer/scan-20140827-13:42-gakusei-small
# ../capturer/scan-20140827-13:49-gakusei-small
# ../capturer/scan-20140827-13:53-gakusei-small
# ../capturer/scan-20140827-13:57-gakusei-small

# merged good parts
# time ./recon --convert \
# 	../capturer/scan-20140827-12:57-gakusei-small \
# 	../capturer/scan-20140827-13:05-gakusei-small \
# 	../capturer/scan-20140827-13:11-gakusei-small \
# 	../capturer/scan-20140827-13:15-gakusei-small \
# 	../capturer/scan-20140827-13:18-gakusei-small \
# 	../capturer/scan-20140827-13:23-gakusei-small \
# 	../capturer/scan-20140827-13:26-gakusei-small \
# 	../capturer/scan-20140827-13:26-gakusei-small \
# 	../capturer/scan-20140827-13:31-gakusei-small \
# 	../capturer/scan-20140827-13:34-gakusei-small \
# 	../capturer/scan-20140827-13:45-gakusei-small
