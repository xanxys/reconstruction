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


# cardbox tower error
#../capturer/scan-20140827-13:23-gakusei-small \


# Exclude 13:23 (because of messed up reflection by window)
time build/recon --debug --convert \
	../data/scan-20140827-12:57-gakusei-small \
	../data/scan-20140827-13:05-gakusei-small \
	../data/scan-20140827-13:11-gakusei-small \
	../data/scan-20140827-13:15-gakusei-small \
	../data/scan-20140827-13:18-gakusei-small \
	../data/scan-20140827-13:26-gakusei-small \
	../data/scan-20140827-13:31-gakusei-small \
	../data/scan-20140827-13:34-gakusei-small \
	../data/scan-20140827-13:38-gakusei-small \
	../data/scan-20140827-13:42-gakusei-small \
	../data/scan-20140827-13:45-gakusei-small \
	../data/scan-20140827-13:49-gakusei-small \
	../data/scan-20140827-13:53-gakusei-small \
	../data/scan-20140827-13:57-gakusei-small \
	--hint hint-small-gakusei-20140827.json \
	--output scene-small-gakusei

# Unfortunately, it seems that 13:26 (current target)
# doesn't align well with ALL OTHER scan in
# ../capturer/scan-20140827-13:31-gakusei-small \
# ../capturer/scan-20140827-13:34-gakusei-small \
# ../capturer/scan-20140827-13:38-gakusei-small \
# ../capturer/scan-20140827-13:42-gakusei-small \
# ../capturer/scan-20140827-13:45-gakusei-small \
# ../capturer/scan-20140827-13:49-gakusei-small \
# ../capturer/scan-20140827-13:53-gakusei-small \
# ../capturer/scan-20140827-13:57-gakusei-small
# Presumably, there's very little overlap in these pairs.