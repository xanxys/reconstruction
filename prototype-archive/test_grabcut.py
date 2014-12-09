#!/bin/python2
import numpy as np
import cv2
import json

scan_ids = [entry["scan_id"] for entry in json.load(open("pose-20140827.json"))]

for (ix_scan, scan_id) in enumerate(scan_ids):
    print("Procesing scan %d : %s" % (ix_scan, scan_id))
    try:
        for ix_cluster in range(20):
            print("Processing cluster %d" % ix_cluster)
            img = cv2.imread(
                'scan-20140827-12:57-gakusei-small/cluster_proj_%s-%d.png' % (scan_id, ix_cluster))
            mask_raw = cv2.imread(
                'scan-20140827-12:57-gakusei-small/cluster_mask_%s-%d.png' % (scan_id, ix_cluster), 0)
            mask = mask_raw.copy()
            mask[:] = cv2.GC_BGD
            mask[mask_raw == 255] = cv2.GC_PR_FGD

            bgdModel = np.zeros((1, 65), np.float64)
            fgdModel = np.zeros((1, 65), np.float64)

            cv2.grabCut(img, mask, None, bgdModel, fgdModel, 5, cv2.GC_INIT_WITH_MASK)

            mask2 = np.where((mask == 2) | (mask == 0), 0, 1).astype('uint8')
            img_res = img * mask2[:, :, np.newaxis] + (50 * (1 - mask2))[:, :, np.newaxis]

            img_res[mask_raw == 255, 2] = img_res[mask_raw == 255, 2] / 2 + 120
            img[mask_raw == 255, 2] = img[mask_raw == 255, 2] / 2 + 120
            cv2.imwrite("grabcut-%d-%d.png" % (ix_scan, ix_cluster), img_res)
            cv2.imwrite("grabcut-src-%d-%d.png" % (ix_scan, ix_cluster), img)
    except:
        pass