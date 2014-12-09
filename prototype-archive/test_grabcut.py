#!/bin/python2
import numpy as np
import cv2
import json

if __name__ == '__main__':
    scan_ids = [entry["scan_id"] for entry in json.load(open("pose-20140827.json"))]
    stat = {
        "clusters_processed": 0,
        "clusters_ok": 0
    }

    for (ix_scan, scan_id) in enumerate(scan_ids):
        print("Procesing scan %d : %s" % (ix_scan, scan_id))
        try:
            for ix_cluster in range(20):
                print("Processing cluster %d" % ix_cluster)
                img = cv2.imread(
                    'scan-20140827-12:57-gakusei-small/cluster_proj_%s-%d.png' % (scan_id, ix_cluster))
                mask_raw = cv2.imread(
                    'scan-20140827-12:57-gakusei-small/cluster_mask_%s-%d.png' % (scan_id, ix_cluster), 0)
                # end of cluster for current scan reached?
                if img is None or mask_raw is None:
                    break
                stat["clusters_processed"] += 1
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

                # extract blobs
                mask_fine = mask2 * 255
                contours = cv2.findContours(mask_fine, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_L1)[0]
                print("#contours=%d" % len(contours))
                # number of clusters other than 1 are unstable, and should be rejected
                if len(contours) != 1:
                    continue
                contour = contours[0]
                # reject degenerate contours
                if len(contour) < 3:
                    continue
                contour_simple = cv2.approxPolyDP(contour, 5, closed=True)
                print("Contour simplification: %d -> %d" % (len(contour), len(contour_simple)))

                cv2.drawContours(img_res, [contour], -1, (0, 0, 255))
                cv2.drawContours(img_res, [contour_simple], -1, (0, 255, 0))
                cv2.imwrite("grabcut-cs-%d-%d.png" % (ix_scan, ix_cluster), img_res)
                if len(contour_simple) < 3:
                    continue
                stat["clusters_ok"] += 1
        except KeyboardInterrupt:
            raise

    print(stat)
