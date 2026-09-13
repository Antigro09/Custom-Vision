/* Adapter to the private pinned AprilTag 3.4.5 source. The included source is
 * Copyright (C) 2013-2016 The Regents of The University of Michigan, BSD-2-Clause;
 * see APRILTAG_LICENSE.txt. No source or binaries from system installs change.
 */
#include "apriltag.c"

/* Decode/refine only GPU-proposed quads, preserving standard AprilTag quality
 * and decoded orientation. Pixels are raw; callers map rectified GPU corners
 * back to the raw camera image first. Points use positive image-space winding.
 */
zarray_t *vision_apriltag_verify_quads(apriltag_detector_t *td, image_u8_t *image,
                                     const float *corners, int count)
{
    zarray_t *quads = zarray_create(sizeof(struct quad));
    zarray_t *detections = zarray_create(sizeof(apriltag_detection_t *));
    for (int index = 0; index < count; ++index) {
        struct quad quad = {0};
        for (int corner = 0; corner < 4; ++corner) {
            quad.p[corner][0] = corners[index * 8 + corner * 2];
            quad.p[corner][1] = corners[index * 8 + corner * 2 + 1];
        }
        zarray_add(quads, &quad);
    }
    struct quad_decode_task task = {
        .i0 = 0, .i1 = count, .quads = quads, .td = td,
        .im = image, .detections = detections, .im_samples = NULL
    };
    quad_decode_task(&task);
    for (int index = 0; index < count; ++index) {
        struct quad *quad;
        zarray_get_volatile(quads, index, &quad);
        if (quad->H) matd_destroy(quad->H);
        if (quad->Hinv) matd_destroy(quad->Hinv);
    }
    zarray_destroy(quads);
    return detections;
}
