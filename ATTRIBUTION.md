# Attribution

The eye scene, controller, tests, documentation, and report were created for this project. No external meshes, recordings, image datasets, or pretrained model weights are included.

Dependencies are installed separately and retain their own licenses:

- [MuJoCo](https://github.com/google-deepmind/mujoco), Apache-2.0: physics and rendering. References: [Python bindings](https://mujoco.readthedocs.io/en/stable/python.html), [camera and visualization](https://mujoco.readthedocs.io/en/stable/programming/visualization.html).
- [OpenCV](https://github.com/opencv/opencv), Apache-2.0: color conversion, masks, morphology, and contours. [HSV thresholding reference](https://docs.opencv.org/4.13.0/da/d97/tutorial_threshold_inRange.html).
- [NumPy](https://numpy.org/), BSD-3-Clause: array and geometric operations.
- [Pillow](https://python-pillow.github.io/), HPND: video composition and preview image.
- [ImageIO](https://github.com/imageio/imageio), BSD-2-Clause, and [imageio-ffmpeg](https://github.com/imageio/imageio-ffmpeg), BSD-2-Clause: MP4 encoding through their packaged FFmpeg binary. FFmpeg and codec components retain their own distribution licenses; no encoder binary is included in this repository.

The repository MIT license applies to original project material, not third-party packages.
