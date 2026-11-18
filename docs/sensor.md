# Sensor footprint model: SIYI A8 mini

Code: `swarm/sensor.py`. Run `python -m swarm.sensor` to print the numbers below.

## Product and verified figures

A small 3-axis gimbal EO camera, 95 g, which a small quad can carry. All figures come from the manufacturer (checked 2026-11-19):

| Figure | Value used | Source (quoted) |
|---|---|---|
| Horizontal FOV | 81° | [S1] [S2] "FOV Diagonal: 93° Horizontal: 81°"; [S3] "HFOV: 81°" |
| Diagonal FOV | 93° (reference only) | same |
| Vertical FOV | **51.3°, derived** (see below) | [S3] says "VFOV: 65°" |
| Recording resolution | 3840 x 2160 (4K) | [S2] "4K (3840 x 2160) @ 25 fps"; [S1] says "4K (4096 × 2160) @ 25 fps" |
| Live HDMI output | 1920 x 1080 | [S3] Q9 "1080P" |
| Frame rate | 25 fps, all modes | [S1] [S2] |
| Weight | 95 g | [S1] [S2] |
| Stabilisation | ±0.01°, 3 axes | [S1] [S2] "Angular Vibration Range ±0.01°" |

- [S1] Official spec page: https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/spec/
- [S2] Official user manual v1.10 (PDF, spec table p.15): https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf
- [S3] Official FAQ: https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/faq/

**Where the official sources disagree:**
1. **4K width.** The spec page says 4096 x 2160 and the manual says 3840 x 2160. I used the manual's 3840, because the manual also says the still-photo resolution follows it. The two differ by 6 % in GSD only. The footprint does not depend on pixel count.
2. **Vertical FOV.** The FAQ says 65°. Under a rectilinear lens, HFOV 81°, VFOV 65° and DFOV 93° describe a **4:3** frame: tan(40.5°)·3/4 gives VFOV 65.3° and DFOV 93.7°. Every recording mode is 16:9, so the model derives the VFOV of the 16:9 frame from the HFOV and the aspect ratio. This assumes the 81° spans the full recorded width:

   VFOV = 2·atan(tan(81°/2) · 9/16) = **51.3°**

   This is the smaller, conservative choice, so coverage is never over-claimed. If 65° does hold for the recorded frame, the along-track length at 60 m is 76.5 m instead of 57.7 m. The across-track width, which sets lane spacing, is the same either way. The manufacturer gives no figure for lens distortion, so the model does not include it. Only a calibration of a real unit can settle this.

## Model

- **Nadir pointing.** The gimbal holds the camera straight down, so the footprint does not tilt with roll or pitch.
- **Flat ground** at z = 0. `alt` is the logged z in metres above the area origin.
- **Rectangle**: across = 2·alt·tan(HFOV/2), along = 2·alt·tan(VFOV/2). The wide axis lies across the track. With yaw 0 (north), across runs east-west. The rectangle rotates with the logged yaw (compass convention: 0 = north, clockwise).
- A ground point counts as seen when its along-track and across-track offsets are within half the length and half the width.

| Altitude | Across (E-W at yaw 0) | Along (N-S) | Area | GSD 4K | GSD 1080p stream |
|---|---|---|---|---|---|
| 50 m | 85.4 m | 48.0 m | 4 103 m² | 2.22 cm/px | 4.45 cm/px |
| 60 m | **102.5 m** | **57.7 m** | 5 909 m² | 2.67 cm/px | 5.34 cm/px |
| 70 m | 119.6 m | 67.3 m | 8 042 m² | 3.11 cm/px | 6.23 cm/px |

The allocator's assumed 102 m x 57 m at 60 m agrees to within 1 m.

## Range

The requirement is that a person seen from above (about 0.5 m across the shoulders) covers at least 10 pixels. That means GSD ≤ 5 cm/px.

- **4K onboard recording:** the maximum useful altitude is 0.05 · 3840 / (2·tan 40.5°) = **112 m**. At 60 m the GSD is 2.67 cm/px, so a person covers about 19 px. The model treats anything above 112 m as unseen.
- **1080p live stream:** the maximum is 56 m. At 60 m the GSD is 5.34 cm/px, about 9 px on a person. That is marginal, so a live-detection requirement would mean flying the lower half of the band.

## Update rate

The camera records at 25 fps. The coverage metric samples at **10 Hz**, the rate of the agent state log (one footprint per log row). At 10 m/s the vehicle moves 1 m between rows. That is much less than the 48 m along-track footprint at 50 m, so sampling at 10 Hz rather than 25 fps misses no ground.
