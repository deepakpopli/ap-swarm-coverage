# Sensor footprint model: SIYI A8 mini

Code: `swarm/sensor.py`.

## Product and verified figures

The SIYI A8 mini is a small 3-axis gimbal EO camera that weighs 95 g, so a small quad can carry it. All figures come from the manufacturer (checked 2026-11-19):

| Figure | Value used | Source (quoted) |
|---|---|---|
| Horizontal FOV | 81° | [S1] [S2] "FOV Diagonal: 93° Horizontal: 81°"; [S3] "HFOV: 81°" |
| Vertical FOV | **51.3°, derived** (see below) | [S3] says "VFOV: 65°" |
| Recording resolution | 3840 x 2160 (4K) | [S2] "4K (3840 x 2160) @ 25 fps"; [S1] says "4K (4096 × 2160) @ 25 fps" |
| Live HDMI output | 1920 x 1080 | [S3] Q9 "1080P" |
| Frame rate | 25 fps, all modes | [S1] [S2] |
| Stabilisation | ±0.01°, 3 axes | [S1] [S2] "Angular Vibration Range ±0.01°" |

- [S1] Official spec page: https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/spec/
- [S2] Official user manual v1.10 (PDF, spec table p.15): https://res.siyi.biz/oss/other/2026/06/15/A8_mini_User_Manual_v1_10_563cde30.pdf
- [S3] Official FAQ: https://siyi.biz/en/product/tri-axis-single-camera-gimbal/a8-mini/faq/

**Where the official sources disagree:**
1. **4K width.** The spec page says 4096 x 2160 and the manual says 3840 x 2160. I used the manual's 3840, because the manual also says the still-photo resolution follows it. The two differ by 6 % in GSD only. The footprint does not depend on the pixel count.
2. **Vertical FOV.** The FAQ says 65°. Under a rectilinear lens, HFOV 81°, VFOV 65° and DFOV 93° describe a **4:3** frame: tan(40.5°)·3/4 gives VFOV 65.3° and DFOV 93.7°. Every recording mode is 16:9, so the model derives the VFOV of the 16:9 frame from the HFOV and the aspect ratio. This assumes the 81° spans the full recorded width:

   VFOV = 2·atan(tan(81°/2) · 9/16) = **51.3°**

   This is the smaller, conservative choice, so coverage is never over-claimed. If 65° does hold for the recorded frame, the along-track length is 27 % longer. The across-track width, which sets the lane spacing, is the same either way. The manufacturer gives no figure for lens distortion, so the model leaves it out. Only a calibration of a real unit can settle this.

## Model

- **Nadir pointing.** The gimbal holds the camera straight down, so the footprint does not tilt with roll or pitch.
- **Flat ground** at z = 0. The altitude is the logged z in metres above the area origin.
- **Rectangle**: across = 2·alt·tan(HFOV/2), along = 2·alt·tan(VFOV/2). The wide side lies across the track. With yaw 0 (north), across runs east-west. The rectangle turns with the logged yaw (compass convention: 0 = north, clockwise).
- A ground point counts as seen when its along-track and across-track offsets are within half the length and half the width.
- The allocator plans lanes with `lane_footprint`, which is the footprint rounded down to whole metres: 85 x 48 m at 50 m.

| Altitude | Across (E-W at yaw 0) | Along (N-S) | Area | GSD 4K | GSD 1080p stream |
|---|---|---|---|---|---|
| 50 m | **85.4 m** | **48.0 m** | 4 103 m² | 2.22 cm/px | 4.45 cm/px |
| 60 m | 102.5 m | 57.7 m | 5 909 m² | 2.67 cm/px | 5.34 cm/px |
| 70 m | 119.6 m | 67.3 m | 8 042 m² | 3.11 cm/px | 6.23 cm/px |

## Altitude: 50 m

The requirement is that a person seen from above (about 0.5 m across the shoulders) covers at least 10 pixels. That means a ground sample distance (GSD) of at most 5 cm/px. GSD = across / image width in pixels.

- **1080p live stream** (1920 px): GSD ≤ 5 cm/px needs an altitude of at most 0.05 · 1920 / (2·tan 40.5°) = **56 m**.
- **4K onboard recording** (3840 px): the limit is 112 m.

Detection should work on the live stream, not only afterwards on the recording. The cruise altitude is therefore **50 m** (`config.ALT_CRUISE_M`). That gives a 4.45 cm/px stream and an 85.4 x 48.0 m footprint. Hold altitudes (cruise ± 10 m per layer) are briefly above 56 m. The coverage metric still credits their footprint, so it slightly overstates live-stream detection while an agent holds.

## Update rate

The camera records at 25 fps. The coverage metric uses one footprint per agent log row, at 20 Hz. At 11.5 m/s the vehicle moves 0.6 m between rows, much less than the 48 m along-track footprint. Sampling at 20 Hz rather than 25 fps therefore misses no ground.
