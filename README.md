<div align="center">

**[Read the docs](https://hivevisionftc.github.io/HiveVisionDocs/)** · **[Join the Discord](https://discord.gg/m2yPTprccv)**

[![Docs](https://img.shields.io/badge/DOCS-hivevisionftc.github.io-F2F2F2?style=for-the-badge&labelColor=222222)](https://hivevisionftc.github.io/HiveVisionDocs/)
[![Join the Discord](https://img.shields.io/badge/DISCORD-join%20the%20server-F2F2F2?style=for-the-badge&labelColor=222222&logo=discord&logoColor=white)](https://discord.gg/m2yPTprccv)
[![Latest release](https://img.shields.io/github/v/release/hivevisionftc/Hive-Vision?style=for-the-badge&color=F2F2F2&labelColor=222222)](https://github.com/hivevisionftc/Hive-Vision/releases/latest)

<img src="assets/banner.svg" alt="Hive Vision: computer vision for FIRST Tech Challenge, at your camera of choice" width="100%">

</div>

---

## So, what is this?

Hive Vision is a vision library for FTC that works with whatever camera you've got. A small group of students built it from scratch because we wanted ball tracking that's fast, accurate, and doesn't eat a whole weekend to set up.

Right now it works with two cameras, and they do the job pretty differently.

**Limelight 3A.** A trained neural network runs on the camera's own processor, so the FTC SDK never gets in its way. It gives you the steadiest tracking.

**Webcam.** Plain colour thresholding on the Control Hub, with no model and no TensorFlow. It's quicker to try, but colour thresholding tends to be fussier about lighting.

**Not sure which one to pick?**

- Choose the **Limelight 3A** if you want tracking you can count on in matches, or your events have tricky lighting.
- Choose the **webcam** if you don't have a Limelight and just want to try Hive Vision today, or you're prototyping.

If you have both, start with the Limelight.

## See it work

<table>
  <tr>
    <td align="center" valign="top"><img src="assets/detection.gif" width="400" alt="Hive Vision neural network detecting pollen and nectar"></td>
    <td align="center" valign="top"><img src="assets/driving-chase.gif" width="400" alt="Robot auto-chasing a ball with the Limelight 3A"></td>
  </tr>
  <tr>
    <td align="center" valign="top"><b>Neural detection</b><br>Pollen and nectar, found live</td>
    <td align="center" valign="top"><b>Auto-chase</b><br>Limelight 3A TeleOp from 47 Beavers (13100)</td>
  </tr>
  <tr>
    <td align="center" valign="top"><img src="assets/webcam-chase.gif" width="400" alt="Robot chasing a ball with a webcam"></td>
    <td align="center" valign="top"><img src="assets/pathing.gif" width="200" alt="Robot splining through detected balls"></td>
  </tr>
  <tr>
    <td align="center" valign="top"><b>Webcam chase</b><br>Webcam TeleOp from Owen Cassidy (17113, Hunga Munga)</td>
    <td align="center" valign="top"><b>Spline pathing</b><br>Limelight 3A + PedroPathing from Ajay Govindarajan (22297, Infinibots)</td>
  </tr>
</table>

## Getting started

Setup, usage, all of it lives in the docs: **[hivevisionftc.github.io/HiveVisionDocs](https://hivevisionftc.github.io/HiveVisionDocs/)**. Here's where to look:

- **Brand new?** Start with the [overview](https://hivevisionftc.github.io/HiveVisionDocs/overview/overview/).
- **Got a Limelight 3A?** Follow the [Limelight setup guide](https://hivevisionftc.github.io/HiveVisionDocs/installation/limelight/). You'll upload the model and labels file and you're basically done.
- **Using a webcam?** Follow the [webcam setup guide](https://hivevisionftc.github.io/HiveVisionDocs/installation/webcam/).
- **Want working code to learn from?** There are three example OpModes: [Limelight driving](https://hivevisionftc.github.io/HiveVisionDocs/reference/teleop), [Limelight pathing](https://hivevisionftc.github.io/HiveVisionDocs/reference/auto) and [webcam driving](https://hivevisionftc.github.io/HiveVisionDocs/reference/webcamteleop).
- **Stuck?** Ask in the [Discord](https://discord.gg/m2yPTprccv).

### Webcam in a few lines

This is the whole idea on the webcam side:

```java
BallDetectorPipeline baller = new BallDetectorPipeline();

VisionPortal portal = new VisionPortal.Builder()
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .setCameraResolution(new Size(1280, 720))
        .addProcessor(baller)
        .enableLiveView(true)
        .build();

BallDetectorPipeline.BallBlob red = baller.bestOf(BallDetectorPipeline.BallColor.RED);
if (red != null) {
    telemetry.addData("red", "cx=%.2f cy=%.2f", red.cxNorm, red.cyNorm);
}
```

`bestOf(...)` gives you `null` when nothing of that colour passes the checks. If something does, you get a `BallBlob` with `cxNorm` and `cyNorm`, which are 0 to 1 and relative to the screen. The colours are `YELLOW`, `RED` and `BLUE`.

---

## The neural network

The Limelight model looks for the three game elements:

![yellow_pollen](https://img.shields.io/badge/yellow__pollen-class%200-FFD400?style=for-the-badge&labelColor=000000)
![red_nectar](https://img.shields.io/badge/red__nectar-class%201-E5242B?style=for-the-badge&labelColor=000000)
![blue_nectar](https://img.shields.io/badge/blue__nectar-class%202-2F6BFF?style=for-the-badge&labelColor=000000)

<img src="assets/stats.svg" alt="Hive Vision v2: 97.3% precision, 18 to 21 ms per inference, 25 detections per frame, 14,409 labeled images" width="100%">

v2 is fully 8-bit, which makes it about 6× faster than the old float build. That's what keeps the stream smooth on the 3A's CPU-only runtime. We also retrained it on a pile of hard negatives (field rig, wiring, signage, deep shadows, stuff off the field) so it stops seeing balls that aren't there. Class order and the FTC-side code haven't changed since v1, so upgrading is just swapping the file.

### Model details

| | |
| --- | --- |
| File | `best_limelight3a_ssd_mobilenetv2_300x300_v2_int8_dets25.tflite` |
| Architecture | SSD-MobileNetV2, 300 × 300 |
| Input | `uint8 [1,300,300,3]`, scale 1/255 |
| Output | `TFLite_Detection_PostProcess`: `num[1]`, `scores[1,25]`, `class_ids[1,25]`, `boxes[1,25,4]` (normalized `ymin,xmin,ymax,xmax`), float32 |
| Max detections | 25 |
| Size | 5.19 MB |

**Why 25 detections?** The model keeps the top-N boxes after filtering by score. With N=10, a frame with 14 balls could only ever report 10, and it was the cap holding us back, not the network. 25 gives crowded scenes room, and sparse frames don't change (the average is about 4 per frame on our validation set).

**Reading the raw outputs yourself?** Only trust slots where `i < num_detections[0]`. Anything past that is uninitialized memory and can be garbage.

### How we built it

We retrained SSD-MobileNetV2 (300×300) with the TensorFlow Object Detection API on a mix of synthetic and real ball images. That was 30,000 steps on an RTX 3000 Ada, with a class-weighted loss and baked-in NMS thresholds (score 0.3, IoU 0.6, 20 per class, 20 total). The final total loss was 0.1815, and precision came out at 97.3%.

From there we exported to a SavedModel, wrapped the input so it takes `uint8` directly, and converted to TFLite with full int8 quantization. For calibration we sampled 512 frames evenly across the whole training set. (An early version only used the first N sorted images, which was a bug, and we caught it.) Weights and activations are all 8-bit. Only the post-process outputs stay float32, because that's what the Limelight 3A's neural detector expects.

Before anything ships, we check the output shapes and value ranges, benchmark latency, and run the whole validation set against the previous model.

### The data

14,409 labeled images, split into 11,895 for training and 2,514 for validation. It's a mix of synthetic CAD renders of the game balls, real match and arena footage, and hard-negative frames.

We also used two community datasets, and we're grateful for them:

| Dataset | Author | License |
| --- | --- | --- |
| [FTC-BioBuzz](https://universe.roboflow.com/bluedarkup/ftc-biobuzz-6vjbl) | Bluedarkup on Roboflow | MIT |
| [Chunc Pollen](https://universe.roboflow.com/arushs-workspace/chunc-pollen-l3rnm-wcueq) | Arush's Workspace on Roboflow | CC BY 4.0 |

---

## Tested on real robots

Teams have been running v2 on their own robots and sending back footage:

| ProBotiX v2 testing | Arush v2 testing |
| --- | --- |
| ![ProBotiX v2 testing](https://github.com/hivevisionftc/Hive-Vision/raw/v2.0.0/releases/v2.0.0/probotix_v2_testing.gif) | ![Arush v2 testing](https://github.com/hivevisionftc/Hive-Vision/raw/v2.0.0/releases/v2.0.0/arush_v2_testing.gif) |
| Recorded by Team 16765 ProBotiX | Recorded by Arush, FTC 23511 |

## What's changed

- **[v2.0.0](https://github.com/hivevisionftc/Hive-Vision/releases/tag/v2.0.0):** up to 25 detections per frame, full int8 (about 6× faster), a bigger and harder dataset, and 97.3% precision. Coming from v1? Class order and FTC-side decoding are the same, so no code changes.
- **[v1.1.0](https://github.com/hivevisionftc/Hive-Vision/releases/tag/v1.1.0):** retrained on about 10,000 images. Shadows and off-field objects no longer get detected as game elements.

## Who's behind it

Hive Vision is entirely community built. It was started by **[Harjas Sidhu](https://github.com/sidhuharjas)**, the founder and lead developer, and **[Sid Shah](https://github.com/KnowPa1n)** is a co-developer.

None of this works without the teams who tested it and told us where it broke. The false-positive reports from the field are exactly why v1.1 and v2 exist. Thank you to **Arush** (23511a), **Ajay Govindarajan** (22297), **47 Beavers** (13100), **Owen Cassidy** (17113), **ProBotiX** (16765), **Javi Nashat** (23286) and **Jasper Wolves Blue** (11419).

## Community

Join the [Hive Vision Discord](https://discord.gg/m2yPTprccv) to share robot tests, training data, detector results and setup questions. If you've found a bug, have a suggestion, or want to contribute a dataset, you can also [open an issue on GitHub](https://github.com/hivevisionftc/Hive-Vision/issues).

### Updates

We'll keep updating Hive Vision all season. The plan is a release or model update about every other week, as new real footage, field conditions, camera angles and tricky examples come in. An update might include refreshed SSD-MobileNetV2 weights, better OpenCV HSV settings, new evaluation results, or new demo samples.

### Help us train it

Real footage is the best thing you can send us. To contribute, join the Discord and share a link in the training-data discussion. Please include:

- where the footage came from
- the camera resolution and frame rate
- the lighting conditions
- confirmation that we're allowed to use it

The [training data guide](docs/training_data.md) covers what's most useful to send and exactly how we review it. If you'd like to contribute code instead, start with [CONTRIBUTING.md](CONTRIBUTING.md).

### Contributor certificates

If we accept your training data or testing work, you can ask us for a certificate recognizing your contribution. It can describe work like real-robot data collection, labeling, hard-negative mining, evaluation, or detector testing, and you're welcome to use it in a portfolio or resume. Ask through a GitHub issue or the Discord. The certificate will describe your specific contribution, and it won't suggest you wrote the whole project.

## License

Original Hive Vision code, documentation and configuration are MIT. See [LICENSE](https://github.com/hivevisionftc/Hive-Vision/blob/main/LICENSE). YOLO model artifacts and Ultralytics-dependent material are AGPL-3.0. See [THIRD_PARTY_NOTICES.md](https://github.com/hivevisionftc/Hive-Vision/blob/main/THIRD_PARTY_NOTICES.md).

<br>

<div align="center">

<img src="assets/logo.png" alt="Hive Vision logo" width="140">

</div>
