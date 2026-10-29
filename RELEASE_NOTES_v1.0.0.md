# Hive Vision v1.0.0

Hive Vision is an FTC ball-detection toolkit with two deployment options:

- **Limelight 3A (SSD-MobileNetV2):** a TFLite detector that runs on the Limelight 3A neural detector (CPU runtime).
- **PC / ONNX Runtime YOLO:** a YOLOv8n reference model exported to ONNX for ONNX Runtime hosts (dev PC, Jetson). Limelight neural detectors do not run ONNX.
- **Control Hub OpenCV:** a lightweight VisionPortal processor that runs color and shape detection directly on the Control Hub without a model or coprocessor.

## Included

- `neural-net/weights/best_limelight3a_ssd_mobilenetv2_300x300.tflite` for Limelight 3A
- `neural-net/weights/labels.txt` with Pollen/Nectar class names
- `neural-net/weights/best.onnx` for PC / ONNX Runtime (not a Limelight)
- `neural-net/weights/best.pt` for PC inference and future export or tuning
- Control Hub `VisionProcessor` implementation
- HSV tuning and evaluation tools
- Annotated MP4 demos and looping GIF previews
- YOLO and OpenCV sample images
- Evaluation report and cached truth data
- Training-data contribution workflow

## Model training

The YOLOv8n model was trained on approximately 7,000 labeled images combining synthetic renders and real footage. The data includes yellow, red, and blue balls, along with difficult examples involving distance, blur, occlusion, field lighting, and robot-colored distractions.

## Important limitations

The OpenCV detector produces candidate signals rather than guaranteed ball detections. Its performance depends on lighting, white balance, camera resolution, and field conditions. Require detections to persist across multiple frames before the robot acts on them.

The Limelight and Control Hub paths should be tested on the actual robot hardware before competition.

## Licensing

Original Hive Vision code and documentation are MIT-licensed. The YOLO
weights and Ultralytics-dependent material are subject to Ultralytics' AGPL-3.0
licensing terms; see `THIRD_PARTY_NOTICES.md`.

## Demo source

The included example footage and sample screenshot were derived from this kickoff video:

https://www.youtube.com/watch?v=gO98TkgY0kI

## Community and contributions

Share robot tests, detector results, and training data through the [Hive Vision Discord server](https://discord.gg/jfj54TUmPT) or [GitHub Issues](https://github.com/sidhuharjas/Hive-Vision/issues).

Useful training data includes empty-field and real-robot footage, 360-degree field views, varied ball distances and angles, motion blur, partial occlusion, and hard negatives such as panels, tape, shadows, and reflections.

Contributors whose accepted data or testing work is used may request a certificate recognizing their specific contribution.
