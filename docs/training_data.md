[Docs](https://hivevisionftc.github.io/HiveVisionDocs/) · [Join the Discord](https://discord.gg/m2yPTprccv) · [Back to the README](../README.md)

# Training data

Real footage is the best way to make Hive Vision better. Every field, camera angle and lighting setup shows the model something new, and the false positives you catch are exactly what it gets trained against.

Share your data in the training-data discussion on the [Discord](https://discord.gg/m2yPTprccv), or in a [GitHub issue](https://github.com/hivevisionftc/Hive-Vision/issues).

## What helps most

- **Real robot footage with no balls in view.** This teaches the model what backgrounds, robot parts, field markings and an empty field look like.
- **A 360-degree view of the field from your camera's position**, if you can get one.
- **Lots of ball views** at different distances, angles, lighting conditions and amounts of motion blur.
- **Hard ball cases:** tiny, partly hidden, overlapping, or at the edge of the frame.
- **False-positive bait:** robot panels, tape, shadows, reflections, and anything else that has fooled the detector.

## How to submit

1. Share a link to the original video or image in the Discord or a GitHub issue.
2. Include the camera, resolution, frame rate, lighting and where on the field it was shot, and confirm the footage can be used.
3. If you can, separate the useful empty-field footage from clips where balls are visible.
4. Label visible balls as `yellow`, `red` or `blue`. Don't label things that only look like balls.
5. Add a short note on the difficult cases and what the current model missed.
6. Everything submitted gets reviewed. Accepted data can go into a future every-other-week dataset or model update.

## Recognition

If your training data or testing work is accepted, you can request a certificate recognizing your contribution to Hive Vision. It can describe work like real-robot data collection, labeling, hard-negative mining, evaluation or detector testing, and you can include it in a portfolio or resume. Ask through a GitHub issue or the Discord.

The certificate will describe your specific contribution. It won't suggest you wrote the whole project.

## Want to contribute code instead?

See [CONTRIBUTING.md](../CONTRIBUTING.md).
