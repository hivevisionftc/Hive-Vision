[Docs](https://hivevisionftc.github.io/HiveVisionDocs/) · [Join the Discord](https://discord.gg/jfj54TUmPT) · [Back to the README](README.md)

# Contributing to Hive Vision

Thanks for helping out. Two kinds of contributions help most: detector and model data, and code.

Data has its own workflow, so see the [training data guide](docs/training_data.md). This page is about code. If you have questions before you start, [join the Discord](https://discord.gg/jfj54TUmPT).

## Scope

Hive Vision is a model-plus-advice project. It ships published reference weights (the Limelight 3A SSD-MobileNetV2 TFLite), Control Hub OpenCV pipelines, and the FTC-side `ftc_ball_chase_lib`.

The API is small on purpose. Before opening a PR, say what problem it solves, whether that's the tracker, the docs, or the robot libraries.

## Setting up

For the PC tooling you'll need Python 3.10 or newer:

```
pip install -r requirements.txt
```

The FTC-side library is standalone, so you don't need Python for it. Build and test it with the laptop self-test:

```
cd ftc_ball_chase_lib
.\compile_check.cmd -runall
```

## What a good PR looks like

- **One logical change per PR.** If the docs and the code are coupled, like a new class field and its reference entry, change them together.
- **Run the eval before and after** any detector or config change, and put the numbers in the PR description:

  ```
  python cv/tools/eval_cv_vs_yolo.py
  ```

  The [README](https://github.com/hivevisionftc/Hive-Vision/blob/main/README.md) has the exact commands. (The Lab track eval was retired along with the track in 2026-10. Its tools live in `archive/lab/`.)
- **Never reorder shipped class IDs or labels.** The `yellow`, `red`, `blue` order matters everywhere: the Limelight labels and the Control Hub pipelines both depend on it.
- **Keep the performance table in the README in sync** with any measured change. Those numbers are candidate-level detector benchmarks, not robot results, so keep the caveats next to them.
- **Reorganize docs, don't prune them.** When you rework a page, merge the content instead of deleting it.

## Testing

- Docs-only PRs don't need a test run.
- For library changes, run `.\compile_check.cmd -runall`. All tests must pass.
- For detector or tooling changes, run the relevant eval script on a short clip and paste the output. CI also runs an eval smoke check on every PR. It fails on tooling regressions and prints the metrics for review.

## Licensing

Code, docs and configuration are MIT. The archived YOLO model artifacts in `archive/yolo-pc/` and any Ultralytics-dependent material are AGPL-3.0. See [THIRD_PARTY_NOTICES.md](https://github.com/hivevisionftc/Hive-Vision/blob/main/THIRD_PARTY_NOTICES.md).

By contributing, you agree to release your contribution under those terms.
