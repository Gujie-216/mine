# Budget-Aware Monocular ORB-SLAM3 on Raspberry Pi

This project studies whether tracking feedback can reduce the tail latency of monocular visual SLAM on an ARM CPU while preserving trajectory accuracy and coverage. The target platform is a Raspberry Pi 4B. TurtleBot3 camera recordings provide a separate ground-robot evaluation.

**Project status:** research design. The baseline command below runs upstream ORB-SLAM3. The adaptive extractor, board measurements and evaluation artifacts are planned. The lidar simulation in the parent repository is a separate project. [Research plan and protocol (Chinese)](RESEARCH_PLAN.zh.md)

## Methods

1. **Adaptive feature budget.** Use the previous frame's verified map-point inliers, spatial coverage and recent tracking latency to choose the next normal-tracking budget. Preserve the upstream initialization path and restore the full budget on degraded tracking.
2. **Budgeted FAST retries.** Run the first FAST pass in every cell. During stable tracking, allocate low-threshold retries across empty cells according to regional candidate density. Record the number of calls saved and the number of useful points lost.

## Reproduce the upstream baseline

On a Linux host with the upstream build dependencies installed, start with the tagged source. Record the resolved commit, compiler, OpenCV build and board configuration with every run. Review the build script's parallel job count before compiling on a memory-limited board. [Upstream requirements and build instructions](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/v1.0-release)

```bash
git clone --branch v1.0-release --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
git rev-parse HEAD
bash build.sh
# Set EUROC_MH01 to the extracted sequence directory containing mav0/.
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt \
  Examples/Monocular/EuRoC.yaml "${EUROC_MH01:?Set EUROC_MH01 first}" \
  Examples/Monocular/EuRoC_TimeStamps/MH01.txt MH01_baseline
```

Use a distinct trajectory name per run. The example takes left-camera images and a timestamp file. Its playback loop waits when processing finishes early. Timing a complete invocation includes that wait. [EuRoC dataset](https://projects.asl.ethz.ch/datasets/euroc-mav/) · [Upstream runner](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/v1.0-release/Examples/Monocular/mono_euroc.cc)

## Evaluation protocol

Use MH_01_easy and V1_01_easy for development, then freeze the configuration before evaluating the remaining nine EuRoC sequences. Compare stock ORB-SLAM3, two fixed budgets, a count-matched fixed budget, adaptive budgets alone, selective retries alone and both methods together. Run each setting five times per test sequence.

Report Sim(3)-aligned ATE, one-second translational and rotational RPE where reference poses permit, input-frame coverage, resets, p50/p95 latency, missed deadlines, dropped frames and peak RSS. Archive trajectories, per-frame tracking states, hardware manifests and failed runs. Report complete-frame playback separately from timestamp-paced playback.

| Result to publish | Unit | Required evidence |
|---|---|---|
| Tracking p95 and capture-to-pose p95 | ms | Board model, input rate, queue policy and per-frame timestamps |
| Paired ATE change and coverage | %, percentage points | Per-sequence errors, uncertainty interval and all failures |
| FAST retry cost and yield | ms, calls, points | Stage timings and retry-derived descriptors |

## Attribution

Cite Campos et al., *ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multi-Map SLAM*, IEEE Transactions on Robotics (2021), and Burri et al., *The EuRoC Micro Aerial Vehicle Datasets*, IJRR (2016). Retain upstream license notices when distributing a modified implementation. [ORB-SLAM3 source and license](https://github.com/UZ-SLAMLab/ORB_SLAM3) · [EuRoC citation](https://doi.org/10.1177/0278364915620033)
