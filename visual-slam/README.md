# Tracking-Guided Feature Extraction for Monocular ORB-SLAM3 on TurtleBot3

This repository contains a source patch for ORB-SLAM3 and a trajectory and CPU evaluation tool. The patch targets the CPU on a Raspberry Pi carried by a TurtleBot3. It provides independent switches for feature quotas and low-threshold FAST retries. Board benchmarks and robot trials are pending. [Paper outline, abstract and reviewer Q&A](PAPER.md) · [Original design notes](RESEARCH_PLAN.zh.md)

## Environment

Target: Raspberry Pi 4B, 4 GB RAM, 64-bit Linux, active cooling, calibrated monocular camera. For the pinned release, install Git, CMake, a C++11 compiler, Eigen3, OpenCV 4.4-compatible development libraries, Pangolin and the upstream build dependencies. Use a Python 3 virtual environment with NumPy and PyYAML for evaluation. ROS is optional for recording; the two playback commands read files directly. Report actual OS, dependency versions, board revision, camera settings, governor and temperatures in every run manifest. [Upstream build instructions](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/v1.0-release)

## Build on Linux

```bash
git clone https://github.com/Gujie-216/mine.git
export STUDY="$(cd mine/visual-slam && pwd)"
git clone --branch v1.0-release --depth 1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git
cd ORB_SLAM3
test "$(git rev-parse HEAD)" = 0df83dde1c85c7ab91a0d47de7a29685d046f637 || exit 1
git apply --check "$STUDY/patches/embedded-front-end.patch" && git apply "$STUDY/patches/embedded-front-end.patch"
# Run upstream's build in its source directory with one make job for a small board.
sed -E 's/make -j[0-9]*/make -j1/g' build.sh > build-board.sh
bash -e build-board.sh
export ORB_ROOT="$PWD"
python3 -m venv "$STUDY/.venv"
"$STUDY/.venv/bin/pip" install numpy pyyaml
```

## Run one sequence

Set `EUROC_MH01` to the extracted MH_01_easy directory containing `mav0/` and `TUM_FR1_DESK` to the extracted fr1_desk directory containing `rgb.txt` and `groundtruth.txt`. Use fresh output directories for repeated runs. On the same board repeat with `stock`, `A`, `B` and `AB`; for count controls use `fixed` with `ORB_FIXED_PERCENT=60` or `80`. Stock runs the original feature policy with identical instrumentation. For a parity check also build the clean tag separately.

```bash
export ORB_BUDGET_MODE=AB
run_dir="$PWD/bench-runs/$(date +%Y%m%d-%H%M%S)-$ORB_BUDGET_MODE"
mkdir -p "$run_dir" && cd "$run_dir"
export ORB_FRAME_LOG="$PWD/euroc.csv"
"$ORB_ROOT/Examples/Monocular/mono_euroc" "$ORB_ROOT/Vocabulary/ORBvoc.txt" \
    "$ORB_ROOT/Examples/Monocular/EuRoC.yaml" "${EUROC_MH01:?set sequence root}" \
    "$ORB_ROOT/Examples/Monocular/EuRoC_TimeStamps/MH01.txt" MH01
"$STUDY/.venv/bin/python" "$STUDY/evaluate.py" euroc "$EUROC_MH01" euroc.csv > euroc.json

export ORB_FRAME_LOG="$PWD/tum.csv"
"$ORB_ROOT/Examples/Monocular/mono_tum" "$ORB_ROOT/Vocabulary/ORBvoc.txt" \
    "$ORB_ROOT/Examples/Monocular/TUM1.yaml" "${TUM_FR1_DESK:?set sequence root}"
"$STUDY/.venv/bin/python" "$STUDY/evaluate.py" tum "$TUM_FR1_DESK" tum.csv > tum.json
```

Choose TUM1, TUM2 or TUM3 intrinsics to match the Freiburg camera. EuRoC uses cam0 only. TUM uses RGB only. In both examples, the CSV contains online camera-in-world poses for each input timestamp. The TUM runner's separate keyframe export serves a different purpose. Count all input frames, including initialization and tracking failures. Reconcile row counts against the image manifest before publishing.

## Measurement and ablations

The frame timer starts inside GrabImageMonocular before grayscale conversion and ends after tracking and inlier counting. Its mean includes initialization and failed frames. It excludes decoding, driver queues, artificial playback sleep, buffered-log writing and the final budget update. Measure caller-side end-to-end latency separately when reporting robot response time. Equal logging settings are required for every group.

On Linux, whole-process CPU utilization is 100 times the increase in process CPU seconds divided by elapsed monotonic seconds between the first and last logged frame. Four fully used cores can yield about 400%. CPU-ms/frame uses the same CPU delta divided by the number of intervening frame intervals. Report both values; CPU utilization depends on replay pacing. Keep the same resolution, frame rate, CPU policy, headless mode, input order and process boundaries. Include a separate clean-tag timing check to quantify instrumentation overhead.

| Full-frame experiment | ATE RMSE (m) | 1 s RPE (m / deg) | Mean / P95 frame (ms) | CPU (%) | Coverage |
|---|---:|---:|---:|---:|---:|
| Stock ORB-SLAM3 | pending | pending | pending | pending | pending |
| Fixed budget (60%, 80%, matched count) | pending | pending | pending | pending | pending |
| A only | pending | pending | pending | pending | pending |
| B only | pending | pending | pending | pending | pending |
| A+B | pending | pending | pending | pending | pending |

**Figure 1 placeholder — accuracy versus processing time.** One point per sequence and method, with repeated runs and matched-count controls. Horizontal axis: mean milliseconds per frame. Vertical axis: Sim(3)-aligned ATE in metres.

**Figure 2 placeholder — difficult turn on TurtleBot3.** Align frame time, feature quota, occupied inlier cells, retry counts and tracking state to the camera timestamps. Mark initialization and loss intervals.

## Citation and license

Cite the upstream algorithm and the two benchmarks. Reference this research artifact by its Git commit while a publication is pending. The source patch is distributed under the upstream GPL terms. [License](patches/LICENSE)

1. C. Campos et al., “ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial, and Multimap SLAM,” IEEE Transactions on Robotics, 2021. [Paper](https://doi.org/10.1109/TRO.2021.3075644)
2. M. Burri et al., “The EuRoC Micro Aerial Vehicle Datasets,” International Journal of Robotics Research, 2016. [Dataset and citation](https://projects.asl.ethz.ch/datasets/euroc-mav/)
3. J. Sturm et al., “A Benchmark for the Evaluation of RGB-D SLAM Systems,” IEEE/RSJ IROS, 2012. [Dataset and citation](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)

