# Tracking-Guided Feature Extraction for Monocular ORB-SLAM3 on TurtleBot3

## Abstract

On a mobile robot, reducing the number of ORB features can shorten frame processing but can also leave too few well-distributed correspondences after a turn. We investigate two changes to the monocular front end of ORB-SLAM3 for a Raspberry Pi carried by a TurtleBot3. The first selects a feature budget for the next frame using the number and image coverage of current tracking inliers and recent frame processing time. The second limits low-threshold FAST retries in empty detection cells, giving priority to regions with few first-pass candidates. Both changes retain the original descriptor and initialization procedure. We define a paired evaluation using the left camera of EuRoC and the color stream of the TUM RGB-D Benchmark, with separate measurements on ground-robot recordings. The comparison includes stock ORB-SLAM3, fixed feature budgets and each modification in isolation. Absolute and relative pose errors, mean frame time, process-wide CPU utilization and trajectory coverage quantify the trade-off between computation and localization. The central test is whether feedback improves that trade-off at a comparable observed feature count.

**Keywords** — monocular visual SLAM, ORB-SLAM3, feature selection, embedded vision, mobile robots

*Pre-experiment manuscript outline. After the evaluation, replace the protocol-only ending of the abstract with the measured effects, uncertainty bounds, successful-run counts and the principal failure case.*

## 1. Introduction

**1.1 Deployment problem.** A ground robot travelling down a textured corridor may obtain more correspondences than its pose estimator needs. A subsequent turn toward a blank wall changes the value of the next few detections. A single global feature count makes the same allocation in both situations. Quantify this observation on the target board before attributing the bottleneck to feature extraction.

**1.2 Research question.** Can the front end spend less CPU time per input frame than stock ORB-SLAM3, and outperform a fixed-budget configuration at comparable feature count, under a predefined localization-error tolerance?

**1.3 Contributions to substantiate.** (i) A causal, guarded budget controller using quantities already computed during tracking. (ii) A spatially prioritized cap on repeated FAST calls inside the existing extractor. (iii) A board-level decomposition of the resulting change in accuracy, coverage and computational cost. A platform port by itself establishes feasibility only.

## 2. Related Work

**2.1 Feature-based SLAM.** Describe ORB-SLAM3's scale pyramid, cell-wise FAST detection, low-threshold fallback, octree distribution and descriptor computation. Identify the local mapping, place recognition and optimization components inherited from the baseline. [Campos et al.](https://github.com/UZ-SLAMLab/ORB_SLAM3)

**2.2 Feature selection and active matching.** Distinguish selecting high-value correspondences for pose estimation from changing the amount of work performed before descriptors exist. Compare the information available to each method, its decision stage and its selection cost. [Zhang and Vela](https://openaccess.thecvf.com/content_cvpr_2015/papers/Zhang_Good_Features_to_2015_CVPR_paper.pdf) · [Zhao and Vela](https://arxiv.org/abs/2001.00714)

**2.3 Embedded evaluation.** Explain why a desktop timing result, a tracking-thread timer and whole-process utilization answer different questions. Report a recent embedded front-end comparator if a reproducible monocular implementation is available. Limit comparative claims to methods actually tested.

## 3. Methodology

**3.1 Adaptive feature budget (A).** Count non-null, non-outlier map-point associations after tracking. Divide the input image into a 4 × 4 grid and count cells containing at least two such points. If three consecutive frames have at least 80 inliers and ten occupied cells, reduce the per-level quotas by one tier when the five-frame median processing time exceeds 90% of the nominal frame period. The tiers are 100%, 80% and 60% of the original quotas. Allow at least five frames between reductions. Fewer than 50 inliers, fewer than six occupied cells, or a tracking failure restores the full budget for the next frame. Protect initialization and the ten-frame relocalization interval. Record actual descriptor counts because octree rounding can differ from target quotas.

**3.2 Selective FAST retries (B).** Run high-threshold FAST in every candidate cell. During stable tracking, collect empty cells by pyramid level, count first-pass candidates in each of 16 regions and rank empty cells by ascending regional support. Rotate the enumeration order before stable sorting to vary ties over time. Retry at most half of the empty cells per level, rounded up. In other states use the original full-retry path. Count empty cells and actual retries. Include ranking overhead when profiling savings.

**3.3 Integration.** Apply the supplied patch to the pinned release. Each frame consumes only previous feedback when selecting its budget. A and B can be enabled independently. The core extractor still supplies ORB descriptors to the upstream matcher, mapper and loop closer. Inspect the net effect on the entire SLAM process, including changes in keyframe production and back-end optimization.

## 4. Experiments

**4.1 Platform and inputs.** Target a TurtleBot3 fitted with a Raspberry Pi 4B (4 GB), a calibrated monocular camera and active cooling. Record the actual image, board revision, camera, compiler flags, affinity, CPU governor, clock and temperature. Use identical builds, inputs and headless settings in all groups. Replay EuRoC cam0 and TUM RGB-D color images only. EuRoC development sequences are MH_01_easy and V1_01_easy, with the remaining nine reserved for testing. Use TUM fr1_xyz for a pipeline check and hold out fr1_desk, fr1_desk2, fr2_xyz and fr3_long_office_household. Freeze parameters before the holdout runs. Record corridor, turn and low-texture runs on the robot, with at least three separately recorded traversals per route. Without external pose truth, use those runs only for deployment costs and tracking continuity. [EuRoC](https://projects.asl.ethz.ch/datasets/euroc-mav/) · [TUM RGB-D](https://cvg.cit.tum.de/data/datasets/rgbd-dataset)

**4.2 Baselines and ablations.** Run stock, A only, B only and A+B to isolate main effects and their interaction. Add fixed 60%, fixed 80% and a development-calibrated fixed count that matches the observed descriptor count of A+B. Report realized counts on the holdout set. Keep the original initialization quota for the primary fixed-budget comparison by changing only normal-tracking quotas in the patched runner. Use the genuinely unmodified release as a parity check for the instrumented stock mode. Use identical viewer settings for timing comparisons. Five independent process starts per sequence, per method, in randomized blocks. Preserve all failures.

**4.3 Outcomes.** Use frame-level online camera poses for all groups. Align one estimated map to reference positions by a single Sim(3). Report translational ATE RMSE (m) and relative pose error over 1 s (translation RMSE in m, rotation RMSE in degrees). For EuRoC convert body truth to the cam0 frame using the supplied extrinsics; identify the source of the reference orientations, including the dataset's estimated ground truth where applicable. TUM uses the camera reference trajectory. Measure mean and P95 frame time, per-process CPU utilization across all threads and CPU milliseconds per frame. Include input frames without poses in the coverage denominator. A row with multiple valid map IDs is a failed single-map run. The evaluation tool accepts nearest-time matches within 5 ms on EuRoC and 20 ms on TUM, and excludes RPE intervals crossing gaps in associated poses. Report these choices with the results.

**4.4 Decision rule.** Primary targets relative to stock are a reduction in mean frame time and CPU time per frame, a one-sided 95% upper bound of at most 10% for the paired relative ATE increase, a coverage decrease no greater than two percentage points, and no reduction in completion rate. Report P95 against the supplementary 20% improvement target. Compute one statistic per run, take medians across repetitions within sequence, then compare methods using equal sequence weights and a paired bootstrap over sequences (10,000 draws). Tabulate each dataset separately, including error in metres, and give the worst sequence. Only paired completed runs enter the relative-error calculation; publish their denominator and analyze incomplete runs separately. Do not infer non-inferiority from a nonsignificant difference.

**4.5 Results and failure analysis.** Table 1: all four requested metrics by dataset and mode, with coverage and completed runs. Figure 1: ATE against mean processing time, paired by sequence. Figure 2: timeline of inliers, coverage, budget, retries and tracking state around an abrupt turn. Table 2: A/B interaction and CPU-ms/frame against fixed budgets. Inspect thermal throttling, differences in the number of keyframes and startup overhead. If fixed budgets dominate, document the conditions under which feedback adds no value.

## 5. Conclusion

State which module contributed the measured CPU savings, where the loss of accuracy stayed within tolerance, and where tracking failed. Scope conclusions to the tested board, sensor mode and trajectories. Distinguish playback under load from live control-loop latency. Claims about metric navigation or safe autonomous motion require additional scale and control evidence.

## Reviewer questions and responses to substantiate

| Likely question | Response and evidence required |
|---|---|
| Why not simply lower the YAML feature count? | Compare against fixed budgets and publish actual descriptor counts. A YAML-only reduction can also alter initialization. Include a control that holds initialization fixed. If a fixed setting reaches the same operating point, report that result. |
| Which operation actually becomes cheaper? | A changes octree output quotas. B reduces second-pass FAST calls. Report actual counts, extractor-stage time and complete-frame time. A smaller point count alone does not establish lower CPU cost. |
| How do you define acceptable accuracy loss? | Predefine the 10% relative ATE margin and show absolute errors, sequence-level uncertainty, RPE, coverage and failures. Restrict the claim when the upper bound exceeds the margin. |
| Can Sim(3) alignment hide scale drift or resets? | Apply one scale per run, report that scale and 1 s RPE, reject multi-map runs for the primary single-map score, and retain their failure counts. |
| Can dropping difficult frames make the method look faster? | The primary experiment feeds all input frames. Verify the log count against the sequence manifest. Report dropped input frames and acquisition-to-pose delay separately in timestamp-paced robot experiments. |
| How do CPU percentages account for threads and sleep? | Count process-wide CPU seconds over a documented wall interval. One busy core is 100%. Report CPU-ms/frame as well, and keep replay pacing identical. Inspect clocks and thermal throttling. |
| Why do UAV and handheld sequences support a TurtleBot3 claim? | They test accuracy on independent references. Deployment requires measurements on the actual board and repeated ground-robot routes, including sharp turns. Without independent robot truth, limit robot-specific claims to performance and tracking continuity. |
| Is the controller tuned to the test set? | Publish development and holdout manifests, frozen thresholds and git hashes. Sweep thresholds on development sequences and report sensitivity before the final evaluation. |
