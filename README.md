# Real-Time Tuning of Soft Task Priorities with Quadratic Programming

An archival robotics research prototype for adapting competing task objectives online. The method uses constrained quadratic programming to update soft task priorities for a UR5 manipulator operating around changing targets and obstacles.

[![Watch the UR5 demo](./assets/real_time_task_priority_tuning_overview.png)](https://www.youtube.com/watch?v=YBgM1MzcjqU)

**[Watch the real-robot demo](https://www.youtube.com/watch?v=YBgM1MzcjqU)**

**Resources:** [Demo video](https://www.youtube.com/watch?v=YBgM1MzcjqU) · [Unpublished technical report](./real_time_task_priority_tuning_technical_report.pdf)

## Overview

A robot often needs to satisfy several objectives simultaneously, such as reaching a target, maintaining its end-effector orientation, avoiding obstacles, and respecting joint constraints. Fixed task weights cannot adapt when these objectives conflict or the environment changes.

This project formulates online task-priority selection as a constrained quadratic program. At each control step, the optimizer updates the task weights from the current robot state, task objectives, and safety constraints, and combines the individual task controllers into one joint command.

## Highlights

- Online adaptation of soft task priorities with quadratic programming
- Position, orientation, and preferred-configuration objectives
- Obstacle-separation and robot-limit constraints
- ROS and MoveIt integration for a UR5 manipulator
- Evaluation in simulation and real-robot experiments

## Experiments

All experiments use three sub-tasks on a 6-DOF UR5: reach a desired end-effector position, keep a desired end-effector orientation, and hold a preferred joint configuration (in the moving-obstacle experiment the third sub-task is replaced by dynamic-obstacle avoidance).

- **Simulation, static obstacle** — compared against fixed weights and real-time Bayesian Optimization (BO) tuning. Only QP tuning keeps the safety constraint satisfied while both objectives converge to zero; BO slightly violates the bound, produces chattering weights, and solves 25×–170× slower than the QP (report Table I) — too slow for high-frequency control.
- **Real UR5, randomly moving obstacle** — a moving AGV tracked by a Kinect via QR codes. QP tuning avoids the obstacle while realizing both objectives; fixed weights violate the safety bound and fail the mission.
- **Real UR5, randomly moving target** — both objectives converge to zero with the safety constraint within its bound.

Plots and details: see the [technical report](./real_time_task_priority_tuning_technical_report.pdf).

## How to Cite

The report is unpublished, so please cite it together with this repository:

```bibtex
@techreport{zhu2019realtimetuning,
  author = {Zhu, Yiyao and Li, Jian and Wang, Yuquan and Su, Yinyin and Chen, Yongquan},
  title  = {Real-time Tuning Soft Task Priorities with Quadratic Programming},
  year   = {2019},
  type   = {Unpublished technical report},
  note   = {Available at \url{https://github.com/zyy721/Real-time_tuning}}
}
```
