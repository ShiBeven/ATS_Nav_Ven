# Codex Workspace Guide

## Scope

This repository is a ROS 2 Humble workspace with multiple packages. Most tasks should stay inside one package unless the request is explicitly cross-package.

Primary packages:

- `src/sp_vision25`: vision, tracking, calibration, serial integration
- `src/pb2025_sentry_behavior`: behavior tree logic and decision plugins
- `src/pb2025_sentry_nav`: navigation, localization, nav plugins
- `src/pb2025_sentry_bringup`: launch entrypoints
- `src/interfaces/pb_rm_interfaces`: custom ROS interfaces

Generated or noisy directories:

- `build/`
- `install/`
- `log/`
- `logs/`
- `records/`

Do not search those directories unless the user explicitly asks for logs or generated artifacts.

## Default Search Strategy

1. Start from the narrowest package path possible.
2. Read the nearest `README.md` or `CONTRIBUTING.md` before making assumptions.
3. Prefer package-level build or test commands over workspace-wide rebuilds.
4. Treat root shell scripts as legacy helpers unless the task is specifically about them.

## Task Intake Contract

Ask the user to provide tasks in this shape when possible:

- Goal: exactly what should change
- Scope: allowed package or file range
- Constraints: API, style, performance, hardware, or dependency limits
- Validation: the command or behavior that proves success
- Stop condition: what is out of scope

If the request mixes multiple goals, split them and complete one objective per session.

## Context Control

- One conversation should map to one objective.
- If the objective changes, start a new conversation with a short handoff summary instead of reusing a long thread.
- Do not paste large logs or full source files unless a specific excerpt is needed.
- Prefer path references, error signatures, and reproduction steps over broad background dumps.

## Verification Policy

- For `sp_vision25`, prefer package-scoped build/test first.
- For behavior tree or nav changes, verify the touched package before attempting workspace-wide validation.
- If verification requires unavailable hardware, validate what can be checked statically and state the remaining gap.
