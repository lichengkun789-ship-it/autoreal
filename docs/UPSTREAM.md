# Upstream relationship

AutoRAX is derived from the RAX `real/franka` branch because that branch provides the
real-robot harness, backend contract, operator arming, sandbox, audit log, and workspace
generation used as the adaptation reference.

- Upstream: <https://github.com/robot-agent-exploration/rax>
- Local development branch: `main`
- Robot-specific implementation: `services/piper/`

Franka hardware code is not part of the AutoRAX working tree. Upstream changes are
reviewed and ported intentionally instead of merging robot-specific implementations.

The native DeepSeek Harness adapter is a focused port from the current upstream
simulation-branch multi-harness implementation. AutoRAX keeps its real-robot session
lifecycle and Piper safety backend; simulator scheduling, formal evaluation, and
benchmark-specific code were not copied.

## Hardware reference policy

The RAX `real/franka` branch is the architectural reference for separating camera
ownership, backend tools, generated workspaces, and operator gates. Its Apache-2.0
license is retained by this repository.

The local `challenge_deploy` checkout was inspected only to understand the installed
Piper and RealSense hardware behavior. As of 2026-10-25 that checkout did not contain a
license or notice granting source-code reuse. AutoRAX therefore neither copies its
implementation nor imports it at runtime. The Piper camera adapter in this repository
is an independent implementation against the public `pyrealsense2` API, and AutoRAX
must continue to run when `challenge_ws` is absent.
