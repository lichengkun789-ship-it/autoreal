# Upstream relationship

AutoRAX is derived from the RAX `real/franka` branch because that branch provides the
real-robot harness, backend contract, operator arming, sandbox, audit log, and workspace
generation used as the adaptation reference.

- Upstream: <https://github.com/robot-agent-exploration/rax>
- Local development branch: `real/piper`
- Robot-specific implementation: `services/piper/`

Franka hardware code is not part of the AutoRAX working tree. Upstream changes are
reviewed and ported intentionally instead of merging robot-specific implementations.
