"""Robot backends behind the agent MCP. The frontend never imports a backend directly.

A backend is constructed once per session by the frontend process (the only robot
owner) and implements:

    tools() -> list[dict]            MCP tool definitions (name, description, inputSchema)
    call(name, arguments) -> (value, images)
                                     value: JSON-safe dict; images: MCP image content
                                     blocks {'type': 'image', 'mimeType', 'data'(base64)}
    status() -> dict                 live robot summary merged into the `status` tool
    close()                          release hardware; must be safe to call twice

Optional: ``mutating`` (set of tool names that can move hardware; a failure in one
marks control uncertain) and ``validate(name, arguments)`` (raise before any motion).
"""
from importlib import import_module

from services.task_tools import compose_task_tools

# registry key -> "module:Class"; the module is imported only when selected.
BACKENDS = {
    'mock': 'services.mock_backend:MockBackend',
    'piper': 'services.piper.backend:PiperBackend',
}


def backend_class(name):
    try:
        module, _, attribute = BACKENDS[name].partition(':')
    except KeyError:
        raise ValueError(f"Unknown robot backend {name!r}; choose one of {', '.join(sorted(BACKENDS))}") from None
    return getattr(import_module(module), attribute)


def make_backend(config):
    """The configured robot's backend, constructed with the operator configuration."""
    return compose_task_tools(config, backend_class(config.robot)(config))
