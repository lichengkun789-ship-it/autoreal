"""Compose manifest-registered task tools around a robot backend."""

from services.task_manifest import adapter_class, adapter_target


def registered_task_adapters() -> dict[tuple[str, str], str]:
    """Return configured adapters for architecture checks and diagnostics."""
    from pathlib import Path
    import json

    result = {}
    services = Path(__file__).resolve().parent
    for path in services.glob("*/task_adapters.json"):
        robot = path.parent.name
        value = json.loads(path.read_text())
        for task, target in value.get("tasks", {}).items():
            result[(robot, task)] = target
    return result


TASK_TOOLSETS = registered_task_adapters()


def task_tool_names(
    task: str | None,
    *,
    robot: str,
    task_options: dict | None = None,
) -> tuple[str, ...]:
    if task is None:
        return ()
    adapter = adapter_class(robot, task)
    if adapter is None:
        return ()
    description = adapter.describe_options(task_options or {})
    return tuple(description["tools"])


def task_motion_tool_names(
    task: str | None,
    *,
    robot: str,
    task_options: dict | None = None,
) -> tuple[str, ...]:
    if task is None:
        return ()
    adapter = adapter_class(robot, task)
    if adapter is None:
        return ()
    description = adapter.describe_options(task_options or {})
    return tuple(description["motion_tools"])


def _toolset(config, backend):
    if adapter_target(config.robot, config.task) is None:
        return None
    return adapter_class(config.robot, config.task)(config, backend=backend)


class BackendWithTaskTools:
    def __init__(self, backend, task_tools):
        self.backend = backend
        self.task_tools = task_tools
        self._base_names = {tool["name"] for tool in backend.tools()}
        task_names = {tool["name"] for tool in task_tools.tools()}
        duplicate = self._base_names & task_names
        if duplicate:
            raise ValueError(f"task tools duplicate backend tools: {sorted(duplicate)}")
        self.mutating = frozenset(getattr(backend, "mutating", ())) | frozenset(
            getattr(task_tools, "mutating", ())
        )

    def tools(self):
        return [*self.backend.tools(), *self.task_tools.tools()]

    def validate(self, name, arguments):
        owner = self.backend if name in self._base_names else self.task_tools
        return owner.validate(name, arguments)

    def call(self, name, arguments):
        owner = self.backend if name in self._base_names else self.task_tools
        return owner.call(name, arguments)

    def status(self):
        value = self.backend.status()
        status = getattr(self.task_tools, "status", None)
        if status is not None:
            value["task_tools"] = status()
        return value

    def close(self):
        try:
            close = getattr(self.task_tools, "close", None)
            if close is not None:
                close()
        finally:
            self.backend.close()


def compose_task_tools(config, backend):
    try:
        tools = _toolset(config, backend)
    except BaseException:
        backend.close()
        raise
    return backend if tools is None else BackendWithTaskTools(backend, tools)
