"""Compose optional task-owned MCP tools around a robot backend."""

from importlib import import_module


TASK_TOOLSETS = {
    "click_bell": "tasks.click_bell.tools:ClickBellTools",
}


def task_tool_names(
    task: str | None,
    *,
    motion_enabled: bool = False,
    task_options: dict | None = None,
) -> tuple[str, ...]:
    if task == "click_bell":
        names = ["click_bell_localize_press_cap"]
        options = task_options if isinstance(task_options, dict) else {}
        preview = options.get("motion") if options else None
        press = options.get("guarded_press") if options else None
        if (
            isinstance(preview, dict) and preview.get("enabled") is True
        ) or (motion_enabled and not options):
            names.append("click_bell_approach_preview")
        if isinstance(press, dict) and press.get("enabled") is True:
            names.extend(
                ["click_bell_guarded_press", "click_bell_confirm_visual_result"]
            )
        return tuple(names)
    return ()


def _toolset(config, backend):
    target = TASK_TOOLSETS.get(config.task)
    if target is None:
        return None
    module, _, attribute = target.partition(":")
    return getattr(import_module(module), attribute)(config, backend=backend)


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
