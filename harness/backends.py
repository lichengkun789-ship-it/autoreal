"""Agent harness registry; implementations are imported only when selected."""
from importlib import import_module


NAMES = ('claude', 'dsh')
PROVIDERS = {
    'claude': ('claude-login', 'anthropic', 'openrouter'),
    'dsh': ('deepseek-api', 'openrouter', 'custom'),
}
MODULES = {
    'claude': 'harness.claude_cli.launcher',
    'dsh': 'harness.deepseek_cli.launcher',
}


def get(name):
    try:
        module = MODULES[name]
    except KeyError:
        raise ValueError(f'Unknown agent harness {name!r}; choose from {NAMES}') from None
    return import_module(module)
