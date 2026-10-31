"""Shared operator-side helpers for launching an AutoRAX Agent session."""

from pathlib import Path
import sys

from harness.backends import get as agent_harness


def model_preflight(config, key_file: Path | None) -> dict:
    launcher = agent_harness(config.agent_cli)
    provider = config.agent_provider
    if config.agent_cli == "claude":
        from harness.claude_cli import experiment_login
        from harness.claude_cli.messages_relay import Credential

        binary = launcher.claude_binary()
        if provider == "claude-login":
            if key_file is not None:
                raise ValueError(
                    "claude-login uses the dedicated experiment token; omit --key-file"
                )
            account = experiment_login.check()
            return {
                "agent_cli": config.agent_cli,
                "provider": provider,
                "binary": str(binary),
                "credential": str(experiment_login.token_file()),
                "account": {
                    key: account.get(key) for key in ("email", "organization")
                },
            }
        Credential(provider, key_file=key_file)
        return {
            "agent_cli": config.agent_cli,
            "provider": provider,
            "binary": str(binary),
            "credential": str(key_file.resolve()) if key_file else "environment",
        }

    from harness.deepseek_cli.login import Credential

    binary = launcher.dsh_binary()
    credential_path = key_file or launcher.default_key(provider)
    Credential(provider, key_file=credential_path)
    return {
        "agent_cli": config.agent_cli,
        "provider": provider,
        "binary": str(binary),
        "credential": str(credential_path.resolve()),
    }


def launcher_command(config, config_path: Path, key_file: Path | None) -> list[str]:
    command = [
        sys.executable,
        "-m",
        "harness.launcher",
        "--config",
        str(config_path),
        "--non-interactive",
        "--no-gemini",
    ]
    if key_file is not None:
        command.extend(("--key-file", str(key_file)))
    return command
