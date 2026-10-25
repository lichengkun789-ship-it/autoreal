"""DeepSeek Harness provider configuration; secret values stay in private files."""
from pathlib import Path
import re
from urllib.parse import urlsplit

from harness.claude_cli.messages_relay import _private_text


PROVIDERS = ('deepseek-api', 'openrouter', 'custom')
DEFAULT_MODELS = {
    'deepseek-api': 'deepseek-flash',
    'openrouter': 'deepseek/deepseek-v4.1-flash-1017',
    'custom': 'deepseek-flash',
}
MODEL_ID = re.compile(r'[A-Za-z0-9][A-Za-z0-9._:/@-]{0,199}')


class Credential:
    def __init__(self, provider, *, key_file=None, base_url=None):
        if provider not in PROVIDERS:
            raise ValueError(f'Unknown DeepSeek Harness provider {provider!r}; choose from {PROVIDERS}')
        self.provider = provider
        self.key_file = Path(key_file).resolve() if key_file else None
        if not self.key_file or not _private_text(self.key_file).strip():
            raise ValueError(f'{provider} requires a nonempty private key file')
        self.base_url = base_url
        if provider == 'custom':
            parsed = urlsplit(base_url or '')
            if (parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.username
                    or parsed.password or parsed.query or parsed.fragment):
                raise ValueError('custom requires an HTTP(S) base URL without embedded credentials')

    def record(self):
        return {'provider': self.provider, 'key_file': str(self.key_file), 'base_url': self.base_url}

    def environment(self):
        return {'RAX_MODEL_KEY': _private_text(self.key_file).strip()}

    def patches(self, model, effort):
        if not isinstance(model, str) or not MODEL_ID.fullmatch(model):
            raise ValueError('Invalid operator model identifier')
        if self.provider == 'custom':
            return 'rax-private', [{'insert': [{'id': 'rax-private',
                'name': str(Path(__file__).with_name('private_provider.mjs')),
                'config': {'baseURL': self.base_url, 'thinking': 'enabled', 'reasoningEffort': 'max'}}]}]
        if self.provider == 'openrouter':
            route = 'rax-model'
            provider = {'api': 'openai-completions', 'baseURL': 'https://openrouter.ai/api/v1',
                'apiKeyEnv': 'RAX_MODEL_KEY', 'reasoning': effort,
                'models': [{'id': model, 'name': model, 'contextWindow': 1048576,
                            'maxTokens': 393216, 'input': ['text', 'image'],
                            'reasoningEfforts': {'high': 'high', 'xhigh': 'max', 'max': 'max'}}]}
            return route, [{'id': 'llm-pi-ai', 'config': {'providers': {route: provider}}}]
        return 'deepseek-official', [{'id': 'llm-deepseek',
            'config': {'thinking': 'enabled', 'reasoningEffort': 'max', 'apiKeyEnv': 'RAX_MODEL_KEY'}}]
