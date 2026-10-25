// Record model request capabilities without prompts, image bytes, headers, or credentials.
import { appendFileSync } from 'node:fs';

const original = globalThis.fetch;
function imageCount(value) {
  if (!value || typeof value !== 'object') return 0;
  if (Array.isArray(value)) return value.reduce((sum, item) => sum + imageCount(item), 0);
  if (['image_url', 'image', 'input_image'].includes(value.type)) return 1;
  return Object.values(value).reduce((sum, item) => sum + imageCount(item), 0);
}
globalThis.fetch = async function(input, init) {
  const url = String(input?.url ?? input);
  if (process.env.RAX_DSH_AUDIT && /\/(chat\/completions|responses|messages)(\?|$)/.test(url)) {
    try {
      const body = JSON.parse(init?.body ?? '{}');
      const messages = body.messages ?? body.input ?? [];
      appendFileSync(process.env.RAX_DSH_AUDIT, JSON.stringify({ at: Date.now()/1000,
        endpoint: url.split('?')[0], model: body.model, reasoning: body.reasoning,
        reasoning_effort: body.reasoning_effort, thinking: body.thinking,
        max_tokens: body.max_tokens ?? body.max_completion_tokens ?? body.max_output_tokens,
        tools: (body.tools ?? []).map(tool => tool.function?.name ?? tool.name),
        image_blocks: imageCount(messages) })+'\n', { mode: 0o600 });
    } catch { /* Audit must never change a provider request. */ }
  }
  return Reflect.apply(original, this, [input, init]);
};
