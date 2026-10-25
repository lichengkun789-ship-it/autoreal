// Use DSH's native DeepSeek protocol with an operator-selected compatible endpoint.
import { pathToFileURL } from 'node:url';
const root = `${process.env.RAX_DSH_INSTALL}/node_modules/@deepseek-ai`;
const protocol = await import(pathToFileURL(`${root}/dsh-llm-deepseek/lib/index.js`).href);
export const Config = protocol.Config;
export const name = 'rax-private-deepseek';
export const inject = ['llm'];

export function apply(ctx, config) {
  const options = () => protocol.resolveAdapterOptions(protocol.plainOptions(config));
  protocol.registerDeepSeekProvider(ctx, 'rax-private', {
    options,
    providerName: 'Private DeepSeek',
    resolveAuth: async () => {
      const key = process.env.RAX_MODEL_KEY;
      if (!key) throw new Error('Private DeepSeek credential is missing');
      return { headers: { authorization: `Bearer ${key}` } };
    },
    discoverModels: async provider => options().models.map(model => protocol.catalogModelInfo(provider, model)),
  });
}
