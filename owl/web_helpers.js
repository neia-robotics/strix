/* Keep each debounced edit bound to its original camera; serialize saves behind edits. */
function createSettingsQueue(request, report = () => {}) {
  const pending = new Map();
  let chain = Promise.resolve();
  let timer;
  let failure = null;
  function flush() {
    clearTimeout(timer);
    const jobs = [...pending];
    pending.clear();
    chain = chain.then(async () => {
      if (jobs.length) failure = null;
      for (const [url, changes] of jobs) {
        try {
          const response = await request(url, {method: 'POST', body: JSON.stringify(changes)});
          if (!response.ok) {
            const error = new Error(`settings request failed (${response.status})`);
            error.retryable = response.status === 503;
            throw error;
          }
        } catch (error) {
          if (error.retryable !== false) pending.set(url, {...changes, ...pending.get(url)});
          failure = error; report(error);
        }
      }
    });
    return chain;
  }
  return {
    set(url, key, value) {
      pending.set(url, {...pending.get(url), [key]: value});
      clearTimeout(timer);
      timer = setTimeout(flush, 80);
    },
    flush,
    async save(url) {
      await flush();
      if (failure) throw failure;
      const response = await request(url, {method: 'POST', body: '{}'});
      if (!response.ok) throw new Error(`save request failed (${response.status})`);
    }
  };
}
if (typeof module !== 'undefined') module.exports = {createSettingsQueue};
