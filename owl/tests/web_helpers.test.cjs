const {test} = require('node:test');
const assert = require('node:assert/strict');
const {createSettingsQueue} = require('../web_helpers.js');
test('edits remain bound to cameras and save waits for all edits', async () => {
  const calls = [];
  const queue = createSettingsQueue(async (url, options) => {
    await new Promise(resolve => setTimeout(resolve, 5));
    calls.push([url, JSON.parse(options.body)]); return {ok: true};
  });
  queue.set('camera1/api/set', 'exposure', 20);
  queue.set('camera2/api/set', 'gain', 4);
  await queue.save('camera2/api/save');
  assert.deepEqual(calls, [['camera1/api/set', {exposure:20}], ['camera2/api/set', {gain:4}], ['camera2/api/save', {}]]);
});
test('failed edits prevent saving stale settings', async () => {
  const calls = [];
  const queue = createSettingsQueue(async url => {calls.push(url); return {ok:false, status:503};});
  queue.set('/api/set', 'gain', 3);
  await assert.rejects(queue.save('/api/save'));
  assert.deepEqual(calls, ['/api/set']);
});
test('save retries a failed edit instead of silently losing it', async () => {
  let fail = true; const calls = [];
  const queue = createSettingsQueue(async (url, options) => {
    calls.push([url, JSON.parse(options.body)]); return {ok:!fail, status:503};
  });
  queue.set('/api/set', 'gain', 3);
  await assert.rejects(queue.save('/api/save'));
  fail = false;
  await queue.save('/api/save');
  assert.deepEqual(calls, [['/api/set', {gain:3}], ['/api/set', {gain:3}], ['/api/save', {}]]);
});
test('a rejected value does not poison subsequent edits', async () => {
  const calls = [];
  const queue = createSettingsQueue(async (url, options) => {
    const values = JSON.parse(options.body); calls.push(values);
    return {ok: !('invalid' in values), status:400};
  });
  queue.set('/api/set', 'invalid', 0);
  await assert.rejects(queue.save('/api/save'));
  queue.set('/api/set', 'gain', 4);
  await queue.save('/api/save');
  assert.deepEqual(calls, [{invalid:0}, {gain:4}, {}]);
});

test('diagnostics retain styling and treat names as text', async () => {
  const vm = require('node:vm');
  const html = require('node:fs').readFileSync(require('node:path').join(__dirname, '../web_ui.html'), 'utf8');
  const source = html.slice(html.indexOf('async function pollSystem()'), html.indexOf('// ---- cameras page:'));
  class Element {
    constructor() { this.children = []; }
    append(...children) { this.children.push(...children); }
    replaceChildren(...children) { this.children = children; }
  }
  const elements = new Map();
  const $ = id => { if (!elements.has(id)) elements.set(id, new Element()); return elements.get(id); };
  const data = {temps:[{name:'<img onerror=bad>',c:91}], disk:{used:1,total:2},
    owl:{nt:{mode:'client',connected:true,peers:[{id:'<script>bad</script>'}]},
      cameras:[{name:'<img onerror=bad>',port:1181,alive:true,pid:1,restarts:2,cpu_fallback:true}]}};
  const context = { $, document:{createElement:()=>new Element()}, api:x=>x, originFor:()=>'',
    dur:String, gb:String, fmt:String, meter:()=>{}, yes:()=>'',
    fetch:async url=>({json:async()=>url==='/api/system'?data:{stats:{},calib_info:null}})};
  vm.createContext(context); vm.runInContext(source, context); await context.pollSystem();
  assert.equal($('sys-temps').children[0].children[1].className, 'bad-text');
  assert.equal($('nt-conn').children[1].textContent, ' (<script>bad</script>)');
  const cells = $('sys-cams').children[0].children;
  assert.equal(cells[0].children[0].textContent, '<img onerror=bad>');
  assert.equal(cells[2].children[0].className, 'badge good');
  assert.equal(cells[3].children[0].className, 'warn-text');
  assert.match(cells[4].children[0].textContent, /CPU fallback/);
  assert.equal(cells[7].children[0].className, 'badge bad');
  data.owl.cameras = []; await context.pollSystem();
  assert.match($('sys-cams').children[0].children[0].textContent, /waiting for the main process/);
});
