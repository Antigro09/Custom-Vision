#!/usr/bin/env python3
"""Bounded headless Chromium QA against an already running localhost preview.

Uses real producer golden fixtures through the actual browser replay file input.
Synthetic identity/configuration variations are recorded in qa-inputs; no camera,
NT client, server write, remote browser, deployment or hardware is used. One
browser runs with GPU rendering disabled. All screenshots and receipts are local.
"""
from __future__ import annotations

import argparse
import ast
import copy
import hashlib
import importlib.metadata
import json
from pathlib import Path
import sys
import traceback
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]

# Actual production implementations are wrapped only inside the QA browser to
# inspect their state. Polling timers alone are throttled from 500 to 1200 ms so
# Playwright can observe its required 500 ms network-idle interval. Rendering and
# expiry timers are unchanged. Neither wrapper is written into product assets.
INSTRUMENT = """
globalThis.__fieldQa = {stores: [], renderers: [], scene: null};
const qaTimer = globalThis.setTimeout;
globalThis.setTimeout = (fn, ms, ...args) => qaTimer(fn,
  ['pollStatus','pollPreview'].includes(fn?.name) ? Math.max(ms,1200) : ms, ...args);
for (const [name, method, collection] of [
  ['CVFieldModel','createStore','stores'], ['CVFieldRenderer','createRenderer','renderers']]) {
  Object.defineProperty(globalThis,name,{configurable:true,set(api) {
    const create = api[method];
    api[method] = (...args) => {
      const result = create(...args); globalThis.__fieldQa[collection].push(result);
      if (method === 'createRenderer') {
        const setScene = result.setScene;
        result.setScene = scene => {
          globalThis.__fieldQa.scene = {field:scene.field, tags:scene.tags, geometry:scene.geometry,
            sources:structuredClone(scene.sources), hasBackground:!!scene.backgroundImage};
          return setScene(scene);
        };
      }
      return result;
    };
    Object.defineProperty(globalThis,name,{value:api,writable:true,configurable:true});
  }});
}
"""


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fixture(name):
    return json.loads((ROOT / 'protocol' / 'fixtures' / f'{name}.json').read_text())


def synthetic_mount():
    # Read the fixture generator's literal public mount without importing any
    # perception/native dependency or repeating an undocumented measured mount.
    tree = ast.parse((ROOT / 'tools' / 'generate_protocol_fixtures.py').read_text())
    for statement in tree.body:
        if isinstance(statement, ast.Assign) and any(isinstance(t, ast.Name) and t.id == 'MOUNT' for t in statement.targets):
            return ast.literal_eval(statement.value)
    raise AssertionError('Synthetic producer fixture mount literal not found')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:5844')
    parser.add_argument('--chromium', required=True, type=Path, help='Existing local Chromium executable; no browser download')
    parser.add_argument('--output', type=Path, default=ROOT / 'data' / 'field-visualization')
    parser.add_argument('--map', type=Path, help='Optional shared synthetic field-map fixture')
    parser.add_argument('--image', type=Path, help='Its matching local image')
    args = parser.parse_args()
    parsed = urlsplit(args.url)
    if parsed.scheme != 'http' or parsed.hostname not in {'127.0.0.1', 'localhost', '::1'} or parsed.username or parsed.password:
        parser.error('--url must be an explicit unauthenticated HTTP localhost preview')
    if not args.chromium.is_file():
        parser.error('--chromium must name an existing executable')
    if bool(args.map) != bool(args.image):
        parser.error('--map and --image must be supplied together')
    for path in [args.map, args.image]:
        if path and not path.is_file():
            parser.error(f'Missing input: {path}')
    args.output.mkdir(parents=True, exist_ok=True)
    inputs = args.output / 'qa-inputs'
    inputs.mkdir(exist_ok=True)
    receipt = {'status': 'running', 'preview': args.url, 'browser': 'headless Chromium, GPU disabled, renderer-process-limit=1',
               'polling_note': 'QA-only named network poll timers throttled to 1200 ms; render/expiry timers unchanged',
               'checks': [], 'screenshots': [], 'console': [], 'page_errors': [], 'requests': [], 'responses': [],
               'request_failures': [], 'external_requests': [], 'server_writes': [], 'fixture_hashes': {}, 'input_hashes': {}}
    source_names = ['single_tag', 'camera_only_localization', 'same_frame_watchdog', 'repeated_invalidation', 'new_boot', 'unsynchronized_time']
    for name in source_names:
        receipt['fixture_hashes'][f'protocol/fixtures/{name}.json'] = digest(ROOT / 'protocol' / 'fixtures' / f'{name}.json')
    layout = json.loads((ROOT / 'protocol' / 'fixture-layouts.json').read_text())['layouts']['scene']
    mount = synthetic_mount()
    control = {'field_view': None}
    browser = None
    page = None

    def check(name, condition, evidence=None):
        assert condition, f'{name}: {evidence}'
        receipt['checks'].append({'name': name, 'passed': True, 'evidence': evidence})
        print(f'PASS {name}', flush=True)

    def write_input(name, packets, *, field=layout, mounts=None):
        path = inputs / f'{name}.json'
        data = {'packets': packets, 'field_layout': field, 'mounts': mounts if mounts is not None else {'front_tags': mount}}
        path.write_text(json.dumps(data, allow_nan=False, indent=2) + '\n')
        receipt['input_hashes'][f'qa-inputs/{path.name}'] = digest(path)
        return path

    def capture(name):
        page.screenshot(path=str(args.output / name), full_page=True)
        receipt['screenshots'].append({'file': name, 'sha256': digest(args.output / name)})
        if name in {'2d.png','3d.png'}:
            panel=name.replace('.png','-panel.png')
            page.locator('.field-panel').screenshot(path=str(args.output/panel))
            receipt['screenshots'].append({'file':panel,'sha256':digest(args.output/panel)})

    def text(selector):
        return page.locator(selector).inner_text()

    def wait_text(selector, value):
        page.wait_for_function('([selector,value]) => document.querySelector(selector).textContent.includes(value)', arg=[selector, value], timeout=5000)

    def snapshot():
        return page.evaluate('globalThis.__fieldQa.stores[1].snapshot(performance.now())')

    def load(path):
        page.locator('#field-replay-import').set_input_files(str(path))
        page.wait_for_function("() => document.querySelector('#field-replay-status').textContent.startsWith('REPLAY')", timeout=5000)
        page.wait_for_timeout(100)

    try:
        from playwright.sync_api import sync_playwright
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(headless=True, executable_path=str(args.chromium),
                args=['--disable-gpu', '--renderer-process-limit=1'])
            context = browser.new_context(viewport={'width': 1440, 'height': 1100}, device_scale_factor=1)
            origin = (parsed.scheme, parsed.hostname, parsed.port or 80)

            def route(request_route):
                request = request_route.request
                target = urlsplit(request.url)
                if (target.scheme, target.hostname, target.port or 80) != origin:
                    receipt['external_requests'].append({'url': request.url, 'method': request.method})
                    request_route.abort('blockedbyclient')
                elif request.method != 'GET':
                    receipt['server_writes'].append({'url': request.url, 'method': request.method})
                    request_route.abort('blockedbyclient')
                elif target.path == '/api/field-view' and control['field_view'] is not None:
                    if control['field_view'] == 'offline':
                        request_route.abort('connectionrefused')
                    else:
                        request_route.fulfill(status=200, content_type='application/json', body=json.dumps(control['field_view']))
                else:
                    request_route.continue_()

            context.route('**/*', route)
            page = context.new_page()
            page.add_init_script(INSTRUMENT)
            page.on('console', lambda message: receipt['console'].append({'type': message.type, 'text': message.text}) if len(receipt['console']) < 256 else None)
            page.on('pageerror', lambda error: receipt['page_errors'].append(str(error)))
            page.on('request', lambda request: receipt['requests'].append({'url': request.url, 'method': request.method}) if len(receipt['requests']) < 512 else None)
            page.on('response', lambda response: receipt['responses'].append({'url': response.url, 'status': response.status}) if len(receipt['responses']) < 512 else None)
            page.on('requestfailed', lambda request: receipt['request_failures'].append({'url': request.url, 'failure': request.failure}) if len(receipt['request_failures']) < 256 else None)
            page.goto(args.url, wait_until='networkidle', timeout=15000)
            page.wait_for_function('() => !!(globalThis.CVFieldDashboard && globalThis.CVFieldScene && globalThis.__fieldQa.renderers.length)', timeout=5000)
            receipt['reconnaissance_dom'] = page.locator('.field-panel').inner_text()
            capture('recon.png')
            check('actual dashboard modules and fixed coordinates load', 'FIXED NWU' in receipt['reconnaissance_dom'])
            check('preview setup is read-only', page.locator('#save').is_disabled())
            page.locator('.field-import summary').click()

            single_file = write_input('single-tag', [fixture('single_tag')])
            load(single_file)
            wait_text('#field-state', 'valid')
            check('replay file input displays distinct real fixture poses', 'X 1.61' in text('#field-robot-pose') and 'X 2.00' in text('#field-camera-pose'))
            check('synthetic replay label is explicit', 'REPLAY' in text('#field-source-label') and 'synthetic fixture' in text('#field-source-label'))

            if args.map:
                page.locator('#field-map-import').set_input_files(str(args.map))
                wait_text('#field-geometry-note', 'does not match')
                check('mismatched map dimensions suppress geometry', page.evaluate('globalThis.__fieldQa.scene.geometry===null'))
                declared = json.loads(args.map.read_text())['map']
                matching_layout = copy.deepcopy(layout)
                matching_layout['field'] = {'length': declared['width_m'], 'width': declared['height_m']}
                matching_layout['tags'] = [copy.deepcopy(layout['tags'][0])]
                load(write_input('matching-shared-map', [fixture('single_tag')], field=matching_layout))
                page.locator('#field-image-import').set_input_files(str(args.image))
                wait_text('#field-map-status', 'Matching local picture verified')
                check('exact local image hash and calibrated map bind', page.evaluate('globalThis.__fieldQa.scene.hasBackground===true'))
                receipt['input_hashes']['shared-map'] = digest(args.map)
                receipt['input_hashes']['shared-image'] = digest(args.image)
                check('unknown obstacle height stays unknown', page.evaluate("globalThis.__fieldQa.scene.geometry.obstacles.some(o => o.height_m===null && o.base_z_m===null)"))

            page.locator('#field-mode-2d').click()
            check('2D mode button updates rendered mode', page.locator('#field-mode-2d').get_attribute('aria-pressed') == 'true')
            capture('2d.png')
            page.locator('#field-mode-3d').click()
            page.wait_for_timeout(100)
            check('3D mode uses actual renderer', page.evaluate("globalThis.__fieldQa.renderers[0].getView().mode==='3d'"))
            home_view = page.evaluate('globalThis.__fieldQa.renderers[0].getView()')
            canvas = page.locator('#field-canvas')
            box = canvas.bounding_box()
            page.mouse.move(box['x'] + box['width'] * .6, box['y'] + box['height'] * .5)
            page.mouse.down(); page.mouse.move(box['x'] + box['width'] * .7, box['y'] + box['height'] * .6, steps=5); page.mouse.up()
            orbited = page.evaluate('globalThis.__fieldQa.renderers[0].getView()')
            check('3D pointer orbit changes azimuth and elevation', orbited['azimuth'] != home_view['azimuth'] and orbited['elevation'] != home_view['elevation'])
            page.locator('#field-home').click()
            check('Fit view restores deterministic home', page.evaluate('globalThis.__fieldQa.renderers[0].getView()') == home_view)
            canvas.press('ArrowLeft')
            check('keyboard orbit changes actual renderer view', page.evaluate('globalThis.__fieldQa.renderers[0].getView().azimuth') != home_view['azimuth'])
            canvas.press('Home')
            check('keyboard Home restores view', page.evaluate('globalThis.__fieldQa.renderers[0].getView()') == home_view)
            capture('3d.png')

            load(write_input('camera-only', [fixture('camera_only_localization')], mounts={'front_tags': None}))
            wait_text('#field-camera-state', 'Estimated')
            check('missing mount preserves camera and explicitly withholds robot', text('#field-robot-state') == 'Unavailable' and 'no_robot_to_camera' in text('#field-robot-pose'))
            check('missing configured mount is explicit', 'Missing' in text('#field-mount'))
            capture('camera-only.png')

            load(write_input('watchdog-and-reboot', [fixture(name) for name in ['single_tag', 'same_frame_watchdog', 'repeated_invalidation', 'new_boot']]))
            page.locator('#field-replay-next').click(); wait_text('#field-state', 'invalid')
            view = snapshot()['sources'][0]
            check('same-frame watchdog clears actual current poses', view['cameraPose'] is None and view['robotPose'] is None and len(view['cameraTrail']) == 1)
            page.locator('#field-replay-next').click()
            check('repeated invalidation stays cleared', snapshot()['sources'][0]['cameraPose'] is None)
            page.locator('#field-replay-next').click(); wait_text('#field-state', 'valid')
            check('new boot resets trail scope', len(snapshot()['sources'][0]['cameraTrail']) == 1 and snapshot()['sources'][0]['bootId'] == 'fixture-boot-b')

            recovered = fixture('single_tag'); recovered['packet_seq'] = 2; recovered['frame_id'] += 1
            load(write_input('invalid-gap-recovery', [fixture('single_tag'), fixture('same_frame_watchdog'), recovered]))
            page.locator('#field-replay-next').click(); page.locator('#field-replay-next').click()
            check('actual browser trails break across invalid intervals', [point['break'] for point in snapshot()['sources'][0]['cameraTrail']] == [True, True])

            same_frame = fixture('single_tag'); same_frame['packet_seq'] = 1
            load(write_input('same-frame-republication', [fixture('single_tag'), same_frame]))
            page.locator('#field-replay-next').click()
            check('same capture frame adds no duplicate browser trail point', len(snapshot()['sources'][0]['cameraTrail']) == 1)

            changed = fixture('single_tag'); changed['packet_seq'] = 1; changed['frame_id'] += 1
            changed['calibration_revision'] = 'browser-qa-synthetic-revision-change'
            load(write_input('revision-change', [fixture('single_tag'), changed]))
            page.locator('#field-replay-next').click()
            check('revision changes reset historical trails', len(snapshot()['sources'][0]['cameraTrail']) == 1)
            page.locator('#field-clear-trail').click()
            check('clear-trails button preserves current estimate', not snapshot()['sources'][0]['cameraTrail'] and snapshot()['sources'][0]['camera']['status'] == 'valid')

            malformed = fixture('single_tag')
            malformed['localization']['field_to_camera']['rotation_quaternion_wxyz'] = [2, 0, 0, 0]
            load(write_input('malformed-quaternion', [malformed]))
            wait_text('#field-state', 'invalid')
            check('malformed WXYZ is rejected without rendering a pose', snapshot()['sources'][0]['cameraPose'] is None and snapshot()['sources'][0]['robotPose'] is None)

            second = fixture('camera_only_localization'); second['pipeline'] = 'rear_fixture_tags'
            load(write_input('two-independent-cameras', [fixture('single_tag'), second], mounts={'front_tags': mount, 'rear_fixture_tags': None}))
            page.locator('#field-replay-next').click(); page.locator('#field-show-all').check()
            check('two camera sources remain independent without fusion', page.evaluate('globalThis.__fieldQa.scene.sources.length') == 2 and len(snapshot()['sources']) == 2)
            page.locator('#field-show-all').uncheck()
            check('selected-camera mode filters renderer only', page.evaluate('globalThis.__fieldQa.scene.sources.length') == 1 and len(snapshot()['sources']) == 2)

            load(single_file)
            draw_start = page.evaluate('({time:performance.now(),count:globalThis.__fieldQa.renderers[0].getStats().drawCount})')
            page.wait_for_timeout(1800)
            wait_text('#field-state', 'stale')
            check('unchanged replay expires despite browser rendering updates', snapshot()['sources'][0]['camera']['status'] == 'stale' and 'historical only' in text('#field-camera-pose'))
            draw_end = page.evaluate('({time:performance.now(),count:globalThis.__fieldQa.renderers[0].getStats().drawCount})')
            check('browser canvas redraw count stays within 15 FPS bound',
                  draw_end['count'] - draw_start['count'] <= (draw_end['time'] - draw_start['time']) * .015 + 2,
                  {'draws': draw_end['count'] - draw_start['count'], 'elapsed_ms': draw_end['time'] - draw_start['time']})
            capture('stale.png')

            no_nt = fixture('unsynchronized_time'); no_nt.pop('packet_seq', None); no_nt.pop('protocol_profile', None)
            no_nt['boot_id'] = 'browser-qa-disabled-publisher'
            control['field_view'] = {'version': 1, 'results': {'front_tags': no_nt}, 'receipt_age_ms': {'front_tags': 0}, 'field_layout': layout, 'mounts': {'front_tags': mount}}
            page.locator('#field-live').click()
            wait_text('#field-sync', 'unavailable')
            page.wait_for_function("() => globalThis.__fieldQa.stores[0].snapshot(performance.now()).sources[0].bootId==='browser-qa-disabled-publisher'", timeout=5000)
            live_view = page.evaluate('globalThis.__fieldQa.stores[0].snapshot(performance.now()).sources[0]')
            check('disabled NT publisher missing packet_seq still displays geometry', live_view['packetSeq'] is None and live_view['cameraPose'] is not None)
            page.wait_for_timeout(1800); wait_text('#field-state', 'stale')
            check('unchanged server poll cannot refresh unsequenced pose', 'historical only' in text('#field-camera-pose'))
            old_source = copy.deepcopy(no_nt); old_source['boot_id'] = 'browser-qa-already-old-server-packet'
            control['field_view'] = {'version': 1, 'results': {'front_tags': old_source},
                'receipt_age_ms': {'front_tags': 3000}, 'field_layout': layout, 'mounts': {'front_tags': mount}}
            page.wait_for_function("() => globalThis.__fieldQa.stores[0].snapshot(performance.now()).sources[0].bootId==='browser-qa-already-old-server-packet'", timeout=5000)
            old_view = page.evaluate('globalThis.__fieldQa.stores[0].snapshot(performance.now()).sources[0]')
            check('first already-old server packet is immediately stale', old_view['receiptAgeMs'] >= 3000 and old_view['camera']['status'] == 'stale')
            control['field_view'] = 'offline'
            wait_text('#connection', 'Unavailable')
            check('offline dashboard labels retained pose stale', text('#field-state') == 'stale')

            load(single_file)
            page.set_viewport_size({'width': 390, 'height': 844})
            page.wait_for_timeout(100)
            check('mobile page fits viewport without horizontal overflow', page.evaluate('document.documentElement.scrollWidth<=innerWidth+1'))
            capture('mobile.png')
            check('browser has no unhandled JavaScript exceptions', not receipt['page_errors'], receipt['page_errors'])
            check('all requests remain localhost and read-only', not receipt['external_requests'] and not receipt['server_writes'])
            receipt['renderer_stats'] = page.evaluate('globalThis.__fieldQa.renderers[0].getStats()')
            receipt['status'] = 'passed'
            browser.close(); browser = None
    except Exception:
        receipt['status'] = 'failed'; receipt['exception'] = traceback.format_exc()
        if page:
            try: capture('failure.png')
            except Exception: pass
        traceback.print_exc()
    finally:
        if browser:
            try: browser.close()
            except Exception: pass
        # Producer/source inputs must remain byte-identical after browser QA.
        for name, old_hash in receipt['fixture_hashes'].items():
            if digest(ROOT / name) != old_hash:
                receipt['status'] = 'failed'; receipt['fixture_mutation'] = name
        (args.output / 'browser-qa.json').write_text(json.dumps(receipt, allow_nan=False, indent=2) + '\n')
    print(f"{receipt['status'].upper()}: {len(receipt['checks'])} checks; {args.output / 'browser-qa.json'}", flush=True)
    return 0 if receipt['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
