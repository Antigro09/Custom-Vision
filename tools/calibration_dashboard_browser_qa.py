#!/usr/bin/env python3
"""One bounded browser QA run against an already running offline desktop app.

Imports three tiny synthetic board PNGs and runs actual selection only. Candidate,
cancel and failed-job display cases are explicitly mocked UI fixtures, never a
native solve, computed intrinsics or hardware measurement. The export uses a
temporary, explicitly invalid synthetic artifact in the running server's ignored
QA store and its real HTTP download route; the fixture is removed afterward.
No runtime activation,
configuration write, camera, NT, external network or GPU workload is permitted.
"""
from __future__ import annotations
import argparse
import hashlib
import importlib.metadata
import json
from pathlib import Path
import shutil
import struct
import sys
import traceback
from urllib.parse import urlsplit
from urllib.request import urlopen
import uuid
import zlib

ROOT=Path(__file__).resolve().parents[1]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def install_export_fixture(store, candidate_id, job_id):
    """Seed only owned QA records; no inputs, computed intrinsics or solver job."""
    store=store.resolve()
    if not store.is_relative_to(ROOT/'data') or not all((store/name).is_dir() for name in ('jobs','candidates')):
        raise ValueError('--fixture-store must be the running server\'s existing ignored QA store inside this checkout/data')
    candidate_dir=store/'candidates'/candidate_id;job_dir=store/'jobs'/job_id
    # Random IDs plus exclusive directory creation never overwrite saved work.
    candidate_dir.mkdir()
    try:
        job_dir.mkdir()
    except Exception:
        candidate_dir.rmdir();raise
    payload={'synthetic_ui_fixture':True,'valid_calibration':False}
    body=(json.dumps(payload,sort_keys=True)+'\n').encode()
    # A separate synthetic session identity prevents these UI-only records from
    # appearing as jobs/candidates of the genuinely selected source session.
    synthetic_session=uuid.uuid4().hex
    candidate={'candidate_id':candidate_id,'session_id':synthetic_session,'job_id':job_id,
        'directory':str(candidate_dir.relative_to(store)),'solver':'synthetic_ui_fixture',
        'quality_status':'SYNTHETIC UI FIXTURE — not a computed calibration',
        'created_utc':'2026-10-08T00:00:00Z','calibration':payload,
        'report':{'status':'synthetic_ui_fixture_not_a_calibration','issues':['No calibration was computed.']},
        'artifacts':[{'name':'synthetic-ui-only.json','content_type':'application/json','size_bytes':len(body)}]}
    try:
        (job_dir/'job.json').write_text(json.dumps({'job_id':job_id,'session_id':synthetic_session,
            'status':'completed','operation':'synthetic_ui_fixture','solver':'synthetic_ui_fixture',
            'created_utc':'2026-10-08T00:00:00Z'}))
        (candidate_dir/'synthetic-ui-only.json').write_bytes(body)
        (candidate_dir/'candidate.json').write_text(json.dumps(candidate,ensure_ascii=False))
    except Exception:
        shutil.rmtree(candidate_dir);shutil.rmtree(job_dir);raise
    return {'directories':[candidate_dir,job_dir],'payload':payload,'body':body}

def synthetic_board(path, origin, square):
    """8x6 squares -> 7x5 inner corners, 320x240 monochrome lossless PNG."""
    width,height=320,240;pixels=bytearray([190]*(width*height))
    for row in range(6):
        for col in range(8):
            value=15 if (row+col)%2 else 245
            for y in range(origin[1]+row*square,origin[1]+(row+1)*square):
                start=y*width+origin[0]+col*square;pixels[start:start+square]=bytes([value])*square
    raw=b''.join(b'\0'+pixels[y*width:(y+1)*width] for y in range(height))
    def chunk(name,body):return struct.pack('!I',len(body))+name+body+struct.pack('!I',zlib.crc32(name+body)&0xffffffff)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!IIBBBBB',width,height,8,0,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:5845')
    parser.add_argument('--chromium',type=Path,required=True,help='Already installed local Chromium executable')
    parser.add_argument('--fixture-store',type=Path,required=True,help='Running offline server --data directory inside this checkout/data; temporary invalid synthetic export records only')
    parser.add_argument('--output',type=Path,default=ROOT/'data/calibration-ui-qa')
    args=parser.parse_args();parsed=urlsplit(args.url)
    if parsed.scheme!='http' or parsed.hostname not in {'localhost','127.0.0.1','::1'} or parsed.username or parsed.password:parser.error('Use an unauthenticated HTTP localhost URL')
    if not args.chromium.is_file():parser.error('Chromium executable must already exist; nothing is downloaded')
    args.output.mkdir(parents=True,exist_ok=True);inputs=args.output/'inputs';inputs.mkdir(exist_ok=True)
    boards=[]
    for i,(origin,size)in enumerate([((40,35),24),((70,60),24),((55,70),20)]):
        path=inputs/f'synthetic-board-{i}.png';synthetic_board(path,origin,size);boards.append(path)
    oversized=inputs/'oversized-timestamps.jsonl';oversized.write_bytes(b' '*((2*1024**2)+1))
    receipt={'status':'running','url':args.url,'versions':{'playwright':importlib.metadata.version('playwright')},
        'execution':'One headless browser, GPU disabled, one renderer process; three 320x240 synthetic board images; real selection only',
        'concurrent_load_note':'Parent reports a concurrent CPU-only AGI campaign; these are functional checks, not timing/load benchmarks',
        'mock_notice':'Candidate, cancellation and worker-error checks use explicitly synthetic HTTP UI fixtures; export uses the real HTTP route with a temporary invalid synthetic artifact; no native or OpenCV calibration is computed',
        'checks':[],'screenshots':[],'requests':[],'console':[],'page_errors':[],'blocked_requests':[],
        'inputs':{path.name:sha(path) for path in boards},'mocked_jobs':[]}
    mock={'operation':None,'job':None,'candidate':None,'canceled':False}
    browser=None;page=None;export_fixture=None
    candidate_id=uuid.uuid4().hex;job_id=uuid.uuid4().hex
    def check(name,condition,evidence=None):
        assert condition,f'{name}: {evidence}';receipt['checks'].append({'name':name,'passed':True,'evidence':evidence});print('PASS '+name,flush=True)
    def screenshot(name):
        page.screenshot(path=str(args.output/name),full_page=True);receipt['screenshots'].append({'file':name,'sha256':sha(args.output/name)})
    def wait_text(selector,value,timeout=10000):
        page.wait_for_function('([selector,value])=>document.querySelector(selector).textContent.includes(value)',arg=[selector,value],timeout=timeout)
    def text(selector):return page.locator(selector).inner_text()
    def candidate_fixture(session_id,native=False):
        return {'candidate_id':candidate_id,'session_id':session_id,'job_id':job_id,'solver':'mrcal' if native else 'opencv',
          'quality_status':'SYNTHETIC UI FIXTURE — not a computed calibration','calibration':{'synthetic_ui_fixture':True,'rms_error_px':.321 if native else 0.0},
          'report':{'status':'synthetic_ui_fixture_not_a_calibration','training_rms_px':.321 if native else 0.0,
            'holdout':{'status':'not_computed'},'uncertainty':{'status':'not_computed'},
            'models':{'opencv8':{'holdout':{'rms_px':.654},'uncertainty':{'p95_px':.987}}} if native else {},
            'issues':['SYNTHETIC UI ONLY: these displayed diagnostics are injected test values.'],
            'warnings':['No native or baseline calibration was run. This export is deliberately not valid camera intrinsics.']},
          'artifacts':[{'name':'synthetic-ui-only.json','content_type':'application/json','size_bytes':59}]}
    try:
        from playwright.sync_api import sync_playwright
        with sync_playwright() as p:
            browser=p.chromium.launch(headless=True,executable_path=str(args.chromium),args=['--disable-gpu','--renderer-process-limit=1'])
            receipt['versions']['chromium']=browser.version
            context=browser.new_context(viewport={'width':1440,'height':1100},device_scale_factor=1,accept_downloads=True)
            origin=(parsed.scheme,parsed.hostname,parsed.port or 80)
            def route(r):
                req=r.request;target=urlsplit(req.url);path=target.path
                receipt['requests'].append({'url':req.url,'method':req.method,'mocked':bool(mock['operation'])})
                if (target.scheme,target.hostname,target.port or 80)!=origin:
                    receipt['blocked_requests'].append({'url':req.url,'reason':'external origin'});r.abort('blockedbyclient');return
                allowed_post=path in {'/api/calibration/assets','/api/calibration/sessions','/api/calibration/jobs'} or (path.startswith('/api/calibration/jobs/') and path.endswith('/cancel'))
                if req.method!='GET' and not (req.method=='POST' and allowed_post):
                    receipt['blocked_requests'].append({'url':req.url,'reason':'configuration/activation/unrelated write forbidden'});r.abort('blockedbyclient');return
                if mock['operation'] and path=='/api/calibration/jobs' and req.method=='POST':
                    payload=req.post_data_json;session_id=payload['session_id'];operation=mock['operation']
                    status='queued' if operation=='cancel' else 'failed' if operation=='failed' else 'completed'
                    job={'job_id':job_id,'session_id':session_id,'operation':payload['operation'],'solver':payload['solver'],'status':status,'stage':'SYNTHETIC UI fixture',
                      'progress':{'processed_frames':0,'accepted_views':0},'error':'Synthetic UI worker failure; inputs retained' if operation=='failed' else None,
                      'result':{'candidate_id':candidate_id} if operation in {'baseline','native'} else None}
                    mock['job']=job;receipt['mocked_jobs'].append(operation)
                    if operation in {'baseline','native'}:mock['candidate']=candidate_fixture(session_id,operation=='native')
                    r.fulfill(content_type='application/json',body=json.dumps(job));return
                if mock['operation'] and path=='/api/calibration/jobs/'+job_id+'/cancel':
                    mock['canceled']=True;job={**mock['job'],'status':'canceled'};r.fulfill(content_type='application/json',body=json.dumps(job));return
                if mock['operation'] and path=='/api/calibration/jobs/'+job_id:
                    job={**mock['job'],'status':'canceled' if mock['canceled'] else 'running'};r.fulfill(content_type='application/json',body=json.dumps(job));return
                if mock['candidate'] and path=='/api/calibration/candidates/'+candidate_id:
                    r.fulfill(content_type='application/json',body=json.dumps(mock['candidate']));return
                # Chromium's download navigation bypasses route interception;
                # artifact bytes must come from the genuine local HTTP server.
                r.continue_()
            context.route('**/*',route);page=context.new_page()
            page.on('console',lambda message:receipt['console'].append({'type':message.type,'text':message.text}) if len(receipt['console'])<128 else None)
            page.on('pageerror',lambda error:receipt['page_errors'].append(str(error)))
            # Same QA-only named poll throttle as the field harness; rendering,
            # calibration monitor and expiry timers keep their production rates.
            page.add_init_script("const qt=globalThis.setTimeout;globalThis.setTimeout=(fn,ms,...a)=>qt(fn,['pollStatus','pollPreview'].includes(fn?.name)?Math.max(ms,1200):ms,...a)")
            page.goto(args.url,wait_until='networkidle',timeout=15000)
            wait_text('#cal-connection','Local offline workspace')
            capabilities=page.evaluate("JSON.parse(document.querySelector('#cal-capabilities').textContent)");receipt['capabilities']=capabilities
            check('standalone workspace suppresses live camera/vision controls',page.locator('body').evaluate("e=>e.classList.contains('calibration-only')") and not page.locator('#setup-form').is_visible())
            check('runtime activation is absent in standalone app',not page.locator('#cal-activation').is_visible() and capabilities.get('runtime_activation') is False)
            native_option=page.locator('#cal-solver option[value="mrcal"]')
            native_disabled=native_option.evaluate('option=>option.disabled')
            check('native mrcal availability matches configured worker',native_disabled==(not capabilities['solvers']['mrcal']['available']))
            if not capabilities['solvers']['mrcal']['available']:
                page.locator('#cal-solver').focus();page.locator('#cal-solver').press('End')
                check('unavailable native solver is skipped by actual keyboard selection',page.locator('#cal-solver').input_value()=='opencv' and native_option.evaluate('option=>option.disabled'))
            check('unknown camera model does not guess mode or physical ID',page.locator('#cal-width').input_value()=='' and page.locator('#cal-physical-id').input_value()=='')
            page.locator('#cal-cols').fill('7');page.locator('#cal-rows').fill('5');page.locator('#cal-square').fill('.03')
            page.locator('#cal-camera-label').fill('SYNTHETIC board PNGs — OV9281 label is manual')
            page.locator('#cal-capture-history').fill('Generated lossless 320x240 PNGs for software UI selection only; no camera or OBS capture.')
            page.locator('.cal-mode-details summary').click();page.locator('#cal-focus-kind').select_option('fixed');page.locator('#cal-focus-locked').check()
            page.locator('#cal-files').set_input_files([str(path) for path in boards]);wait_text('#cal-upload-status','3 source assets')
            check('three real local uploads complete sequentially',page.locator('#cal-assets li').count()==3)
            asset_posts=sum(req['method']=='POST' and urlsplit(req['url']).path=='/api/calibration/assets' for req in receipt['requests'])
            page.locator('#cal-timestamps').set_input_files(str(oversized));wait_text('#cal-message','no larger than 2 MiB')
            check('oversized sidecar is rejected before an asset request',sum(req['method']=='POST' and urlsplit(req['url']).path=='/api/calibration/assets' for req in receipt['requests'])==asset_posts)
            form_validity=page.locator('#cal-setup-form').evaluate("form=>({valid:form.checkValidity(),invalid:Array.from(form.elements).filter(e=>e.willValidate&&!e.validity.valid).map(e=>({id:e.id,value:e.value,message:e.validationMessage,stepMismatch:e.validity.stepMismatch}))})")
            check('required setup and collapsed default thresholds satisfy native form validity',form_validity['valid'] and not form_validity['invalid'],form_validity)
            page.locator('#cal-create').click();page.wait_for_function("()=>document.querySelector('#cal-session-id').textContent.length===32",timeout=10000)
            session_id=text('#cal-session-id');receipt['real_session_id']=session_id
            check('saved session locks immutable board and camera declarations',page.locator('#cal-cols').is_disabled() and page.locator('#cal-camera-label').is_disabled())
            check('solve remains gated until actual selection and review',page.locator('#cal-solve').is_disabled())
            page.locator('#cal-select').click();wait_text('#cal-job-state','completed',timeout=30000)
            page.wait_for_function("()=>Number(document.querySelector('#cal-accepted').textContent)>0",timeout=10000)
            accepted=int(text('#cal-accepted'));receipt['real_selection_accepted_views']=accepted
            check('tiny real CPU selection reports actual coverage and decoded resolution',accepted>=1 and page.locator('#cal-coverage-grid span').count()==48 and 'Decoded 320 × 240' in text('#cal-resolution'))
            check('unknown declared dimensions remain separate from decoded dimensions','declared' not in text('#cal-resolution').lower())
            page.locator('#cal-preview').wait_for(state='visible');page.wait_for_function("()=>document.querySelector('#cal-preview').naturalWidth>0",timeout=5000)
            check('review shows actual worker preview and bounded view metrics',page.locator('#cal-views tr').count()==accepted)
            check('image sequence times are not labeled measured capture time','image_sequence_index_not_capture_time' in text('#cal-clock'))
            check('review checkbox is explicit before solving',page.locator('#cal-solve').is_disabled())
            page.locator('#cal-review-confirm').check();check('review enables the baseline solve action',not page.locator('#cal-solve').is_disabled())
            screenshot('desktop-review.png')

            mock['operation']='cancel';page.locator('#cal-select').click();wait_text('#cal-job-state','queued');page.locator('#cal-cancel').click()
            wait_text('#cal-message','Saved inputs are retained')
            check('cancellation stops polling and retains saved inputs with truthful guidance','run selection again' in text('#cal-message'))
            page.locator('#cal-review-confirm').check()
            mock.update(operation='baseline',canceled=False);page.locator('#cal-solve').click();wait_text('#cal-candidate-state','SYNTHETIC UI FIXTURE')
            check('baseline keeps fit residual distinct from uncomputed validation/uncertainty',text('#cal-fit')=='0.000 px' and text('#cal-heldout')=='Not computed' and text('#cal-uncertainty')=='Not computed')
            export_fixture=install_export_fixture(args.fixture_store,candidate_id,job_id)
            export_url=page.locator('#cal-downloads a').first.get_attribute('href')
            expected_path=f'/api/calibration/candidates/{candidate_id}/artifacts/synthetic-ui-only.json'
            check('candidate export link addresses its exact allowlisted local artifact',export_url==expected_path,export_url)
            with urlopen(args.url.rstrip('/')+expected_path,timeout=5) as response:
                artifact_headers=dict(response.headers);artifact_bytes=response.read();artifact_status=response.status
            receipt['export_http']={'url':args.url.rstrip('/')+expected_path,'status':artifact_status,'headers':artifact_headers,
                'sha256':hashlib.sha256(artifact_bytes).hexdigest(),'synthetic_ui_fixture':True,'valid_calibration':False}
            check('actual HTTP export returns exact synthetic bytes and attachment metadata',artifact_status==200 and artifact_bytes==export_fixture['body'] and 'attachment;' in artifact_headers.get('Content-Disposition',''))
            with page.expect_download() as download_info:page.locator('#cal-downloads a').first.click()
            download=download_info.value;download_failure=download.failure()
            receipt['export_download']={'url':download.url,'failure':download_failure,'suggested_filename':download.suggested_filename}
            check('actual browser export completes without download failure',download_failure is None,receipt['export_download'])
            target=args.output/'synthetic-ui-only.json';download.save_as(str(target))
            check('candidate export is a local explicit download',target.read_bytes()==export_fixture['body'] and json.loads(target.read_text())==export_fixture['payload'])
            check('mock candidate cannot imply physical qualification','not physically qualified' in text('#cal-candidate-state') and 'SYNTHETIC UI ONLY' in text('#cal-candidate-issues'))

            mock['operation']='native';page.locator('#cal-solve').click();wait_text('#cal-heldout','0.654 px RMS')
            check('synthetic native UI fixture displays three distinct diagnostic families',text('#cal-fit')=='0.321 px' and text('#cal-uncertainty')=='0.987 px p95')
            check('synthetic native diagnostics remain prominently qualified','SYNTHETIC UI FIXTURE' in text('#cal-candidate-state') and 'No native or baseline calibration was run' in text('#cal-candidate-warnings'))
            screenshot('desktop-export-synthetic-ui.png')
            mock['operation']='failed';page.locator('#cal-solve').click();wait_text('#cal-message','Synthetic UI worker failure')
            check('worker failure clears obsolete candidate and surfaces actual error','No candidate solved' in text('#cal-candidate-state') and page.locator('#cal-downloads a').count()==0)
            mock['operation']=None;mock['candidate']=None
            page.locator('#cal-new').click();page.locator('#cal-sessions').select_option(session_id);page.locator('#cal-resume').click()
            wait_text('#cal-session-id',session_id)
            check('saved session recovery preserves review without applying anything',int(text('#cal-accepted'))==accepted and page.locator('#cal-solve').is_disabled())
            page.set_viewport_size({'width':390,'height':844});page.wait_for_timeout(100)
            check('mobile calibration workspace has no horizontal overflow',page.evaluate('document.documentElement.scrollWidth<=innerWidth+1'))
            screenshot('mobile-review.png')
            check('no unhandled JavaScript errors',not receipt['page_errors'],receipt['page_errors'])
            check('no external request or runtime/configuration write attempted',not receipt['blocked_requests'],receipt['blocked_requests'])
            receipt['status']='passed';browser.close();browser=None
    except Exception:
        receipt['status']='failed';receipt['exception']=traceback.format_exc();traceback.print_exc()
        if page:
            try:screenshot('failure.png')
            except Exception:pass
    finally:
        if browser:
            try:browser.close()
            except Exception:pass
        if export_fixture:
            for directory in export_fixture['directories']:shutil.rmtree(directory)
            receipt['temporary_export_fixture_removed']=True
        (args.output/'browser-qa.json').write_text(json.dumps(receipt,allow_nan=False,indent=2)+'\n')
    print(f"{receipt['status'].upper()}: {len(receipt['checks'])} checks; {args.output/'browser-qa.json'}",flush=True)
    return 0 if receipt['status']=='passed' else 1

if __name__=='__main__':sys.exit(main())
