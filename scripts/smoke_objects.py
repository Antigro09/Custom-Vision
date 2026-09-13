#!/usr/bin/env python3
"""Verify runtime ranging, target association and invalidation with explicit synthetic input."""
import json
from pathlib import Path
import subprocess
import sys


def main():
    root=Path(__file__).resolve().parents[1]
    result=subprocess.run([sys.executable,str(root/'scripts'/'demo_objects.py'),'--headless','--stdout-json',
                           '--max-frames','20','--clear-after','16'],cwd=root,text=True,capture_output=True,timeout=60)
    if result.returncode:raise RuntimeError(result.stderr)
    directory=root/'data'/'object-demo'
    (directory/'smoke-results.jsonl').write_text(result.stdout)
    packets=[json.loads(line) for line in result.stdout.splitlines()]
    frames=[p for p in packets if p['connected']]
    observations=[p for p in frames if p['frame_id']<16]
    empty_frames=[p for p in frames if p['frame_id']>=16]
    # Cold initialization or load may legitimately exceed the age limit. Those
    # frames must clear targets; the smoke test must never disable that protection.
    assert len(observations)>=8 and empty_frames,[(p['error'],p['frame_id']) for p in packets]
    track_ids=[]
    expected=[[1.1,-.25,.05],[1.7,.3,.05],[.85,.15,.05]]
    for p in observations:
        obj=p['objects'];assert obj['valid'] and len(obj['targets'])==3
        ids=sorted(t['track_id'] for t in obj['targets']);track_ids.append(ids)
        for target in obj['targets']:
            xyz=target['translation_m']
            assert min(sum((a-b)**2 for a,b in zip(xyz,e))**.5 for e in expected)<.015,xyz
            assert target['capture_monotonic_us']==p['capture_monotonic_us']
            assert target['observed'] and not target['predicted']
        assert obj['selected_track_id'] in ids
        assert 'selected_target' not in obj  # NT sends each target once, selected by ID.
    fresh_pairs=0
    for i in range(1,len(track_ids)):
        gap_us=observations[i]['capture_monotonic_us']-observations[i-1]['capture_monotonic_us']
        if gap_us<95000 and observations[i]['frame_id']==observations[i-1]['frame_id']+1:
            assert track_ids[i]==track_ids[i-1],(i,gap_us,track_ids)
            fresh_pairs+=1
    assert fresh_pairs>0,'No consecutive fresh frames were processed'
    for p in empty_frames:assert not p['detections'] and not p['objects']['valid'] and not p['objects']['targets']
    for p in packets:
        if not p['connected']:assert not p['detections'] and not p.get('objects',{}).get('valid',False)
    assert packets[-1]['connected'] is False and packets[-1]['detections']==[]
    summary={'synthetic_only':True,'position_tolerance_m':.015,'requested_frames':20,'valid_observation_frames':len(observations),
             'empty_frames':len(empty_frames),'stable_fresh_pairs':fresh_pairs,'lost_targets_cleared':True,'shutdown_cleared':True}
    directory=root/'data'/'object-demo';(directory/'smoke.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
