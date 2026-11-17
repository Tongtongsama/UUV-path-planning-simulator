"""Export a verified LOTUSim navigation recording as one offline HTML player."""
import argparse
import hashlib
import json
from pathlib import Path


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def export(recording, output):
    recording=Path(recording).resolve(); output=Path(output).resolve()
    if output.exists(): raise FileExistsError(f'Output already exists: {output}')
    if output.is_relative_to(recording): raise ValueError('Output must be outside the source recording')
    manifest=json.loads((recording/'manifest.json').read_text(encoding='utf-8'))
    normalized={name.replace('\\','/'):value for name,value in manifest.items()}
    required={'plan.json','navigation.jsonl','transactions.jsonl','worlds/minimal.world','metadata.json','run_status.json'}
    if not required.issubset(normalized): raise ValueError('Incomplete recording manifest')
    for name,expected in normalized.items():
        path=(recording/name).resolve()
        if not path.is_relative_to(recording) or not path.is_file() or digest(path)!=expected:
            raise ValueError(f'Recording hash mismatch: {name}')
    plan=json.loads((recording/'plan.json').read_text(encoding='utf-8'))
    meta=json.loads((recording/'metadata.json').read_text(encoding='utf-8'))
    status=json.loads((recording/'run_status.json').read_text(encoding='utf-8'))
    nav=[json.loads(line) for line in (recording/'navigation.jsonl').read_text(encoding='utf-8').splitlines() if line]
    states={row['sequence']:row for row in (json.loads(line) for line in
            (recording/'transactions.jsonl').read_text(encoding='utf-8').splitlines() if line) if row.get('valid')}
    if not nav or any(row['sequence'] not in states for row in nav): raise ValueError('Missing navigation state')
    if any(b['elapsed']<=a['elapsed'] for a,b in zip(nav,nav[1:])): raise ValueError('Non-monotone navigation time')
    samples=[]
    for row in nav:
        state=states[row['sequence']]
        samples.append([row['elapsed'],*row['position_xz'],state['pitch_nose_up_rad'],
                        row['swept_clearance_m'],row['path_distance_m'],*row['lookahead_xz'],
                        row['target_pitch'],row['phase']])
    acceptance_path=recording/'acceptance.json'
    acceptance=json.loads(acceptance_path.read_text(encoding='utf-8')) if acceptance_path.exists() else None
    payload={'scene':plan['config']['scene'],'case':plan['case'],'flow':meta.get('flow','F0'),
             'current':plan['config']['current'],'obstacles':
             plan['config'].get('obstacles',[plan['config'].get('obstacle')]),
             'boundary':plan['config']['boundary'],'goal':plan['config']['goal'],
             'path':plan['path'],'raw_path':plan['raw_path'],'samples':samples,
             'status':status['status'],'accepted':None if acceptance is None else acceptance['passed'],
             'source':str(recording)}
    template=(Path(__file__).with_suffix('.html')).read_text(encoding='utf-8')
    data=json.dumps(payload,ensure_ascii=False,allow_nan=False,separators=(',',':')).replace('<','\\u003c')
    output.mkdir(parents=True)
    (output/'index.html').write_text(template.replace('/* REPLAY_DATA */',data),encoding='utf-8')
    (output/'source.json').write_text(json.dumps({'recording':str(recording),'manifest_sha256':digest(recording/'manifest.json'),
        'acceptance_sha256':digest(acceptance_path) if acceptance_path.exists() else None,
        'exporter_sha256':digest(Path(__file__)),'html_sha256':digest(output/'index.html'),
        'source_unchanged':all(digest(recording/name)==value for name,value in normalized.items()),
        'display_only':True},indent=2),encoding='utf-8')
    return output/'index.html'


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    print(export(args.recording,args.output))


if __name__=='__main__': main()
