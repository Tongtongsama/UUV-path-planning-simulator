"""Freeze a dirty-tree Phase B candidate without creating a Git commit or tag."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED

ROOT=Path(__file__).resolve().parents[1]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--git-commit',required=True)
    args=parser.parse_args()
    if len(args.git_commit)!=40 or any(c not in '0123456789abcdef' for c in args.git_commit):
        raise ValueError('require the independently read full Git HEAD')
    evidence=[]
    for name in ('guidance_phase_b_straight_v1','guidance_phase_b_remaining_v1'):
        folder=ROOT/'artifacts'/name
        manifest=json.loads((folder/'manifest.json').read_text())
        for file,expected in manifest.items():
            if digest(folder/file)!=expected:raise ValueError(f'evidence mismatch: {name}/{file}')
        summary=json.loads((folder/'summary.json').read_text())
        # Document/validation changes do not invalidate the original runtime evidence.
        for file,expected in summary['source_sha256'].items():
            if file.split('/')[0] in ('core','guidance','navigation','controller','physics','environment','planner','trajectory','simulation','config'):
                if digest(ROOT/file)!=expected:raise ValueError(f'candidate runtime differs from six-case evidence: {file}')
        evidence.extend(sorted(p for p in folder.iterdir() if p.is_file()))
    junit=ROOT/'artifacts/guidance_phase_b_tests_v1.xml'
    suites=ET.parse(junit).getroot().findall('testsuite')
    counts={key:sum(int(s.attrib.get(key,0)) for s in suites) for key in ('tests','failures','errors','skipped')}
    if counts!={'tests':751,'failures':0,'errors':0,'skipped':0}:raise ValueError(counts)
    evidence.append(junit)
    source=[]
    for folder in ('guidance','navigation','core','controller','physics','environment','planner','trajectory','simulation','validation','tests','config','docs'):
        source.extend(p for p in (ROOT/folder).rglob('*') if p.is_file() and p.suffix in ('.py','.json','.yaml','.md'))
    source.extend(ROOT/name for name in ('README.md','requirements.txt','requirements-dev.txt','requirements-validation.txt','pytest.ini','.gitignore'))
    args.output.mkdir(parents=True,exist_ok=False)
    record=dict(candidate='guidance-v1.0-phase-b-candidate',created_utc=datetime.now(timezone.utc).isoformat(),
        git_head=args.git_commit,working_tree_dirty=True,
        identity='Git HEAD plus uncommitted/untracked working-tree snapshot; NOT a release/tag/commit',
        source_sha256={p.relative_to(ROOT).as_posix():digest(p) for p in sorted(source)},
        evidence_sha256={p.relative_to(ROOT).as_posix():digest(p) for p in evidence},junit=counts,
        config=json.loads((ROOT/'config/guidance_phase_b.json').read_text()))
    with ZipFile(args.output/'snapshot.zip','w',ZIP_DEFLATED) as archive:
        for p in source+evidence:archive.write(p,p.relative_to(ROOT).as_posix())
    record['snapshot_sha256']=digest(args.output/'snapshot.zip')
    (args.output/'candidate.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
    (args.output/'manifest.json').write_text(json.dumps({p.name:digest(p) for p in args.output.iterdir() if p.is_file()},indent=2),encoding='utf-8')
    print(json.dumps(dict(candidate=record['candidate'],sources=len(source),evidence=len(evidence),junit=counts,snapshot_sha256=record['snapshot_sha256'])))


if __name__=='__main__':main()
