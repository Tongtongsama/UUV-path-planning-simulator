"""Offline gate CLI; deliberately provides no simulation launch command."""
import argparse,json
from pathlib import Path
from .gate import evaluate

def main():
    p=argparse.ArgumentParser();p.add_argument('--python-matrix',type=Path,required=True);p.add_argument('--report',type=Path,required=True)
    args=p.parse_args()
    if args.report.exists():p.error('Report exists')
    try:result=evaluate(args.python_matrix).report
    except Exception as exc:result=dict(passed=False,ready_to_run=False,real_started=False,error=repr(exc))
    with args.report.open('x',encoding='utf-8') as f:json.dump(result,f,indent=2)
    print(json.dumps({k:v for k,v in result.items() if k!='adapter_equivalence'},indent=2))
    return 0 if result.get('passed') else 2
if __name__=='__main__':raise SystemExit(main())
