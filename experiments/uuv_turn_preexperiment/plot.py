"""Regenerate figures from persisted CSV and JSON without rerunning dynamics."""
import argparse
import csv
import json
from pathlib import Path

from .run import make_plots


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    output=parser.parse_args().output
    run=json.loads((output/'run_summary.json').read_text(encoding='utf-8'))
    histories={}
    for name in ('identity','E1_00','E1_08','E2_00','E2_20'):
        with (output/f'{name}.csv').open(newline='',encoding='utf-8') as handle:
            histories[name]=[
                {key:float(value) if key in ('time_s','x_m','z_m','pitch_rad','q_rad_s','tau_m_applied_Nm') else value
                 for key,value in row.items()}
                for row in csv.DictReader(handle)]
    make_plots(output,run['groups'],histories,run['config'])


if __name__=='__main__':main()
