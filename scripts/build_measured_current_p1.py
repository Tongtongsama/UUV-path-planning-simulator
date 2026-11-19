"""Convert the raw NOAA CO-OPS pm0101 bin-1 current JSON into current-data-v1 CSV."""
from datetime import datetime, timezone
from pathlib import Path
import csv, json, math

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT/'artifacts/current_data_p1_20261118_01/raw/pm0101_20261117_bin1.json'
OUT = ROOT/'artifacts/current_data_p1_20261118_01'


def main():
    raw = json.loads(RAW.read_text(encoding='utf-8-sig'))
    data = raw['data']
    start = datetime.strptime(data[0]['t'], '%Y-%m-%d %H:%M').replace(tzinfo=timezone.utc)
    rows = []
    for item in data:
        t = datetime.strptime(item['t'], '%Y-%m-%d %H:%M').replace(tzinfo=timezone.utc)
        speed_mps = float(item['s']) / 100.0  # CO-OPS currents speed is cm/s
        direction_deg = float(item['d'])
        east = speed_mps * math.sin(math.radians(direction_deg))
        north = speed_mps * math.cos(math.radians(direction_deg))
        rows.append((t, (t - start).total_seconds(), east, north, 0.0, 0.0, direction_deg, speed_mps))
    with (OUT/'pm0101_measured_uniform.csv').open('w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['time_s','east_mps','north_mps','up_mps','uncertainty_mps','time_utc','direction_deg','speed_mps'])
        for r in rows:
            w.writerow([r[1], r[2], r[3], r[4], r[5], r[0].isoformat(), r[6], r[7]])
    metadata = {
        'schema':'current-data-v1','source_type':'measured',
        'source_id':'NOAA-COOPS-pm0101-bin1-20261117',
        'license':'public-domain-US-government',
        'provenance':'NOAA CO-OPS Portsmouth Harbor, Gangway Rocks LB13 (pm0101), bin 1, 2026-11-17 UTC. Raw: https://api.tidesandcurrents.noaa.gov/api/prod/datagetter?product=currents&station=pm0101&bin=1. Speed converted cm/s -> m/s; direction is CO-OPS toward-direction (degrees true).',
        'datum':'CO-OPS station bin 1; no chart datum or vertical current supplied',
        'time_reference':'UTC','vertical_reference':'surface bin 1; up_mps set to 0',
        'station':raw['metadata'],
        'sample_count':len(rows),
        'start_utc':start.isoformat(),
        'end_utc':rows[-1][0].isoformat(),
        'speed_units_original':'cm/s',
        'speed_units_converted':'m/s',
        'uncertainty':'not supplied by CO-OPS; set to 0.0 and must not be read as a calibrated uncertainty',
        'spatial_support':'single station treated as spatially uniform for P1 smoke only',
        'limitations':['no vertical current','no spatial shear','no chart datum/tide correction','public CO-OPS observation, not a navigation-grade survey']
    }
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    print(json.dumps({'samples':len(rows),'start':metadata['start_utc'],'end':metadata['end_utc'],'max_speed_mps':max(r[7] for r in rows)},indent=2))

if __name__=='__main__': main()
