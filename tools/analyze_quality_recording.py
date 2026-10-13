"""Summarize measured corner residuals; does not establish pose accuracy or ground truth.
Usage: python tools/analyze_quality_recording.py recording.jsonl [--threshold 3]
"""
import argparse
import json
import math


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('files', nargs='+')
    parser.add_argument('--threshold', type=float, default=3)
    args = parser.parse_args()
    maxima = []
    for path in args.files:
        with open(path) as stream:
            for line in stream:
                record = json.loads(line)
                values = list(record['per_tag_rms_px'].values())
                if values and all(math.isfinite(v) for v in values):
                    maxima.append(max(values))
    maxima.sort()
    print(f'{len(maxima)} frames with measured per-tag corner residuals')
    for percentile in (50, 90, 95, 99):
        if maxima:
            print(f'p{percentile}: {maxima[round((len(maxima)-1)*percentile/100)]:.3f} px')
    print(f'{sum(v > args.threshold for v in maxima)} exceed {args.threshold:g} px')
    print('Residual consistency is not ground-truth pose accuracy. Review known-good and failed frames separately.')


if __name__ == '__main__':
    main()
