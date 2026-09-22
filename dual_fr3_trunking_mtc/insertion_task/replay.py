"""Deterministic policy replay; this module has no robot or ROS connection."""
import argparse
import json
import math

from dual_fr3_maniskill.usb.insertion import InsertionPolicy
from .real_session import load_real_config


def replay(config, records):
    policy = InsertionPolicy(config['insertion'])
    previous = previous_operation = None
    for record in records:
        if record.get('kind', 'policy_event') != 'policy_event':
            continue
        timestamp = float(record['time_s'])
        operation = record.get('operation', 'update')
        paired_step = (operation == 'alignment_step' and previous_operation == 'alignment_update'
                       and timestamp == previous)
        if not math.isfinite(timestamp) or (previous is not None and timestamp <= previous and not paired_step):
            raise ValueError('Replay timestamps must strictly increase except paired alignment steps')
        observation = record['observation']
        if operation == 'begin_alignment':
            policy.begin_alignment(timestamp, observation)
        elif operation == 'alignment_update':
            if policy.previous is None:
                raise ValueError('Replay requires an explicit begin or begin_alignment event')
            policy.update_alignment(timestamp, observation)
        elif operation == 'alignment_step':
            if not paired_step:
                raise ValueError('Alignment step requires a same-time alignment update')
            distance, angle = float(record['distance_m']), float(record['angle_rad'])
            if not all(math.isfinite(x) and x >= 0 for x in (distance, angle)):
                raise ValueError('Alignment step must be finite and nonnegative')
            policy.record_alignment_step(distance, angle)
        elif operation == 'begin':
            policy.begin(timestamp, observation)
        elif operation == 'update':
            if policy.previous is None:
                raise ValueError('Replay requires an explicit begin or begin_alignment event')
            policy.update(timestamp, observation)
        else:
            raise ValueError('Unsupported replay operation: ' + operation)
        previous, previous_operation = timestamp, operation
        yield dict(time_s=timestamp, operation=operation, **policy.snapshot())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--input', required=True, help='JSONL: time_s, operation, observation')
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    config = load_real_config(args.config)
    with open(args.input, encoding='utf-8') as source:
        records = [json.loads(line) for line in source if line.strip()]
    results = list(replay(config, records))
    with open(args.output, 'w', encoding='utf-8') as target:
        for result in results:
            target.write(json.dumps(result, allow_nan=False) + '\n')
    print(f'Replayed {len(results)} observations without ROS or robot commands')
    return 0
