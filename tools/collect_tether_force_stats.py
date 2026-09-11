#!/usr/bin/env python3
import argparse
import json
import math
import re
import subprocess


FIELD_RE = re.compile(
    r'\s*([xyz]):\s*('
    r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?'
    r'|[-+]?inf'
    r'|nan'
    r')\s*$',
    re.IGNORECASE,
)


def _finish_message(current, values, delimited=True, saw_content=True):
    """Fecha uma mensagem. Campos ausentes valem 0 (protobuf omite zeros).

    Uma mensagem so e invalida se ficou aberta no fim do stream (truncada) ou se
    algum campo nao e finito. Exigir a presenca de x/y era errado: theta=0 e
    omega=0 sao valores legitimos, e a serializacao de texto os omite.
    """
    if not saw_content:
        return 0
    if not delimited:
        return 1
    sample = (current.get('x', 0.0), current.get('y', 0.0), current.get('z', 0.0))
    if not all(math.isfinite(value) for value in sample):
        return 1
    values.append(sample)
    return 0


def parse_vector3d_stream_detailed(text):
    values = []
    invalid_messages = 0
    current = {}
    saw_content = False

    for line in text.splitlines():
        if not line.strip():
            invalid_messages += _finish_message(current, values, saw_content=saw_content)
            current, saw_content = {}, False
            continue

        saw_content = True
        match = FIELD_RE.match(line)
        if not match:
            continue

        field = match.group(1).lower()
        if field in current:
            invalid_messages += _finish_message(current, values, saw_content=True)
            current = {}
        current[field] = float(match.group(2))

    invalid_messages += _finish_message(
        current, values, delimited=False, saw_content=saw_content)
    return values, invalid_messages


def parse_vector3d_stream(text):
    values, _ = parse_vector3d_stream_detailed(text)
    return values


def rms(items):
    if not items:
        return None
    return math.sqrt(sum(value * value for value in items) / len(items))


def main():
    parser = argparse.ArgumentParser(description='Collect /cabo/conexao/stats metrics.')
    parser.add_argument('--samples', type=int, default=200)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--topic', default='/cabo/conexao/stats')
    args = parser.parse_args()

    cmd = [
        'gz',
        'topic',
        '-e',
        '-t',
        args.topic,
        '-n',
        str(args.samples),
    ]
    timed_out = False
    try:
        result = subprocess.run(
            cmd,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=args.timeout,
            check=False,
        )
        stdout = result.stdout
        stderr = result.stderr.strip()
        returncode = result.returncode
    except subprocess.TimeoutExpired as exc:
        timed_out = True
        stdout = exc.stdout or ''
        stderr = exc.stderr or ''
        returncode = None
        if isinstance(stdout, bytes):
            stdout = stdout.decode(errors='replace')
        if isinstance(stderr, bytes):
            stderr = stderr.decode(errors='replace')
        stderr = stderr.strip()

    values, invalid_messages = parse_vector3d_stream_detailed(stdout)
    errors = [item[0] for item in values]
    forces = [item[1] for item in values]
    saturated = [item[2] >= 0.5 for item in values]
    output = {
        'topic': args.topic,
        'requested_samples': args.samples,
        'samples': len(values),
        'error_rms_m': rms(errors),
        'error_max_m': max(errors) if errors else None,
        'force_rms_n': rms(forces),
        'force_max_n': max(forces) if forces else None,
        'saturation_fraction': (sum(saturated) / len(saturated)) if saturated else None,
        'invalid_messages': invalid_messages,
        'timed_out': timed_out,
        'valid': (
            (not timed_out)
            and returncode == 0
            and invalid_messages == 0
            and len(values) >= args.samples
        ),
        'gz_topic_returncode': returncode,
        'stderr': stderr,
    }
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
