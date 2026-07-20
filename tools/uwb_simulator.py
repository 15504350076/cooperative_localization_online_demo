#!/usr/bin/env python3
"""确定性的三车测距数据源，用于验证仅测距兼容模式。

The temporary integration transport is UDP/ZJCL.  The production AIBrainBox
adapter remains an SJTU ROS 2 responsibility; this utility only makes the
algorithm library and GCS path independently testable before hardware arrives.
"""

import argparse
import math
import random
import socket
import sys
import time

import zjcl_protocol as zjcl


DEFAULT_EDGES = (
    (1, 2, 3.0),
    (1, 3, 4.0),
    (2, 3, 5.0),
)


def _finite_nonnegative(name, value):
    number = float(value)
    if not math.isfinite(number) or number < 0.0:
        raise ValueError(f"{name} must be finite and non-negative")
    return number


def _probability(name, value):
    number = float(value)
    if not math.isfinite(number) or number < 0.0 or number > 1.0:
        raise ValueError(f"{name} must be in [0, 1]")
    return number


class UwbSimulator:
    """Generate one ZJCL range datagram per non-dropped cooperative edge."""

    def __init__(
        self,
        *,
        seed,
        noise_std_m,
        nlos_probability,
        drop_probability,
        range_std_m,
        nlos_bias_m=0.5,
    ):
        self.noise_std_m = _finite_nonnegative("noise_std_m", noise_std_m)
        self.nlos_probability = _probability(
            "nlos_probability", nlos_probability
        )
        self.drop_probability = _probability(
            "drop_probability", drop_probability
        )
        self.range_std_m = _finite_nonnegative("range_std_m", range_std_m)
        if self.range_std_m == 0.0:
            raise ValueError("range_std_m must be positive")
        self.nlos_bias_m = _finite_nonnegative("nlos_bias_m", nlos_bias_m)
        self._random = random.Random(seed)
        self.sequence_by_edge = {
            (source, target): 0 for source, target, _ in DEFAULT_EDGES
        }

    def generate_tick(self, timestamp_ns):
        if isinstance(timestamp_ns, bool) or not isinstance(timestamp_ns, int):
            raise ValueError("timestamp_ns must be an integer")
        if timestamp_ns < 0 or timestamp_ns > (1 << 64) - 1:
            raise ValueError("timestamp_ns is outside uint64")

        datagrams = []
        for source, target, true_range_m in DEFAULT_EDGES:
            edge = (source, target)
            self.sequence_by_edge[edge] += 1
            sequence = self.sequence_by_edge[edge]

            if self._random.random() < self.drop_probability:
                continue

            nlos_flag = self._random.random() < self.nlos_probability
            noise_m = self._random.gauss(0.0, self.noise_std_m)
            measured_range_m = (
                true_range_m
                + noise_m
                + (self.nlos_bias_m if nlos_flag else 0.0)
            )
            measured_range_m = max(measured_range_m, 1.0e-6)
            payload = zjcl.encode_range_payload(
                zjcl.RangePayload(
                    range_m=measured_range_m,
                    range_std_m=self.range_std_m,
                    nlos_probability=self.nlos_probability,
                    nlos_flag=nlos_flag,
                    has_nlos_probability=True,
                    valid=True,
                    status=0,
                )
            )
            datagrams.append(
                zjcl.encode_frame(
                    zjcl.Frame(
                        message_type=zjcl.MSG_RANGE,
                        flags=0,
                        sequence=sequence,
                        timestamp_ns=timestamp_ns,
                        source_node=source,
                        target_node=target,
                        payload=payload,
                    ),
                    udp=True,
                )
            )
        return datagrams


def _positive_float(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a number") from error
    if not math.isfinite(value) or value <= 0.0:
        raise argparse.ArgumentTypeError("must be finite and positive")
    return value


def _nonnegative_float(text):
    try:
        value = float(text)
    except ValueError as error:
        raise argparse.ArgumentTypeError("must be a number") from error
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("must be finite and non-negative")
    return value


def _unit_float(text):
    value = _nonnegative_float(text)
    if value > 1.0:
        raise argparse.ArgumentTypeError("must be in [0, 1]")
    return value


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description="Send deterministic 3-4-5 UWB ranges over temporary ZJCL/UDP"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=39001)
    parser.add_argument("--rate-hz", type=_positive_float, default=20.0)
    parser.add_argument(
        "--duration",
        type=_nonnegative_float,
        default=10.0,
        help="seconds; zero runs until interrupted",
    )
    parser.add_argument("--seed", type=int, default=2026)
    parser.add_argument("--noise-std-m", type=_nonnegative_float, default=0.02)
    parser.add_argument("--range-std-m", type=_positive_float, default=0.10)
    parser.add_argument(
        "--nlos-probability", type=_unit_float, default=0.0
    )
    parser.add_argument("--nlos-bias-m", type=_nonnegative_float, default=0.5)
    parser.add_argument("--drop-probability", type=_unit_float, default=0.0)
    return parser


def run(args):
    if args.port < 1 or args.port > 65535:
        raise ValueError("port must be in [1, 65535]")
    simulator = UwbSimulator(
        seed=args.seed,
        noise_std_m=args.noise_std_m,
        nlos_probability=args.nlos_probability,
        drop_probability=args.drop_probability,
        range_std_m=args.range_std_m,
        nlos_bias_m=args.nlos_bias_m,
    )
    target = (args.host, args.port)
    period_s = 1.0 / args.rate_hz
    start = time.monotonic()
    next_tick = start
    tick_count = 0
    sent_count = 0

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as output_socket:
        try:
            while args.duration == 0.0 or time.monotonic() - start < args.duration:
                timestamp_ns = time.time_ns()
                datagrams = simulator.generate_tick(timestamp_ns)
                for datagram in datagrams:
                    output_socket.sendto(datagram, target)
                    sent_count += 1
                tick_count += 1
                next_tick += period_s
                delay_s = next_tick - time.monotonic()
                if delay_s > 0.0:
                    time.sleep(delay_s)
                elif delay_s < -period_s:
                    next_tick = time.monotonic()
        except KeyboardInterrupt:
            pass

    elapsed_s = time.monotonic() - start
    print(
        "SUMMARY status=OK"
        f" ticks={tick_count} sent={sent_count} elapsed_s={elapsed_s:.3f}"
        f" target={args.host}:{args.port}",
        flush=True,
    )
    return 0


def main(argv=None):
    parser = build_argument_parser()
    args = parser.parse_args(argv)
    try:
        return run(args)
    except (OSError, ValueError) as error:
        print(f"ERROR {error}", file=sys.stderr, flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
