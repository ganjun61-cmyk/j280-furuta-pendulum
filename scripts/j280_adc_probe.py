"""Live read-only angle-sensor probe.

Never writes to the port. Run it while wiggling the three-wire sensor cable or
probing with a multimeter: every line shows the raw 10-bit code, the implied
voltage at the given reference, the firmware adc_valid flag and the encoder.

Hanging pendulum reads about 221 codes, upright about 737; with a 3.3 V ADC
reference that is 0.71 V and 2.38 V on the sensor signal wire.
"""
import argparse
import time

import serial

import j280_balance_board as board
import j280_v6_protocol as protocol6


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', required=True)
    ap.add_argument('--baud', type=int, default=230400)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--interval', type=float, default=0.5)
    ap.add_argument('--vref', type=float, default=3.3)
    ap.add_argument('--low', type=int, default=150,
                    help='lowest code of the hanging window (default 150)')
    ap.add_argument('--high', type=int, default=240,
                    help='highest code of the hanging window (default 240)')
    args = ap.parse_args()

    stream = protocol6.FrameStream()
    port = serial.Serial(args.port, args.baud, timeout=0.01)
    print(f"J280 ADC probe on {args.port}, {args.vref:g} V reference, {args.seconds:g} s. Read-only.")
    print("Hanging pendulum = ~221 codes (~0.71 V at 3.3 V), upright = ~737 (~2.38 V).")
    print("   t(s)   adc    volt  valid   enc  state")
    samples = []
    latest = None
    start = time.monotonic()
    next_print = start
    try:
        while time.monotonic() - start < args.seconds:
            chunk = board.read_chunk(port)
            if chunk:
                rows = stream.feed(chunk)
                if rows:
                    latest = rows[-1]
                    samples.append(latest)
            now = time.monotonic()
            if latest is not None and now >= next_print:
                next_print += args.interval
                adc = latest['adc_raw10']
                print(f"  {now - start:5.1f}  {adc:4d}  {adc / 1023 * args.vref:6.2f}  "
                      f"{'yes' if latest['adc_valid'] else 'no ':5s}  {latest['enc_count']:4d}  "
                      f"{latest['state']}", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()

    if not samples:
        print("No telemetry received: is the board powered and the port correct?")
        return
    codes = [r['adc_raw10'] for r in samples]
    good = sum(1 for c in codes if args.low <= c <= args.high)
    valid = sum(1 for r in samples if r['adc_valid'])
    print(f"\n{len(samples)} frames: adc {min(codes)}..{max(codes)} "
          f"({min(codes) / 1023 * args.vref:.2f}..{max(codes) / 1023 * args.vref:.2f} V), "
          f"in {args.low}..{args.high}: {100 * good / len(samples):.1f}%, "
          f"adc_valid: {100 * valid / len(samples):.1f}%")
    if good == 0:
        print("Diagnosis: no signal. Check the 12 V supply and the three-wire connector.")
    elif good < 0.9 * len(samples):
        print("Diagnosis: intermittent contact. Wiggle one joint at a time to find it.")
    else:
        print("Diagnosis: the angle signal is steady.")


if __name__ == '__main__':
    main()
