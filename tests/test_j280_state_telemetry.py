"""Independent fixtures for passive state telemetry v2; never opens hardware."""
import csv
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from j280_measurement import crc16_ccitt

SCRIPT = ROOT/'scripts/j280_state_telemetry.py'
telemetry = None
if SCRIPT.exists():
    spec = importlib.util.spec_from_file_location('state_telemetry', SCRIPT)
    telemetry = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(telemetry)


def fixture(packet=0, seq=None, tick=None, quality=7, reason=None, **changes):
    tick = 20000+packet*10000 if tick is None else tick
    seq = packet*5 if seq is None else seq
    reason = (0 if quality == 7 else 1 if quality & 1 else 2) if reason is None else reason
    values = dict(segment_id=1, tick=tick, position=24 if quality & 1 else 0,
                  enc_velocity=200*65536 if quality & 2 else 0,
                  adc_velocity=-100*65536 if quality & 4 else 0,
                  enc_span=10000 if quality & 2 else 0,
                  adc_span=10000 if quality & 4 else 0,
                  adc_time=tick-2 if quality & 1 else 0,
                  enc_mid=2*tick-10000 if quality & 2 else 0,
                  adc_mid=2*(tick-2)-10000 if quality & 4 else 0,
                  enc_age=10000 if quality & 2 else 0,
                  adc_age=10004 if quality & 4 else 0,
                  estimator_version=1, window=10000, period=2000,
                  max_age=100, adc_step=64, enc_step=64, telemetry_period=10000,
                  raw=500, age=2, encoder=24, flags=3 if quality & 1 else 2,
                  fault=0 if quality & 1 else 1, mode=0, pwm=0, pwm_period=0)
    values.update(changes)
    v = values
    payload = bytearray(124)
    struct.pack_into('<BBIIHHiHHBBHH', payload, 0, 2, v['flags'], seq,
                     tick % 2**32, v['raw'], v['age'], v['encoder'],
                     v['pwm'], v['pwm_period'], v['mode'], 2, v['fault'], 65535)
    struct.pack_into('<IQqiiIIQQQIIBBHHHHHHIH', payload, 28,
                     v['segment_id'], v['tick'], v['position'], v['enc_velocity'],
                     v['adc_velocity'], v['enc_span'], v['adc_span'], v['adc_time'],
                     v['enc_mid'], v['adc_mid'], v['enc_age'], v['adc_age'], quality,
                     reason, v['estimator_version'], v['window'], v['period'],
                     v['max_age'], v['adc_step'], v['enc_step'], packet,
                     v['telemetry_period'])
    return b'\xa5\x5a'+payload+struct.pack('<H',crc16_ccitt(payload))


def changed(frame, offset, value):
    raw = bytearray(frame)
    raw[2+offset] = value
    raw[-2:] = struct.pack('<H',crc16_ccitt(raw[2:-2]))
    return bytes(raw)


class StateTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(telemetry, 'Implement scripts/j280_state_telemetry.py')

    def test_ready_frame_exact_fields_units_and_timestamps(self):
        rows = telemetry.FrameStream().feed(fixture())
        self.assertEqual(len(rows),1)
        r = rows[0]
        self.assertEqual((r['seq'],r['packet_seq'],r['tick_us_unwrapped']), (0,0,20000))
        self.assertEqual((r['adc_raw10'],r['enc_count']), (500,24))
        self.assertEqual(r['encoder_velocity_count_s'],200)
        self.assertEqual(r['adc_velocity_code_s'],-100)
        self.assertEqual(r['adc_position_time_us'],19998)
        self.assertEqual(r['encoder_velocity_midpoint_us'],15000)
        self.assertEqual(r['adc_velocity_midpoint_us'],14998)
        self.assertEqual(r['adc_velocity_age_us'],5002)
        self.assertEqual(r['status'],'ready')
        self.assertFalse(r['controller_ready'])

    def test_invalid_and_independent_warming_have_blank_estimates(self):
        for quality in (0,1,3,5):
            with self.subTest(quality=quality):
                r = telemetry.FrameStream().feed(fixture(quality=quality))[0]
                self.assertEqual(r['position_valid'],bool(quality&1))
                self.assertEqual(r['encoder_velocity_valid'],bool(quality&2))
                self.assertEqual(r['adc_velocity_valid'],bool(quality&4))
                if not quality&1:
                    self.assertIsNone(r['encoder_position_count'])
                    self.assertIsNone(r['adc_position_time_us'])
                if not quality&2:
                    self.assertIsNone(r['encoder_velocity_count_s'])
                if not quality&4:
                    self.assertIsNone(r['adc_velocity_code_s'])

    def test_partial_chunks_noise_crc_and_recovery(self):
        parser = telemetry.FrameStream()
        bad = bytearray(fixture());bad[45] ^= 1
        wire = b'noise'+bytes(bad)+fixture(packet=1)+fixture(packet=2)
        rows=[]
        for i in range(0,len(wire),7):rows += parser.feed(wire[i:i+7])
        self.assertEqual([r['packet_seq'] for r in rows],[1,2])
        self.assertGreaterEqual(parser.stats['crc_errors'],1)
        self.assertEqual(parser.buffered_bytes,0)

    def test_rejects_reserved_version_config_and_contradictory_quality(self):
        cases = [changed(fixture(),116,1), changed(fixture(),0,1),
                 changed(fixture(),96,8), changed(fixture(),97,2),
                 changed(fixture(),98,9), fixture(window=0), fixture(period=0),
                 fixture(quality=0,position=24),fixture(quality=1,enc_velocity=1),
                 fixture(enc_mid=3),fixture(adc_age=3),fixture(enc_span=9000),
                 fixture(position=25), fixture(adc_time=19997),
                 fixture(quality=2,reason=1),fixture(raw=1024)]
        for frame in cases:
            with self.subTest(frame=frame.hex()[:60]):
                parser=telemetry.FrameStream()
                self.assertEqual(parser.feed(frame),[])
                self.assertEqual(parser.stats['frames_invalid'],1)

    def test_decimated_raw_sequence_is_not_four_lost_packets(self):
        parser=telemetry.FrameStream()
        rows=parser.feed(fixture()+fixture(1)+fixture(2))
        self.assertEqual([r['seq'] for r in rows],[0,5,10])
        self.assertTrue(all(r['packet_seq_gap']==0 for r in rows))
        self.assertTrue(all(not r['sample_seq_continuity_error'] for r in rows))
        self.assertEqual(parser.stats['packet_seq_missed'],0)

    def test_lost_duplicate_reordered_and_reset_packets_are_flagged(self):
        parser=telemetry.FrameStream()
        rows=parser.feed(fixture()+fixture(2)+fixture(2)+fixture(1)+fixture(3))
        self.assertEqual(rows[1]['packet_seq_gap'],1)
        self.assertTrue(rows[2]['duplicate'])
        self.assertTrue(rows[3]['out_of_order'])
        self.assertFalse(rows[4]['out_of_order'])
        r=parser.feed(fixture(0,seq=0,tick=2000,quality=1,reason=12))[0]
        self.assertTrue(r['reset_or_reorder'])
        self.assertGreater(r['epoch'],rows[0]['epoch'])

    def test_wrap_and_time_or_sample_discontinuity(self):
        p=telemetry.FrameStream()
        first=fixture(2**32-1,seq=2**32-3,tick=2**32-4000)
        second=fixture(0,seq=2,tick=2**32+6000)
        rows=p.feed(first+second)
        self.assertFalse(rows[1]['reset_or_reorder'])
        self.assertEqual(rows[1]['packet_seq_gap'],0)
        self.assertFalse(rows[1]['sample_seq_continuity_error'])
        self.assertFalse(rows[1]['time_continuity_error'])
        p=telemetry.FrameStream();p.feed(fixture())
        r=p.feed(fixture(1,seq=7,tick=32000))[0]
        self.assertTrue(r['sample_seq_continuity_error'])
        self.assertTrue(r['time_continuity_error'])

    def test_signed_positions_and_fractional_midpoints(self):
        r=telemetry.FrameStream().feed(fixture(position=-2**31-1,encoder=2**31-1,
                     adc_span=10001,adc_mid=29995,adc_age=10005))[0]
        self.assertEqual(r['encoder_position_count'],-2**31-1)
        self.assertEqual(r['adc_velocity_midpoint_us'],14997.5)

    def test_cli_decode_preserves_inputs_and_refuses_output_reuse(self):
        with tempfile.TemporaryDirectory() as d:
            source=Path(d)/'wire.bin';output=Path(d)/'rows.csv'
            source.write_bytes(fixture()+fixture(1));before=source.read_bytes()
            command=[sys.executable,str(SCRIPT),'decode','--input',str(source),
                     '--output',str(output)]
            result=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads(result.stdout)['frames_valid'],2)
            with output.open(encoding='utf-8',newline='') as f:rows=list(csv.DictReader(f))
            self.assertEqual(len(rows),2)
            self.assertEqual(rows[0]['controller_ready'],'False')
            self.assertEqual(rows[0]['encoder_velocity_count_s'],'200.0')
            original=output.read_bytes()
            result=subprocess.run(command,capture_output=True,text=True)
            self.assertEqual(result.returncode,2)
            self.assertEqual(output.read_bytes(),original)
            self.assertEqual(source.read_bytes(),before)


if __name__=='__main__':unittest.main()
