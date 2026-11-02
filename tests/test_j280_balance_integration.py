"""True UART pins and B PWM pins for the balance top, with source provenance."""
import csv
import hashlib
import json
import os
from pathlib import Path
import shutil
import sys
import unittest

from test_j280_motor_test_rtl import simulate
from test_j280_balance_cmd_rx_rtl import config

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import j280_balance_protocol as wire
from j280_balance_math import design_coefficients

SOURCES=[ROOT/'rtl/j280_measure'/n for n in ('j280_adc_capture.v','j280_uart_tx.v')]
SOURCES += [ROOT/'rtl/j280_state'/n for n in ('j280_raw_estimator.v','j280_signed_divider.v','j280_state_packet_tx.v')]
SOURCES += [ROOT/'rtl/j280_motor_test'/n for n in ('j280_uart_rx.v','j280_local_inputs.v')]
SOURCES += [ROOT/'rtl/j280_balance'/n for n in ('j280_encoder_filtered.v','j280_balance_cmd_rx.v','j280_balance_core.v',
            'j280_balance_pwm.v','j280_balance_guard.v','j280_balance_top.v')]
SOURCES += [ROOT/'sim/j280_balance/tb_balance_top.v']


def fixture_commands():
    values=config();values.update(coefficients=design_coefficients(),run_ms=50,exit_code=20)
    good=bytearray(wire.build_command('config',values))
    # Only this testbench lowers MIN_HOLD_MS. Production host validation rejects12ms.
    good[20:22]=(12).to_bytes(2,'little')
    good[190:]=wire.crc16_ccitt(good[2:190]).to_bytes(2,'little')
    bad=bytearray(good);bad[164:168]=(129<<20).to_bytes(4,'little',signed=True)
    bad[190:]=wire.crc16_ccitt(bad[2:190]).to_bytes(2,'little')
    return [bytes(bad),bytes(good)]


class BalanceIntegrationTests(unittest.TestCase):
    def scenario(self,case,reason):
        packets=fixture_commands()
        commands='\n'.join(p[::-1].hex() for p in packets)+'\n'
        _,run=simulate('tb_balance_top',SOURCES,{'commands.hex':commands},plusargs=(f'+CASE={case}',),timeout=240)
        data=(run/'wire.bin').read_bytes();stream=wire.FrameStream();rows=stream.feed(data)
        self.assertTrue(rows)
        for key in ('frames_invalid','discarded_bytes','incomplete_tail_bytes','packet_seq_missed',
                    'duplicates','out_of_order','resets','reset_or_reorder','continuity_errors'):
            self.assertEqual(stream.stats[key],0,(key,stream.stats))
        self.assertEqual([r['packet_seq'] for r in rows],list(range(len(rows))))
        self.assertEqual([r['raw_seq'] for r in rows],[5*i for i in range(len(rows))])
        self.assertEqual([r['tick_us'] for r in rows],[2000+10000*i for i in range(len(rows))])
        self.assertIn('CAPTURE',{r['state'] for r in rows})
        self.assertIn('BALANCE',{r['state'] for r in rows})
        self.assertTrue(any(r['output_enabled'] for r in rows))
        self.assertTrue(any(r['rx_status']&15==5 and not r['config_loaded'] for r in rows))
        for r in rows:
            if r['config_loaded']:
                self.assertEqual((r['config_id'],r['config_crc']),(0x1234,int.from_bytes(packets[1][190:],'little')))
        self.assertEqual(rows[-1]['stop_reason'],reason)
        self.assertEqual(rows[-1]['state'],'SAFE' if case==0 else 'FAULT')
        self.assertFalse(rows[-1]['output_enabled'])
        self.assertFalse(rows[-1]['config_loaded'])
        with (run/'pins.csv').open(encoding='ascii',newline='') as source:
            pinrows=[tuple(map(int,x)) for x in csv.reader(source)]
        self.assertTrue(any(in1 or in2 for _,pwm,in1,in2 in pinrows))
        if case==0:
            self.assertTrue(any(in2 for _,pwm,in1,in2 in pinrows))
            self.assertTrue(any(in1 for _,pwm,in1,in2 in pinrows))
        with (run/'snapshots.csv').open(encoding='ascii',newline='') as source:
            snapshotrows=[tuple(map(int,x)) for x in csv.reader(source)]
        self.assertEqual([x[2] for x in snapshotrows],list(range(len(snapshotrows))))
        self.assertTrue(all(b[1]-a[1]==2000 for a,b in zip(snapshotrows,snapshotrows[1:])))
        result=dict(case=case,frames=len(rows),stats=stream.stats,final_reason=reason,
                    simulated_min_hold_ms=2,command_hold_ms=12,
                    wire_sha256=hashlib.sha256(data).hexdigest(),
                    source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCES})
        (run/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        evidence=os.environ.get('J280_BALANCE_EVIDENCE_DIR')
        if evidence:
            target=Path(evidence)/('stop' if case==0 else 'angle')
            target.mkdir(parents=True,exist_ok=True)
            for name in ('commands.hex','wire.bin','pins.csv','snapshots.csv','events.csv','result.json','compile.log','simulation.log'):
                shutil.copy2(run/name,target/name)

    def test_top_local_start_bidirectional_feedback_and_stop(self):self.scenario(0,'stop_key')
    def test_top_angle_fault_latches_and_clears_configuration(self):self.scenario(1,'angle_limit')


if __name__=='__main__':unittest.main()
