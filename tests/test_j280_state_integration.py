"""State firmware UART pin integration; all motors remain passive."""
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from j280_state_telemetry import FrameStream
from test_j280_state_telemetry import fixture


class StateIntegrationTests(unittest.TestCase):
    def simulate(self, top, sources, fixture_payload=None):
        base=Path(os.environ.get('LOCALAPPDATA',tempfile.gettempdir()))/'J280Build/state-integration-tests'
        base.mkdir(parents=True,exist_ok=True)
        stage=Path(tempfile.mkdtemp(prefix='run-',dir=base))
        for source in sources:
            self.assertTrue(source.is_file(),f'Implement {source.relative_to(ROOT)}')
            shutil.copy2(source,stage/source.name)
        if fixture_payload is not None:
            (stage/'payload.hex').write_text(fixture_payload[::-1].hex()+'\n',encoding='ascii')
        from test_j280_motor_test_rtl import BIN as binary
        env=dict(os.environ,PATH=str(binary)+os.pathsep+os.environ.get('PATH',''))
        commands=[[binary/'iverilog.exe','-g2001','-Wall','-s',top,'-o','test.vvp',
                   *[p.name for p in sources]], [binary/'vvp.exe','test.vvp']]
        for index,command in enumerate(commands):
            r=subprocess.run([str(x) for x in command],cwd=stage,env=env,
                             capture_output=True,timeout=120)
            text=(r.stdout+r.stderr).decode('utf-8',errors='replace')
            (stage/f'job-{index}.log').write_text(text,encoding='utf-8')
            self.assertEqual(r.returncode,0,text)
        self.assertIn('PASS '+top,text)
        return (stage/'wire.bin').read_bytes()

    def test_packet_transmitter_crc_immutability_and_overflow(self):
        frame=fixture()
        wire=self.simulate('tb_state_packet_tx',[
            ROOT/'sim/j280_state/tb_state_packet_tx.v',
            ROOT/'rtl/j280_state/j280_state_packet_tx.v',
            ROOT/'rtl/j280_measure/j280_uart_tx.v'],frame[2:-2])
        self.assertEqual(wire,frame)
        rows=FrameStream().feed(wire)
        self.assertEqual(len(rows),1)
        self.assertEqual(rows[0]['encoder_velocity_count_s'],200)

    def test_top_uart_pin_end_to_end_passive(self):
        rtl=ROOT/'rtl'
        wire=self.simulate('tb_state_top',[
            ROOT/'sim/j280_state/tb_state_top.v',
            rtl/'j280_state/j280_state_top.v', rtl/'j280_state/j280_raw_estimator.v',
            rtl/'j280_state/j280_signed_divider.v', rtl/'j280_state/j280_state_packet_tx.v',
            rtl/'j280_measure/j280_adc_capture.v', rtl/'j280_measure/j280_encoder.v',
            rtl/'j280_measure/j280_uart_tx.v'])
        parser=FrameStream()
        rows=parser.feed(wire)
        s=parser.stats
        self.assertEqual(len(rows),9)
        self.assertEqual((s['frames_invalid'],s['discarded_bytes'],s['packet_seq_missed'],
                          s['continuity_errors'],s['reset_or_reorder']),(0,0,0,0,0))
        self.assertEqual([r['packet_seq'] for r in rows],list(range(9)))
        self.assertEqual([r['tick_us_unwrapped'] for r in rows],[2000+10000*i for i in range(9)])
        self.assertTrue(all(not r['controller_ready'] for r in rows))
        self.assertTrue(all(r['reason'] != 'input_overrun' for r in rows))
        self.assertEqual((rows[0]['status'],rows[0]['reason']),('warming','segment_start'))
        for r in rows[3:6]:
            self.assertEqual((r['status'],r['segment_id']),('ready',1))
            self.assertEqual(r['encoder_velocity_count_s'],2000)
            self.assertEqual(r['adc_velocity_code_s'],1000)
        r=rows[6]
        self.assertEqual((r['status'],r['reason'],r['fault_bits']),('invalid','invalid_flags',1))
        self.assertIsNone(r['encoder_velocity_count_s'])
        self.assertEqual((rows[7]['segment_id'],rows[7]['status']),(2,'warming'))
        self.assertEqual((rows[8]['status'],rows[8]['encoder_velocity_count_s'],
                          rows[8]['adc_velocity_code_s'],rows[8]['enc_count']),('ready',0,0,80))


if __name__=='__main__':unittest.main()
