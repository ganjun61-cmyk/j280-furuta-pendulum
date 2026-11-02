"""v4 physical UART receive -> immutable 188-byte payload cross-end tests."""
from pathlib import Path
import struct
import sys
import unittest

from test_j280_motor_test_rtl import simulate,rx_script

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import j280_balance_protocol as wire


def config():
    return dict(config_id=0x1234,algorithm_id=1,run_ms=1000,keepalive_ms=200,
        travel_limit_count=100,speed_limit_count_s=5000,adc_speed_limit_code_s=10000,
        max_compare=1000,hold_ms=200,wait_ms=5000,upright_zero_q16=735<<16,
        enter_code=8,exit_code=56,comp_compare=175,blend_compare=25,
        comp_gain_q20=7<<20,coefficients=[0x5aa5,-0x5aa5]+[0]*30,sample_period_us=2000)


class BalanceCommandReceiveTests(unittest.TestCase):
    sources=[ROOT/'rtl/j280_motor_test/j280_uart_rx.v',
             ROOT/'rtl/j280_balance/j280_balance_cmd_rx.v',
             ROOT/'sim/j280_balance/tb_balance_cmd_rx.v']

    def run_rx(self,script,**params):
        _,run=simulate('tb_balance_cmd_rx',self.sources,{'rx_script.txt':script},params)
        lines=(run/'rx_log.txt').read_text().splitlines()
        frames=[(bytes.fromhex(x.split()[1])[::-1],int(x.split()[2],16))
                for x in lines if x.startswith('V ')]
        return dict(frames=frames,frame_errors=lines.count('X'),byte_errors=lines.count('R'),
                    received=bytes(int(x[2:],16) for x in lines if x.startswith('D ')))

    def test_full_config_and_controls_back_to_back(self):
        packets=[wire.build_command('config',config()),wire.build_command('keepalive'),wire.build_command('abort')]
        source=b''.join(packets)
        out=self.run_rx(rx_script(('B',source,1000)))
        self.assertEqual(out['received'],source)
        self.assertEqual(out['frame_errors'],0)
        self.assertEqual(out['frames'],[(p[2:190],struct.unpack_from('<H',p,190)[0]) for p in packets])

    def test_crc_version_noise_and_embedded_sync_recover(self):
        good=wire.build_command('config',config())
        corrupt=bytearray(good);corrupt[191]^=1
        version=bytearray(good);version[2]=3
        version[190:]=wire.crc16_ccitt(version[2:190]).to_bytes(2,'little')
        source=b'\x00\xa5\x11'+bytes(corrupt)+bytes(version)+b'\xa5'+good
        out=self.run_rx(rx_script(('B',source,1000)))
        self.assertEqual(out['frames'],[(good[2:190],int.from_bytes(good[190:],'little'))])
        self.assertEqual(out['frame_errors'],2)

    def test_timeout_and_bad_stop_drop_incomplete_command(self):
        good=wire.build_command('abort')
        out=self.run_rx(rx_script(('B',good[:70],1000),('B',good[70:],1100000),
             ('B',good[:5],50000),('F',b'\0',0),('B',good[5:],0),('B',good,50000)))
        self.assertEqual(len(out['frames']),1)
        self.assertEqual(out['byte_errors'],1)
        self.assertGreaterEqual(out['frame_errors'],2)

    def test_uart_glitch_and_baud_error_tolerance(self):
        good=wire.build_command('keepalive')
        for period in (4253,4427):
            with self.subTest(period=period):
                out=self.run_rx(rx_script(('G',1500,1000),('B',good,20000)),BIT_NS=period)
                self.assertEqual(len(out['frames']),1)
                self.assertEqual(out['frame_errors'],0)


if __name__=='__main__':unittest.main()
