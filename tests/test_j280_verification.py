import importlib.util
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest import mock


def load_verifier():
    path=Path(__file__).resolve().parents[1]/'scripts/j280_verify_measurement.py'
    spec=importlib.util.spec_from_file_location('j280_verify_measurement',path)
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def timing_html(tns='0.000', main_fmax='120.000', adc=True, hold_count=0, hold_slack='0.2'):
    names=['clk_50m','adc_5m'] if adc else ['clk_50m']
    frequencies=[('50.000',main_fmax),('5.000','8.000')]
    clocks=''.join('<tr><td>1</td><td>'+name+'</td><td>'+frequencies[i][0]+
                   '(MHz)</td><td>'+frequencies[i][1]+'(MHz)</td><td>1</td><td>TOP</td></tr>'
                   for i,name in enumerate(names))
    slacks=''.join('<tr><td>'+name+'</td><td>'+kind+'</td><td>'+tns+
                  '</td><td>0</td></tr>' for name in names for kind in ('Setup','Hold'))
    summary=('<h2><a name="STA_Tool_Run_Summary">STA</a></h2><table>'+
             '<tr><td>Numbers of Setup Violated Endpoints</td><td>0</td></tr>'+
             '<tr><td>Numbers of Hold Violated Endpoints</td><td>'+str(hold_count)+'</td></tr></table>')
    details=('<h3><a name="Setup_Slack_Table">Setup</a></h3><h4>report</h4><table>'+
             '<tr><td>1</td><td>0.2</td></tr></table>'+
             '<h3><a name="Hold_Slack_Table">Hold</a></h3><h4>report</h4><table>'+
             '<tr><td>1</td><td>'+hold_slack+'</td></tr></table>')
    return (summary+'<h2><a name="Max_Frequency_Report">Max</a></h2><table>'+clocks+'</table>'+
            '<h2><a name="Total_Negative_Slack_Report">TNS</a></h2><table>'+slacks+'</table>'+details)


class VerificationTests(unittest.TestCase):
    def test_requires_both_clocks_and_all_four_slack_rows(self):
        v=load_verifier()
        result=v.check_timing_report(timing_html())
        self.assertEqual(result['constraints_mhz'],{'clk_50m':50.0,'adc_5m':5.0})
        self.assertEqual(len(result['tns_ns']),4)

    def test_rejects_negative_slack_even_after_tool_exit_zero(self):
        with self.assertRaisesRegex(ValueError,'slack'):
            load_verifier().check_timing_report(timing_html(tns='-0.001'))

    def test_rejects_missing_adc_clock(self):
        with self.assertRaisesRegex(ValueError,'clock'):
            load_verifier().check_timing_report(timing_html(adc=False))

    def test_rejects_clock_that_cannot_meet_constraint(self):
        with self.assertRaisesRegex(ValueError,'frequency'):
            load_verifier().check_timing_report(timing_html(main_fmax='39.152'))

    def test_rejects_global_hold_violations_hidden_by_tns_summary(self):
        with self.assertRaisesRegex(ValueError,'violat'):
            load_verifier().check_timing_report(timing_html(hold_count=2))

    def test_rejects_negative_detail_slack_even_with_zero_summaries(self):
        with self.assertRaisesRegex(ValueError,'slack'):
            load_verifier().check_timing_report(timing_html(hold_slack='-0.169'))

    def test_noop_zero_exit_build_cannot_certify_old_artifacts(self):
        v=load_verifier()
        with tempfile.TemporaryDirectory() as directory:
            base=Path(directory)
            tools=base/'bin';tools.mkdir()
            for name in ('iverilog.exe','vvp.exe'): (tools/name).touch()
            old=base/'build/gowin/impl/pnr';old.mkdir(parents=True)
            (old/'j280_measure.fs').write_bytes(b'old artifact')
            (old/'j280_measure_tr_content.html').write_text(timing_html(),encoding='utf-8')
            fake_log='Ran 1 test in 0.01s\nOK\n'+''.join('PASS '+top+'\n' for top in (
                'tb_adc_capture','tb_encoder','tb_frame_tx','tb_uart_tx','tb_measure_top','tb_output_age'))
            with mock.patch.object(v,'_run',return_value=fake_log), \
                    mock.patch.object(v,'_check_wire_fixture',return_value={'frames_valid':5}), \
                    contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                status=v.main(['--build','--build-dir',str(base/'build'),'--iverilog-bin',str(tools)])
            self.assertNotEqual(status,0,'old bitstream was accepted as a new build')
            self.assertFalse((base/'build/verification.json').exists())


if __name__=='__main__':
    unittest.main()
