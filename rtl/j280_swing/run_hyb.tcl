set_device -name GW2A-55C GW2A-LV55PG484C8/I7
add_file j280_adc_capture.v
add_file j280_encoder_filtered.v
add_file j280_uart_tx.v
add_file j280_signed_divider.v
add_file j280_raw_estimator.v
add_file j280_state_packet_tx.v
add_file j280_uart_rx.v
add_file j280_local_inputs.v
add_file j280_balance_cmd_rx.v
add_file j280_balance_core.v
add_file j280_balance_pwm.v
add_file j280_alpha_svf.v
add_file j280_keypad.v
add_file j280_swing_tables.v
add_file j280_swing_core.v
add_file j280_traj.v
add_file j280_aux_inputs.v
add_file j280_swing_guard.v
add_file j280_phase_lut16.v
add_file j280_swing_hyb_top.v
add_file j280_swing.cst
add_file j280_swing.sdc
set_option -top_module j280_swing_hyb_top
# SW4 (T20) is a dual-purpose SSPI pin; the board is programmed over JTAG.
set_option -use_sspi_as_gpio 1
# Timing-driven placement: closes the 50 MHz clock with the v6 logic (2026-10-19).
set_option -place_option 2
set_option -output_base_name j280_swing
run all
