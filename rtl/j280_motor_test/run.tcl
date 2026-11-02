set_device -name GW2A-55C GW2A-LV55PG484C8/I7
add_file j280_adc_capture.v
add_file j280_encoder.v
add_file j280_uart_tx.v
add_file j280_signed_divider.v
add_file j280_raw_estimator.v
add_file j280_state_packet_tx.v
add_file j280_uart_rx.v
add_file j280_cmd_rx.v
add_file j280_local_inputs.v
add_file j280_motor_guard.v
add_file j280_pwm_pulse.v
add_file j280_motor_test_top.v
add_file j280_motor_test.cst
add_file j280_motor_test.sdc
set_option -top_module j280_motor_test_top
set_option -output_base_name j280_motor_test
run all
