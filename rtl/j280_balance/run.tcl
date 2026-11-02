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
add_file j280_balance_guard.v
add_file j280_balance_pwm.v
add_file j280_balance_top.v
add_file j280_balance.cst
add_file j280_balance.sdc
set_option -top_module j280_balance_top
set_option -output_base_name j280_balance
run all
