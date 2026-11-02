set_device -name GW2A-55C GW2A-LV55PG484C8/I7
add_file j280_adc_capture.v
add_file j280_encoder.v
add_file j280_uart_tx.v
add_file j280_signed_divider.v
add_file j280_raw_estimator.v
add_file j280_state_packet_tx.v
add_file j280_state_top.v
add_file j280_state.cst
add_file j280_state.sdc
set_option -top_module j280_state_top
set_option -output_base_name j280_state
run all
