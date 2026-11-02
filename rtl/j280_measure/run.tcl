set_device -name GW2A-55C GW2A-LV55PG484C8/I7
add_file j280_adc_capture.v
add_file j280_encoder.v
add_file j280_uart_tx.v
add_file j280_frame_tx.v
add_file j280_measure_top.v
add_file j280_measure.cst
add_file j280_measure.sdc
set_option -top_module j280_measure_top
set_option -output_base_name j280_measure
run all
