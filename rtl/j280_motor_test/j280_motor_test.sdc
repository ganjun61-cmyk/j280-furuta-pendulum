create_clock -name clk_50m -period 20.000 [get_ports {clk_50m}]
create_generated_clock -name adc_5m -source [get_ports {clk_50m}] -divide_by 10 [get_ports {adc_clk}]
# Engineering budget, NOT a datasheet guaranteed tOD maximum. Hardware check required.
# Includes estimated clock-output, board round trip and ADC data propagation.
set_input_delay -clock adc_5m -max 60.000 [get_ports {adc_data[*] adc_otr}]
set_input_delay -clock adc_5m -min 0.000 [get_ports {adc_data[*] adc_otr}]
# Only asynchronous board inputs to their first receiving flop, not ADC data.
set_false_path -from [get_ports {rst_n enc1_a enc1_b uart_rx key_sw1 key_sw2 dip_sw1}]
