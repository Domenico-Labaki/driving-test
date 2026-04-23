# ============================================================================
# driving_test.sdc
# Timing constraints for TimeQuest.
# ============================================================================

# Primary clock: 50 MHz board oscillator
create_clock -name {CLOCK_50} -period 20.000 [get_ports {CLOCK_50}]

# Derived clocks (clk_25, clk_game) -- automatic derivation
derive_pll_clocks
derive_clock_uncertainty

# Relax timing on async user inputs (switches/buttons)
set_false_path -from [get_ports {KEY[*]}] -to *
set_false_path -from [get_ports {SW[*]}]  -to *

# Relax timing on asynchronous outputs to slow peripherals
set_false_path -from * -to [get_ports {LEDR[*]}]
set_false_path -from * -to [get_ports {LEDG[*]}]
set_false_path -from * -to [get_ports {HEX0[*] HEX1[*] HEX2[*] HEX3[*] HEX4[*] HEX5[*]}]
set_false_path -from * -to [get_ports {LCD_DATA[*] LCD_EN LCD_RS LCD_RW LCD_ON LCD_BLON}]
