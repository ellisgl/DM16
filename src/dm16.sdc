// 50 MHz board oscillator; the PLL output clock is derived automatically
create_clock -name clk_in -period 20 -waveform {0 10} [get_ports {clk_in}]
