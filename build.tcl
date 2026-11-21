# DM16 build for the Tang Console 138K. From the project directory:
#   gw_sh build.tcl     ->  impl/pnr/DM16.fs
set_device GW5AST-LV138PG484AC1/I0 -device_version B
add_file src/dm16_top.v
add_file src/dm16_pll.v
add_file src/pll_init.v
add_file src/dm16_core.v
add_file src/dm16_uart_seq.v
add_file src/dm16_cluster.v
add_file src/dm16_pe.v
add_file src/dm16_bf16.v
add_file src/hypercube_router.v
add_file src/dm16.cst
add_file src/dm16.sdc
set_option -output_base_name DM16
set_option -top_module dm16_top
set_option -verilog_std v2001
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
set_option -use_done_as_gpio 1
set_option -use_cpu_as_gpio 1
set_option -use_ready_as_gpio 1
# Keep JTAG as JTAG so the board can be reloaded without a power cycle
set_option -use_jtag_as_gpio 0
set_option -vccx 1.8
set_option -vcc 0.9
set_option -timing_driven 1
set_option -correct_hold_violation 1
run all
