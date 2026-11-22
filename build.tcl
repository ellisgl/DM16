# DM16 build. From the project directory:
#   ./build.sh                ->  impl/pnr/DM16.fs                   (Tang Console 138K)
#   ./build.sh nano20k        ->  build/nano20k/impl/pnr/DM16.fs     (Tang Nano 20K)
#   ./build.sh nano9k         ->  build/nano9k/impl/pnr/DM16.fs      (Tang Nano 9K)
# build.sh sets DM16_BOARD and runs gw_sh in the board's directory, with a
# timestamped build log. The board's geometry is set in its top-level file.
set root [file normalize [file dirname [info script]]]
set board console138k
if {[info exists ::env(DM16_BOARD)]} { set board $::env(DM16_BOARD) }

set common {dm16_core.v dm16_fb_writer.v dm16_uart_seq.v dm16_cluster.v dm16_pe.v dm16_bf16.v hypercube_router.v}

switch $board {
    console138k {
        set_device GW5AST-LV138PG484AC1/I0 -device_version B
        set files {dm16_top.v dm16_pll.v pll_init.v dm16_hdmi.v dm16.cst dm16.sdc}
        set top dm16_top
    }
    nano20k {
        set_device GW2AR-LV18QN88C8/I7 -device_version C
        set files {boards/dm16_top_nano20k.v boards/tang_nano_20k.cst boards/dm16_27m.sdc}
        set top dm16_top_nano20k
    }
    nano9k {
        set_device GW1NR-LV9QN88PC6/I5 -device_version C
        set files {boards/dm16_top_nano9k.v boards/tang_nano_9k.cst boards/dm16_27m.sdc}
        set top dm16_top_nano9k
    }
    default {
        error "unknown board '$board' (console138k, nano20k or nano9k)"
    }
}

foreach f [concat $files $common] { add_file $root/src/$f }
set_option -output_base_name DM16
set_option -top_module $top
set_option -verilog_std v2001
set_option -use_mspi_as_gpio 1
set_option -use_sspi_as_gpio 1
# Keep JTAG as JTAG so the board can be reloaded without a power cycle
set_option -use_jtag_as_gpio 0
if {$board eq "console138k"} {
    set_option -use_done_as_gpio 1
    set_option -use_cpu_as_gpio 1
    set_option -use_ready_as_gpio 1
    set_option -vccx 1.8
    set_option -vcc 0.9
}
set_option -timing_driven 1
set_option -correct_hold_violation 1
run all
