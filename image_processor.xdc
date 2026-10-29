## Arty A7-35T pin assignments
## Part: xc7a35ticsg324-1L (as shipped by Digilent). xc7a35tcsg324-1 also works.

## 100 MHz clock
set_property -dict { PACKAGE_PIN E3 IOSTANDARD LVCMOS33 } [get_ports { CLK100MHZ }]
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { CLK100MHZ }]

## Switches SW0 (negative), SW1 (auto white balance), SW2 (auto exposure)
set_property -dict { PACKAGE_PIN A8  IOSTANDARD LVCMOS33 } [get_ports { sw[0] }]
set_property -dict { PACKAGE_PIN C11 IOSTANDARD LVCMOS33 } [get_ports { sw[1] }]
set_property -dict { PACKAGE_PIN C10 IOSTANDARD LVCMOS33 } [get_ports { sw[2] }]

## LEDs LD4..LD7 (green)
set_property -dict { PACKAGE_PIN H5  IOSTANDARD LVCMOS33 } [get_ports { led[0] }]
set_property -dict { PACKAGE_PIN J5  IOSTANDARD LVCMOS33 } [get_ports { led[1] }]
set_property -dict { PACKAGE_PIN T9  IOSTANDARD LVCMOS33 } [get_ports { led[2] }]
set_property -dict { PACKAGE_PIN T10 IOSTANDARD LVCMOS33 } [get_ports { led[3] }]

## USB-UART (FT2232HQ)
## Digilent names these from the FTDI chip's point of view, so:
##   A9  = "uart_txd_in"  : data coming FROM the PC  -> this is the FPGA's RX input
##   D10 = "uart_rxd_out" : data going  TO   the PC  -> this is the FPGA's TX output
## (the earlier file had these two swapped, so the FPGA never heard the PC)
set_property -dict { PACKAGE_PIN A9  IOSTANDARD LVCMOS33 } [get_ports { uart_rx }]
set_property -dict { PACKAGE_PIN D10 IOSTANDARD LVCMOS33 } [get_ports { uart_tx }]

## Configuration voltage (removes the CFGBVS DRC warning)
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
