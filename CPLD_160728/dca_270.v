module dca_270
(
//-----------------------------------------------------input  signals---------------------------------------------------------------
gpmc_cs3_n, gpmc_we_n, gpmc_oe_n, sa, fpga_clk, sys_reset_n, dio_in, pwr_12v_fail, clk_esam_35712mhz, cpu_txd5, 
sim_ncd, pwr_charge_over, pwr_discharge_over,
//-----------------------------------------------------output signals---------------------------------------------------------------
cpld_led_out, irq_cpld, io_led_rev, dio_out, cpld_amp_shdn, wdg_out, buzzer_out, rst_out_n, pwr_4g_en, pcie_rst_n,
pcie_wakeup_in, esam_clk, esam_rst, esam_on, cpu_rxd5, io_led_run, io_led_pwr, 
lcd_pwen, lcd_bl_en, lvds_pwen, io_led_charge, io_led_discharge,
//-----------------------------------------------------inout  signals---------------------------------------------------------------
sd, esam_data
);

`define SA_WIDTH	8
`define SD_WIDTH	8

input gpmc_cs3_n;
input gpmc_we_n;
input gpmc_oe_n;
input [`SA_WIDTH - 1:0] sa;
input fpga_clk;				//FPGA_CLK = 24MHz
input sys_reset_n;
input [1:0] dio_in;
input pwr_12v_fail;
input clk_esam_35712mhz;		//ESAM_CLK = 3.5712MHz
input cpu_txd5;
input sim_ncd;
input pwr_charge_over;
input pwr_discharge_over;

output cpld_led_out;
output [1:0] irq_cpld;
output [3:1] io_led_rev;
output io_led_charge;
output io_led_discharge;
output [1:0] dio_out;
output cpld_amp_shdn;
output wdg_out;
output buzzer_out;
output rst_out_n;
output pwr_4g_en;
output pcie_rst_n;
output pcie_wakeup_in;
output esam_clk;
output esam_rst;
output esam_on;
output cpu_rxd5;
output io_led_run;
output io_led_pwr;
output lcd_pwen;
output lcd_bl_en;
output lvds_pwen;

inout [`SD_WIDTH - 1:0] sd;
inout esam_data;

//---------------------------------------------------------------------------------------------------------------------------------
//	System logic 
//---------------------------------------------------------------------------------------------------------------------------------
parameter ETH_BANK = 1, UART_BANK = 2, MISC_BANK = 3;
wire [3:1] ngcs;
wire noe;
wire nwe;

assign ngcs = {gpmc_cs3_n, 2'b11};
assign nwe  = gpmc_we_n;
assign noe  = gpmc_oe_n;
assign rst_out_n = sys_reset_n;

//------------------------------------------------------------------------------------------------------------------------------
//	Devide fpga_clk(24MHz) to 1MHz 
//------------------------------------------------------------------------------------------------------------------------------
`define CLK_SUM		12
`define CLK_WIDTH 	4

reg [`CLK_WIDTH - 1:0] clk_cnt;
reg clk_1mhz_reg;
wire clk_1mhz;

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)                    clk_cnt <= `CLK_WIDTH'b0;
	else if(clk_cnt >= (`CLK_SUM - 1))  clk_cnt <= `CLK_WIDTH'b0;
	else                                clk_cnt <= clk_cnt + `CLK_WIDTH'b1;
end

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)                    clk_1mhz_reg <= 1'b0;
	else if(clk_cnt >= (`CLK_SUM - 1))  clk_1mhz_reg <= ~clk_1mhz_reg;
end

assign clk_1mhz = clk_1mhz_reg;

//---------------------------------------------------------------------------------------------------------------------------------
//	Watchdog logic, 8 bit data bus.  
//	Address offset: 0x0
//---------------------------------------------------------------------------------------------------------------------------------
`define WDG_SUM	23

wire wdg_cs;
reg [`WDG_SUM - 1:0] wdg_cnt;
reg wdg_en;
reg wdg_pwm;

assign wdg_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b0);

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)
		{wdg_pwm, wdg_en} <= 2'b0;
	else if(wdg_cs && !nwe)
		{wdg_pwm, wdg_en} <= sd[1:0];
end

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)
		wdg_cnt <= `WDG_SUM'b0;
	else
		wdg_cnt <= wdg_cnt + `WDG_SUM'b1;
end

assign wdg_out = wdg_en ? wdg_pwm : wdg_cnt[`WDG_SUM - 1];

//---------------------------------------------------------------------------------------------------------------------------------
//	Buzzer logic, 8 bit data bus.  
//	Address offset: 0x1
//---------------------------------------------------------------------------------------------------------------------------------
`define BUZZER_SUM	25
wire buzzer_cs;
reg buzzer_en;
reg [`BUZZER_SUM - 1:0] buzzer_cnt;

assign buzzer_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b1);

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)
		buzzer_en <= 1'b0;
	else if(buzzer_cs && !nwe)
		buzzer_en <= sd[0];
end

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)
		buzzer_cnt[`BUZZER_SUM - 1:20] <= 5'b11100;
	else if(!buzzer_cnt[`BUZZER_SUM - 1] && buzzer_en)
		buzzer_cnt[`BUZZER_SUM - 1:20] <= 5'b11100;
	else if(buzzer_cnt[`BUZZER_SUM - 1])
		buzzer_cnt <= buzzer_cnt - `BUZZER_SUM'b1;
end

assign buzzer_out = buzzer_cnt[14] || (!buzzer_cnt[`BUZZER_SUM - 1]);

//---------------------------------------------------------------------------------------------------------------------------------
//	IO LED logic, 8 bit data bus.  
//	Address offset: 0x2
//	io_led_reg[0]: io_led_run
//---------------------------------------------------------------------------------------------------------------------------------
wire io_led_out_cs;
reg  io_led_reg;
reg [2:0] io_led_rev_reg;

assign io_led_out_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b10);

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)
		{io_led_rev_reg, io_led_reg} <= 4'b0;
	else if(io_led_out_cs && !nwe)
		{io_led_rev_reg, io_led_reg} <= sd[3:0];
end

assign io_led_run    = ~io_led_reg;
assign io_led_pwr    = ~pwr_12v_fail;
assign io_led_rev    = {~io_led_rev_reg[2], ~io_led_rev_reg[1], ~io_led_rev_reg[0]};

//---------------------------------------------------------------------------------------------------------------------------------
//	This logic used to judge Super CAP's charge status or discharge status.
//---------------------------------------------------------------------------------------------------------------------------------
wire [2:0] pwr_status = {pwr_charge_over, pwr_discharge_over, pwr_12v_fail};
reg  [1:0] state_out;

always @(*)
begin
	case(pwr_status)
		3'b000:		state_out = 2'b10;				//discharge over;
		3'b001:		state_out = {wdg_cnt[`WDG_SUM - 1], 1'b1};	//charge start, but voltage < 5V
		3'b010:		state_out = {1'b1, wdg_cnt[`WDG_SUM - 1]};	//discharging, 5V < voltage > 10.5V
		3'b011:		state_out = {wdg_cnt[`WDG_SUM - 1], 1'b1};	//charging, 5V < voltage < 10.5V 
		3'b100:		state_out = 2'b11;				//No this state.
		3'b101:		state_out = 2'b11;				//No this state.
		3'b110:		state_out = {1'b1, wdg_cnt[`WDG_SUM - 1]};	//discharge start, and 10.5V < voltage
		3'b111:		state_out = 2'b01;				//charge over.
		default:	state_out = 2'b11;
	endcase
end

//assign {io_led_charge, io_led_discharge} = state_out;
assign {io_led_discharge, io_led_charge} = state_out;

//---------------------------------------------------------------------------------------------------------------------------------
//	DIO logic, 8 bit data bus.
//	DIO input Address offset: 0x3
//	DIO output Enable address offset: 0x4
//	DIO output Address offset: 0x5
//---------------------------------------------------------------------------------------------------------------------------------
reg [1:0] dio_output;
wire [1:0] dio_input;
reg dio_en;
wire dio_in_cs;
wire dio_en_cs;
wire dio_out_cs;

assign dio_in_cs  = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b11);
assign dio_en_cs  = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b100);
assign dio_out_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b101);

assign dio_input = {!dio_in[1], !dio_in[0]};

//---------------------------------------------------------------------------------------------------------------------------------
//	YX judgement instance
//	Uses 1MHz clock.
//---------------------------------------------------------------------------------------------------------------------------------
`define YX_WIDTH	4
wire dio_clk = clk_1mhz;
wire [`YX_WIDTH - 1:0] yx_input;
wire [`YX_WIDTH - 1:0] yx_out;
wire irq_yx;

assign yx_input = {sim_ncd, pwr_12v_fail, dio_input};

yx_judge YX1 (
	.clk_in(dio_clk), .rst_n(sys_reset_n), .yx_in(yx_input), .yx_out(yx_out), .irq_out(irq_yx)
);

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)            dio_en <= 1'b0;
	else if(!nwe && dio_en_cs)  dio_en <= sd[0];		
end

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)             dio_output <= 2'b0;
	else if(!nwe && dio_out_cs)  dio_output <= sd[1:0];		
end

assign dio_out = dio_en ? dio_output : 2'b0;
assign irq_cpld = {pwr_12v_fail, irq_yx};

//---------------------------------------------------------------------------------------------------------------------------------
//	LVDS power enable logic, 8 bit data bus.  
//	Address offset: 0x6
//	lvds_reg[0]: LVDS power enable. 1 - Enable(Default); 0 - Disable.
//---------------------------------------------------------------------------------------------------------------------------------
wire lvds_cs;
reg [2:0] lvds_reg;

assign lvds_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b110);

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)          lvds_reg <= 3'b111;
	else if(lvds_cs && !nwe)  lvds_reg <= sd[2:0];
end

assign lvds_pwen =  lvds_reg[0];
assign lcd_pwen  = !lvds_reg[1];
assign lcd_bl_en =  lvds_reg[2];

//---------------------------------------------------------------------------------------------------------------------------------
//	PCIE 4G logic:
//	address offset: 0x7
//	pcie_control[0]: 4G module power enable. 1 - en, 0 - Disable, default is 1.
//	pcie_control[1]: 4G module wakeup_in pin. 1 - sleep can be, 0 - sleep not can be. Default is 0.
//---------------------------------------------------------------------------------------------------------------------------------
reg [1:0] pcie_control;
wire pcie_4g_cs;

assign pcie_4g_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b111);
assign pcie_rst_n = !sys_reset_n;

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)             pcie_control <= 2'b1;
	else if(!nwe && pcie_4g_cs)  pcie_control <= sd[1:0];
end

assign pwr_4g_en      = pcie_control[0];
assign pcie_wakeup_in = pcie_control[1];

// ------------------------------------------------------------------------------------------------------------------------------
//	Address offset: 0x08
// 	ESAM logic: ESAM_CONTROL reg, used to power on ESAM, and reset ESAM.
// 	ESAM_PWR_ON: Enable ESAM's 3.3V Power. 1 - Enable, 0 - Disable. Default is 0.
// 	ESAM_RESET_N: Reset ESAM. 0 - Reset, 1 - Normal. Default is 0.
// 	ESAM_CLK_OUT_EN: Enable ESAM's clock output. 0 - Disable, 1 - Enable. Default is 0.
// 	esam_data_io: Setup ESAM_DATA pin's value. 0 - ESAM data = 0, 1 - ESAM data = ~TXD or 1'bz. 
// ------------------------------------------------------------------------------------------------------------------------------
`define ESAM_IO_WIDTH	4
reg esam_pwr_on;
reg esam_rst_on;
reg esam_clk_en;
reg esam_data_io;

wire[`ESAM_IO_WIDTH - 1:0] esam_ctrl; 
wire esam_ctrl_cs;
wire esam_clk_out = clk_esam_35712mhz;
wire esam_dir;

assign esam_ctrl_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b1000);

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)
		{esam_data_io, esam_clk_en, esam_rst_on, esam_pwr_on} <= 4'b0; 
	else if(!nwe && esam_ctrl_cs)
		{esam_data_io, esam_clk_en, esam_rst_on, esam_pwr_on} <= sd[`ESAM_IO_WIDTH - 1:0]; 
end

assign esam_ctrl     = {esam_data_io, esam_clk_en, esam_rst_on, esam_pwr_on}; 
assign esam_clk      = esam_clk_en  ? esam_clk_out : 1'b0;
assign esam_rst      = esam_rst_on;
assign esam_on       = !esam_pwr_on;
assign esam_dir      = !cpu_txd5;
assign esam_data     = esam_data_io ? (esam_dir ? cpu_txd5 : 1'bz)  : 1'b0; 
assign cpu_rxd5      = esam_data_io ? (esam_dir ? 1'b1 : esam_data) : 1'b1;

// ------------------------------------------------------------------------------------------------------------------------------
// 	Audio Amplifier logic: You can control the Audio Amplifier's OPEN or SHUTDOWN
//	OFFSET address: 0x09
// ------------------------------------------------------------------------------------------------------------------------------
reg audio_amp_en;
wire audio_amp_cs;

assign audio_amp_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b1001);	//offset address: 0x09

always @(posedge fpga_clk or negedge sys_reset_n)
begin
	if(!sys_reset_n)               audio_amp_en <= 1'b1;
	else if(!nwe && audio_amp_cs)  audio_amp_en <= sd[0];
end

assign cpld_amp_shdn = audio_amp_en;

// ------------------------------------------------------------------------------------------------------------------------------
// 	WIFI module logic: 
//	OFFSET address: 0x0a
// ------------------------------------------------------------------------------------------------------------------------------
//wire wifi_module_cs;
//reg [2:0] wifi_module_reg;
//
//assign wifi_module_cs = !ngcs[MISC_BANK] && (sa[`SA_WIDTH - 1:0] == `SA_WIDTH'b1010);	//Offset address: 0x0a
//
//always @(posedge fpga_clk or negedge sys_reset_n)
//begin
//	if(!sys_reset_n)                 wifi_module_reg <= 3'b1;
//	else if(!nwe && wifi_module_cs)  wifi_module_reg <= sd[2:0];
//end
//
//assign {wl_ext_smps_req, wl_ext_pwm_req, wl_host_wake} = wifi_module_reg;
//assign wl_reset_n = sys_reset_n;


assign cpld_led_out = wdg_cnt[`WDG_SUM - 1];

//---------------------------------------------------------------------------------------------------------------------------------
//	data bus logic, 8 bit data bus.  
//---------------------------------------------------------------------------------------------------------------------------------
assign sd = noe ? {`SD_WIDTH{1'bz}} : ( wdg_cs   	   ? {6'b0, wdg_pwm, wdg_en} 
				    : ( buzzer_cs          ? {7'b0, buzzer_en}
				    : ( io_led_out_cs      ? {3'b0, io_led_rev_reg, io_led_reg}
				    : ( dio_in_cs          ? {4'b0, yx_out}
				    : ( dio_en_cs          ? {7'b0, dio_en}
				    : ( dio_out_cs         ? {6'b0, dio_output}
				    : ( lvds_cs            ? {5'b0, lvds_reg}
				    : ( pcie_4g_cs         ? {6'b0, pcie_control}
				    : ( esam_ctrl_cs       ? {4'b0, esam_ctrl}
				    : ( audio_amp_cs       ? {7'b0, audio_amp_en}
				    //: ( wifi_module_cs     ? {5'b0, wifi_module_reg}
				    : {`SD_WIDTH{1'bz}} ))))))))));

endmodule
