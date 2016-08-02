module yx_judge
(
clk_in, rst_n, yx_in, yx_out, irq_out
);

`define YX_WIDTH	4

input clk_in;		//clock must be 1MHz
input rst_n;
input [`YX_WIDTH - 1:0] yx_in;
output [`YX_WIDTH - 1:0] yx_out;
output irq_out;

//---------------------------------------------------------------------------------------------------------------------------------
//	Input clock is 1MHz, and devided to 2MS YX judge clock
//---------------------------------------------------------------------------------------------------------------------------------
`define YX_CLK_SUM	2000
`define YX_CLK_WIDTH	11
reg [`YX_CLK_WIDTH - 1:0] yx_clk;
wire yx_clk_enable;

always @(posedge clk_in or negedge rst_n)
begin
	if(!rst_n)                      yx_clk <= `YX_CLK_WIDTH'b0;
	else if(yx_clk >= `YX_CLK_SUM)  yx_clk <= `YX_CLK_WIDTH'b0;
	else                            yx_clk <= yx_clk + `YX_CLK_WIDTH'b1;
end

assign yx_clk_enable = (yx_clk >= `YX_CLK_SUM);

//---------------------------------------------------------------------------------------------------------------------------------
//	judge current state, next state
//---------------------------------------------------------------------------------------------------------------------------------
reg [`YX_WIDTH - 1:0] dio_in_curr;
reg [`YX_WIDTH - 1:0] dio_in_next;
wire yx_change;

always @(posedge clk_in or negedge rst_n)
begin
	if(!rst_n)  dio_in_curr <= yx_in;
	else        dio_in_curr <= yx_in;
end

always @(posedge clk_in or negedge rst_n)
begin
	if(!rst_n)                              dio_in_next <= yx_in;
	else if(yx_clk_enable) begin
		if(dio_in_next != dio_in_curr)  dio_in_next <= dio_in_curr;
	end
end

assign yx_change = (dio_in_next != dio_in_curr);


assign yx_out  = dio_in_next;
assign irq_out = yx_change;


endmodule
