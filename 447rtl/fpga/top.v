module top(clk100, resetbtn, leds);

	input            clk100;
	input            resetbtn; 
	output reg [7:0] leds = 0;
	
	wire clk; /* nominally 50MHz */
	
	/* Divide 100MHz down to 50MHz */
	wire dcm_fb, dcm_locked;
	
	DCM_BASE dcm(.CLKIN(clk100),
	             .CLKFB(dcm_fb),
	             .CLK0(dcm_fb),
	             .RST(resetbtn),
	             .CLKDV(clk),
	             .LOCKED(dcm_locked));
	defparam dcm.CLK_FEEDBACK = "1X";
	defparam dcm.CLKDV_DIVIDE = 2.0;
	defparam dcm.CLKIN_DIVIDE_BY_2 = "FALSE";
	defparam dcm.CLKIN_PERIOD = 10.000;
	defparam dcm.DESKEW_ADJUST = "SYSTEM_SYNCHRONOUS";
	defparam dcm.DFS_FREQUENCY_MODE = "LOW";
	defparam dcm.DLL_FREQUENCY_MODE = "LOW";
	defparam dcm.PHASE_SHIFT = 0;
	defparam dcm.STARTUP_WAIT = "FALSE";
	
	/* Resets */
	reg rst_b = 0;
	reg resetbtn_1a = 0;
	reg resetbtn_2a = 0;
	always @(posedge clk) begin
		resetbtn_1a <= resetbtn;
		resetbtn_2a <= resetbtn_1a;
		if (resetbtn_2a || !dcm_locked)
			rst_b <= 0;
		else
			rst_b <= 1;
	end
	
	wire [31:0] inst, mem_data_in, mem_data_out;
	wire [29:0] pc, mem_addr;
	wire [3:0] mem_write_en;
	wire mem_excpt, halted;
	
	mips_core core(.clk(clk), .inst_addr(pc), .inst(inst),
	               .inst_excpt(inst_excpt), .mem_addr(mem_addr),
	               .mem_data_in(mem_data_in), .mem_data_out(mem_data_out),
	               .mem_write_en(mem_write_en), .mem_excpt(mem_excpt),
	               .halted(halted), .rst_b(rst_b));

	mips_mem mem(.addr1(pc), .data_out1(inst), .excpt1(inst_excpt),
	             .addr2(mem_addr), .data_in2(mem_data_in),
	             .data_out2(mem_data_out), .we2(mem_write_en),
	             .excpt2(mem_excpt), .rst_b(rst_b), .clk(clk));
	             
	/* And the LEDs?  Well, we'll cheat. */
	always @(posedge clk or negedge rst_b)
		if (!rst_b)
			leds <= 8'h00;
		else
			if (!halted && ({mem_addr,2'b00} == 32'h10003FF0) && mem_write_en[0])
				leds <= mem_data_in[7:0];
endmodule
