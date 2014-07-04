/* mips_mem
 * A small synthesizable memory.
 *
 * Author: Joshua Wise
 */

module mips_mem(addr1,         data_out1,    excpt1,
		addr2,data_in2,data_out2,we2,excpt2,
		rst_b,clk);

	/* Original mips_mem_sync had:
	 *   Home segment at  32'h00000000 (16MB)
	 * 
	 * FPGA version has:
	 *   Home segment at  32'h00000000 (16KB)
	 */

	input         rst_b;
	input         clk;

	input      [29:0] addr1;         // Memory address
	output reg [31:0] data_out1;     // Memory read data
	output reg        excpt1;        // Exception occurred (active high)
	
	input      [29:0] addr2;         // Memory address
	input      [31:0] data_in2;      // Memory write data
	output reg [31:0] data_out2;     // Memory read data
	input       [0:3] we2;           // Write enable (backwards for compatibility with sim version, @#$*@#$*@#)
	output reg        excpt2;        // Exception occurred (active high)

	reg [31:0]        home_seg[0:4095]; /* mem.fixed.dat */
	
	integer i;
	
	initial begin
		for (i = 0; i < 4096; i = i + 1) begin
			home_seg[i] = 32'hdeadbeef;
		end
		$readmemh("mem.fixed.dat", home_seg);
	end
	
	/***************************************************************
	 * Port 1
	 ***************************************************************/
	
	reg decode_text_1, decode_data_1;
	always @(*) begin
		decode_data_1 = 1'b0;
		excpt1 = 1'b0;
		casex ({addr1,2'b00})
		32'b0000_0000_0000_0000_xxxx_xxxx_xxxx_xxxx: decode_data_1 = 1'b1;
		default: excpt1 = 1'b1;
		endcase
	end

	/* outside the always to avoid including the memory in an always @(*) */	
	wire [15:0] lowbits_1 = addr1[15:0];
	wire [31:0] data_seg_q1 = home_seg[lowbits_1];
	always @(*) begin
		case (1'b1) //synopsys parallel_case
		decode_data_1: data_out1 = data_seg_q1;
		default:       data_out1 = {32{1'hx}};
		endcase
	end

	/***************************************************************
	 * Port 2
	 ***************************************************************/
	
	reg decode_text_2, decode_data_2;
	always @(*) begin
		decode_data_2 = 1'b0;
		excpt2 = 1'b0;
		casex ({addr2,2'b00})
		32'b0000_0000_0000_0000_xxxx_xxxx_xxxx_xxxx: decode_data_2 = 1'b1;
		default:                                     excpt2 = 1'b1;
		endcase
	end

	/* outside the always to avoid including the memory in an always @(*) */	
	wire [15:0] lowbits_2 = addr2[15:0];
	wire [31:0] data_seg_q2 = home_seg[lowbits_2];
	always @(*) begin
		case (1'b1) //synopsys parallel_case
		decode_data_2: data_out2 = data_seg_q2;
		default:       data_out2 = {32{1'hx}};
		endcase
	end
	
	genvar ii;
	generate for (ii = 0; ii < 4; ii = ii + 1) begin: we2s
		always @(posedge clk) begin
			if (we2[ii] && decode_data_2)
				home_seg[lowbits_2][(ii+1)*8-1:ii*8] <= data_in2[(ii+1)*8-1:ii*8];
		end
	end
	endgenerate
endmodule
