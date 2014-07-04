/* Multiplier unit for 18-447 MIPS
 * For pipelined synthesis
 * Author: Joshua Wise (jwise)
 */

module multiply_coprocessor(/*AUTOARG*/
   // Outputs
   mul__rd_data_3a, mul__stall_2a,
   // Inputs
   clk, rst_b, mul__opcode_2a, mul__active_2a, rt_data_2a, rs_data_2a
   );

	input              clk, rst_b;
	input        [2:0] mul__opcode_2a;
	input              mul__active_2a;
	input       [31:0] rt_data_2a, rs_data_2a;
	output reg  [31:0] mul__rd_data_3a;
	output wire        mul__stall_2a;
	
	wire busy; /* assigned below */
	
	/******************************************************************
	 * Multiplier control logic                                       *
	 ******************************************************************/
	reg signed [31:0] mulinput1_2a, mulinput2_2a;
	reg        [31:0] mulinput1_u_2a, mulinput2_u_2a;
	reg        mulbusy_2a;
	wire       mul_busy; /* assigned below */
	always @(*) begin
		mulinput1_2a = rs_data_2a;
		mulinput2_2a = rt_data_2a;
		
		mulinput1_u_2a = rs_data_2a;
		mulinput2_u_2a = rt_data_2a;
		
		mulbusy_2a = (mul__opcode_2a == `MUL_MULT || mul__opcode_2a == `MUL_MULTU)
		             && mul__active_2a
		             && !busy;
	end
	
	/* Pipeline the multiplier seven times for ludicrous frequency. */
	reg [63:0] mulresult_3a, mulresult_4a, mulresult_5a, mulresult_6a, mulresult_7a, mulresult_8a, mulresult_9a;
	reg [63:0] mulresult_u_3a, mulresult_u_4a, mulresult_u_5a, mulresult_u_6a, mulresult_u_7a, mulresult_u_8a, mulresult_u_9a;
	reg        signed_3a, signed_4a, signed_5a, signed_6a, signed_7a, signed_8a, signed_9a;
	reg        mulbusy_3a = 1'b0,
	           mulbusy_4a = 1'b0,
	           mulbusy_5a = 1'b0,
	           mulbusy_6a = 1'b0,
	           mulbusy_7a = 1'b0,
	           mulbusy_8a = 1'b0,
	           muldone_9a = 1'b0;
	always @(posedge clk) begin
		/* Multiplier "flops" are not actually resettable, since
		 * they are part of the DSP48E pipeline.
		 *
		 * XXX: This burns power even when the multiplier is idle.
		 * On a real system, not having a clock enable for the
		 * multiplier would be very bad style!
		 */
		mulresult_3a <= mulinput1_2a * mulinput2_2a;
		mulresult_u_3a <= mulinput1_u_2a * mulinput2_u_2a;
		case (mul__opcode_2a)
		`MUL_MULT:  signed_3a <= 1'b1;
		`MUL_MULTU: signed_3a <= 1'b0;
		default:    signed_3a <= 1'bx;
		endcase

		mulresult_4a <= mulresult_3a;
		mulresult_u_4a <= mulresult_u_3a;
		signed_4a <= signed_3a;

		mulresult_5a <= mulresult_4a;
		mulresult_u_5a <= mulresult_u_4a;
		signed_5a <= signed_4a;

		mulresult_6a <= mulresult_5a;
		mulresult_u_6a <= mulresult_u_5a;
		signed_6a <= signed_5a;

		mulresult_7a <= mulresult_6a;
		mulresult_u_7a <= mulresult_u_6a;
		signed_7a <= signed_6a;

		mulresult_8a <= mulresult_7a;
		mulresult_u_8a <= mulresult_u_7a;
		signed_8a <= signed_7a;

		mulresult_9a <= mulresult_8a;
		mulresult_u_9a <= mulresult_u_8a;
		signed_9a <= signed_8a;
	end
	
	always @(posedge clk or negedge rst_b) begin
		if (!rst_b) begin
			mulbusy_3a <= 0;
			mulbusy_4a <= 0;
			mulbusy_5a <= 0;
			mulbusy_6a <= 0;
			mulbusy_7a <= 0;
			mulbusy_8a <= 0;
			muldone_9a <= 0;
		end else begin
			mulbusy_3a <= mulbusy_2a;
			mulbusy_4a <= mulbusy_3a;
			mulbusy_5a <= mulbusy_4a;
			mulbusy_6a <= mulbusy_5a;
			mulbusy_7a <= mulbusy_6a;
			mulbusy_8a <= mulbusy_7a;
			muldone_9a <= mulbusy_8a;
		end
	end
	assign mul_busy = mulbusy_3a | mulbusy_4a | mulbusy_5a | mulbusy_6a | mulbusy_7a | mulbusy_8a | muldone_9a;

	/******************************************************************
	 * Divider control logic                                          *
	 ******************************************************************/
	wire [31:0] div_quotient, div_remainder;
	wire div_busy_0a;
	reg div_start, div_signd;
	reg [31:0] div_dividend, div_divisor;
	
	Div32 div(.quotient(div_quotient),
	          .remainder(div_remainder),
	          .dividend(div_dividend),
	          .divisor(div_divisor),
	          .start(div_start),
	          .busy(div_busy_0a),
	          .signd(div_signd),
	          .rst_b(rst_b),
	          .clk(clk));
	
	always @(*) begin
		div_signd = 1'bx;
		div_start = 1'b0;
		div_dividend = {32{1'bx}};
		div_divisor  = {32{1'bx}};
		
		case ({(mul__active_2a & ~busy),mul__opcode_2a})
		{1'b1,`MUL_DIV}: begin
			div_start = 1'b1;
			div_signd = 1'b1;
			div_dividend = rs_data_2a;
			div_divisor = rt_data_2a;
		end
		{1'b1,`MUL_DIVU}: begin
			div_start = 1'b1;
			div_signd = 1'b0;
			div_dividend = rs_data_2a;
			div_divisor = rt_data_2a;
		end
		endcase
	end
	
	reg div_busy_1a = 1'b0;
	always @(posedge clk or negedge rst_b) begin
		if (!rst_b) begin
			div_busy_1a <= 1'b0;
		end else begin
			div_busy_1a <= div_busy_0a;
		end
	end
	wire div_busy = div_busy_0a | div_busy_1a;

	/******************************************************************
	 * Output & stall control logic                                   *
	 ******************************************************************/
	reg [31:0] lo = 31'h0, hi = 31'h0;
	
	assign busy = div_busy | mul_busy;

	always @(posedge clk or negedge rst_b) begin
		if (!rst_b) begin
			hi <= 31'h0;
			lo <= 31'h0;
		end else begin
			/* This probably should be a case, but I CBA.  It's
			 * not on anyone's critical path, so... */
			if (muldone_9a)
				{hi, lo} <= signed_9a ? mulresult_9a : mulresult_u_9a;
			else if (div_busy_1a && ~div_busy_0a)
				{hi, lo} <= {div_remainder, div_quotient};
			else if (mul__active_2a && ~busy && mul__opcode_2a == `MUL_MTHI)
				hi <= rs_data_2a;
			else if (mul__active_2a && ~busy && mul__opcode_2a == `MUL_MTLO)
				lo <= rs_data_2a;
				
		end
	end

	always @(posedge clk or negedge rst_b) begin
		if (!rst_b) begin
			mul__rd_data_3a <= 32'h0;
		end else begin
			/* Mux the output. */
			case ({mul__active_2a, mul__opcode_2a})
			{1'b1,`MUL_MFHI}: mul__rd_data_3a <= hi;
			{1'b1,`MUL_MFLO}: mul__rd_data_3a <= lo;
			default:          mul__rd_data_3a <= {32{1'bx}};
			endcase
		end
	end

	/* Requests while we are busy must stall. */
	assign mul__stall_2a = mul__active_2a && busy;
endmodule
