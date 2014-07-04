// Include the MIPS constants
`include "internal_defines.vh"

////
//// mips_ALU: Performs all arithmetic and logical operations
////
//// out                    (output) - Final result
//// is_zero                (output) - Outputs if the result of a subtraction is zero (also used in branches)
//// is_greater_zero        (output) - Outputs if the operand is > 0. However, only used in branches
//// excpt                  (output) - Asserted if the ALU has an excpetion
//// op1                    (input)  - Operand modified by the operation
//// op2                    (input)  - Operand used (in arithmetic ops) to modify in1
//// sel                    (input)  - Selects which operation is to be performed

//// TODO   Right now, the ALU adds a 32 bit and 33 bit version to check
////        overflow...combine this to reduce to one adder!
module mips_ALU(// Outputs
                alu__out, alu__out_mul, alu__is_zero, alu__is_greater_zero, alu__excpt, alu__mul_stall,
                // Inputs
                alu__op1, alu__op2, alu__sel, clk, rst_b);
    
    output reg [31:0]   alu__out;
    output     [31:0]   alu__out_mul;           // This is the output of the coprocessor
    output reg          alu__is_zero;           // Only valid for SUB instructions
    output reg          alu__is_greater_zero;   // Only valid for branch ALU instructions
    output reg          alu__excpt;             // Assert if we overflow on signed instructions
    output              alu__mul_stall;
    
    input  [31:0]       alu__op1, alu__op2;
    input  [4:0]        alu__sel;
    input               clk, rst_b;             // Used by the multiply co-processor

    // Signed values are used for comparions and signed operations
    wire signed [31:0] alu__op1_signed, alu__op2_signed;
    assign alu__op1_signed = alu__op1;
    assign alu__op2_signed = alu__op2;

    // The result value will allow for the alu to never have overflow. We will
    // use the first two bits to see if overflow occurred, but only output a 
    // 32 bit answer (so we will ignore result's msb)
    reg [32:0] overflowCheck;
    
    // Variables to setup the coprocessor
    reg [2:0]   mul__opcode;    // Opcode used by the co-processor to determine the instruction
    reg         mul__active;    // Asserted if we are using the multiplier

    // Setup the coprocessor, but only use it if mul__active is enabled (implied mux!)
    multiply_coprocessor coprocessor(// Outputs
                                     .mul__rd_data_3a(alu__out_mul),
                                     .mul__stall_2a(alu__mul_stall),

                                     // Inputs
                                     .clk(clk),
                                     .rst_b(rst_b),
                                     .mul__opcode_2a(mul__opcode),
                                     .mul__active_2a(mul__active),
                                     .rt_data_2a(alu__op2),
                                     .rs_data_2a(alu__op1));

    always @(*) begin
        // We only care about the multiply co-processor if we are using a mult/divide instruction!
        mul__opcode = 3'hx;
        mul__active = 1'b0;

        // Only checking for equality when we subtract, also used on branches
        // to send if the operand is zero
        alu__is_zero = 1'bx;            
        alu__is_greater_zero = 1'bx;

        // Haven't done anything, no exception raised
        alu__excpt = 1'b0;

        // By default, don't care what it does
        alu__out = 32'hxxxxxxxx;
        
        case(alu__sel)

            // Addition / Subtraction
            `ALU_ADDU: begin
                alu__out = alu__op1 + alu__op2;
            end     
            `ALU_ADD: begin
                alu__out = alu__op1_signed + alu__op2_signed;
                
                overflowCheck = alu__op1_signed + alu__op2_signed;

                // If the msb and the msb+1 are different, we have overflow!
                alu__excpt = overflowCheck[31] ^ overflowCheck[32];
            end
            `ALU_SUBU: begin
                alu__out = alu__op1 - alu__op2;

                // Assert if result is zero
                alu__is_zero = (alu__out == 0);
            end
            `ALU_SUB: begin
                alu__out = alu__op1_signed - alu__op2_signed;
    
                // Assert if result is zero
                alu__is_zero = (alu__out == 0);

                overflowCheck = alu__op1_signed - alu__op2_signed;

                // If the msb and the msb+1 are different, we have overflow!
                alu__excpt = overflowCheck[31] ^ overflowCheck[32];
            end

            // Boolean operators
            `ALU_AND: begin
               alu__out = alu__op1 & alu__op2;
            end
            `ALU_OR: begin
               alu__out = alu__op1 | alu__op2;
            end
            `ALU_XOR: begin
                alu__out = alu__op1 ^ alu__op2;
            end
            `ALU_NOR: begin
                alu__out = ~ (alu__op1 | alu__op2);
            end

            // Shifts, only shift by a max of 31 (5 lsb of op1)
            `ALU_SLL: begin
               alu__out = alu__op2 << alu__op1[4:0];
            end
            `ALU_SRL: begin
               alu__out = alu__op2 >> alu__op1[4:0];
            end
            `ALU_SRA: begin
               alu__out = alu__op2_signed >>> alu__op1[4:0];
            end

            // Set on conditions
            `ALU_SLT: begin
                 alu__out = (alu__op1_signed < alu__op2_signed) ? 32'b1 : 32'b0 ;
            end
            `ALU_SLTU: begin
                 alu__out = (alu__op1 < alu__op2) ? 32'b1 : 32'b0;
            end

            // Instructions that requre the co-processor, just output the
            // result of the coprocessor, and set the mul opcode to the correct one
            `ALU_MULT: begin
                mul__opcode = `MUL_MULT;
                mul__active = 1;
            end
            `ALU_MULTU: begin
                mul__opcode = `MUL_MULTU;
                mul__active = 1;
            end
            `ALU_DIV: begin
                mul__opcode = `MUL_DIV;
                mul__active = 1;
            end
            `ALU_DIVU: begin
                mul__opcode = `MUL_DIVU;
                mul__active = 1;
            end

            // Moves from / to proccesor
            `ALU_MFHI: begin
                mul__opcode = `MUL_MFHI;
                mul__active = 1;
            end
            `ALU_MFLO: begin
                mul__opcode = `MUL_MFLO;
                mul__active = 1;
            end
            `ALU_MTHI: begin
                mul__opcode = `MUL_MTHI;
                mul__active = 1;
            end
            `ALU_MTLO: begin
                mul__opcode = `MUL_MTLO;
                mul__active = 1;
            end

            // Load Upper Immediate (for loads)
            `ALU_LUI: begin
                alu__out = alu__op2 << 16;
            end

            // Used for branches
            `ALU_BRC: begin
                alu__out = 32'bx;

                // Asserted if OPERAND greater then zero
                alu__is_greater_zero = (alu__op1_signed > 0);
                
                // Asserted if OPERAND is zero
                alu__is_zero = (alu__op1_signed == 0);
            end

            // Just pass through op2
            `ALU_PASS_OP2: begin
                alu__out = alu__op2;
            end
            
            // Default, output don't cares
            default: begin
                alu__out = 32'hxxxxxxxx;
            end
        endcase //case(alu__sel)

    end
endmodule
