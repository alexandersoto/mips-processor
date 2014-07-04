/*TODO - ADD/ADDI/ADDU/ADDUI - An overflow exception occurs if the carries out of bits 30 and 31 differ (2’s
            complement overflow). The destination register rd is not modified when an integer overflow exception occurs. 
            The only difference between this instruction and the ADDI instruction is that ADDIU never causes an overflow exception.
            Integer overflow exception */

/* TODO - JR/JALR  - Since instructions must be word-aligned, a Jump Register instruction
            must specify a target register (rs) whose two low-order bits are zero. If
            these low-order bits are not zero, an address exception will occur when the
            jump target instruction is subsequently fetched. */

/* TODO - SUB/SUBU - An integer overflow exception takes place if the carries out of bits 30 and
            31 differ (2s complement overflow). The destination register rd is not modified when an integer overflow exception occurs.
            The only difference between this instruction and the SUBU instruction is that SUBU never traps on overflow. Integer overflow exception */

/* TODO??? Not sure if we need these - LB - LBU - LH - LHU - LW - 
                                            TLB refill exception
                                            TLB invalid exception
                                            Bus error exception
                                            Address error exception
                                        SB - SH - SW
                                            TLB refill exception
                                            TLB invalid exception
                                            TLB modification exception
                                            Bus error exception
                                            Address error exception */
// Handles exceptions
module mips_exception_unit( // Outputs
                            excpt__clear_IF_ID, excpt__clear_ID_EX, excpt__clear_EX_MEM, excpt__clear_MEM_WB, excpt__clear_ALL, excpt__addressToJump, excpt__is_jump,
                            excpt__is_epc_write, excpt__epc_data, set_kernel_mode, set_user_mode, excpt__cause_data_in, tlb_miss,
                            // Inputs
                            excpt__IF, excpt__ID, excpt__EX, excpt__MEM, excpt__WB, PC, EPC);

    // Outputs
    output reg          excpt__clear_IF_ID, excpt__clear_ID_EX, excpt__clear_EX_MEM, excpt__clear_MEM_WB, excpt__clear_ALL;
    output reg          set_kernel_mode, set_user_mode;
    output reg [31:0]   excpt__addressToJump;
    output reg          excpt__is_jump;
    output reg          excpt__is_epc_write;
    output reg [31:0]   excpt__epc_data;
    output reg [4:0]    excpt__cause_data_in;
    output              tlb_miss;
    // Inputs
    input [3:0]  excpt__IF;
    input [7:0]  excpt__ID;
    input [9:0]  excpt__EX;
    input [11:0] excpt__MEM, excpt__WB;
    input [31:0] PC, EPC;

    // Decode each of these exception signals, so we can figure out when to
    // clear registers, and do exceptional things (PUN intended)
    
    // Fetch wires and assigns
    wire    AdEL__IF, IBE__IF, TLBL__IF, TLBS__IF;
    assign  IBE__IF         = excpt__IF[0];
    assign  AdEL__IF        = excpt__IF[1];
    assign  TLBL__IF        = excpt__IF[2];
    assign  TLBS__IF        = excpt__IF[3];
    

    // Decode wires and assigns 
    wire    AdEL__ID, IBE__ID, ERET__ID, RI__ID, Sys__ID,
            testDone__ID, TLBL__ID, TLBS__ID;
    assign  IBE__ID         = excpt__ID[0];
    assign  AdEL__ID        = excpt__ID[1];
    assign  TLBL__ID        = excpt__ID[2];
    assign  TLBS__ID        = excpt__ID[3];
    assign  ERET__ID        = excpt__ID[4];
    assign  RI__ID          = excpt__ID[5];
    assign  Sys__ID         = excpt__ID[6];
    assign  testDone__ID    = excpt__ID[7];

    // Execute wires and assigns
    wire    AdEL__EX, IBE__EX, ERET__EX, RI__EX, Sys__EX, testDone__EX,
            AdES__EX, OV__EX, TLBL__EX, TLBS__EX;
    assign  IBE__EX         = excpt__EX[0];
    assign  AdEL__EX        = excpt__EX[1];
    assign  TLBL__EX        = excpt__EX[2];
    assign  TLBS__EX        = excpt__EX[3];
    assign  ERET__EX        = excpt__EX[4];
    assign  RI__EX          = excpt__EX[5];
    assign  Sys__EX         = excpt__EX[6];
    assign  testDone__EX    = excpt__EX[7];
    assign  AdES__EX        = excpt__EX[8];
    assign  OV__EX          = excpt__EX[9];

    // Memory wires and assigns
    wire    AdEL__MEM, IBE__MEM, ERET__MEM, RI__MEM, Sys__MEM, testDone__MEM,
            AdES__MEM, OV__MEM, DBE__MEM, TLBL__MEM, TLBS__MEM;
    assign  IBE__MEM        = excpt__MEM[0];
    assign  AdEL__MEM       = excpt__MEM[1];
    assign  TLBL__MEM       = excpt__MEM[2];
    assign  TLBS__MEM       = excpt__MEM[3];
    assign  ERET__MEM       = excpt__MEM[4];
    assign  RI__MEM         = excpt__MEM[5];
    assign  Sys__MEM        = excpt__MEM[6];
    assign  testDone__MEM   = excpt__MEM[7];
    assign  AdES__MEM       = excpt__MEM[8];
    assign  OV__MEM         = excpt__MEM[9];
    assign  DBE__MEM        = excpt__MEM[10];
    assign  TLBM__MEM       = excpt__MEM[11];
    
    // Write Back wires and assigns
    wire    AdEL__WB, IBE__WB, ERET__WB, RI__WB, Sys__WB, testDone__WB,
            AdES__WB, OV__WB, DBE__WB, TLBL__WB, TLBS__WB;  
    assign  IBE__WB         = excpt__WB[0];
    assign  AdEL__WB        = excpt__WB[1];
    assign  TLBL__WB        = excpt__WB[2];
    assign  TLBS__WB        = excpt__WB[3];
    assign  ERET__WB        = excpt__WB[4];
    assign  RI__WB          = excpt__WB[5];
    assign  Sys__WB         = excpt__WB[6];
    assign  testDone__WB    = excpt__WB[7];
    assign  AdES__WB        = excpt__WB[8];
    assign  OV__WB          = excpt__WB[9];
    assign  DBE__WB         = excpt__WB[10];
    assign  TLBM__WB        = excpt__WB[11];

    wire exception_in_WB;
    assign exception_in_WB = AdEL__WB || IBE__WB || RI__WB || Sys__WB || AdES__WB ||
                             OV__WB || DBE__WB || TLBL__WB || TLBS__WB || TLBM__WB;
    assign tlb_miss = TLBL__WB | TLBS__WB ;

    /*
    On an exception, your pipeline should perform the following:
        -Nullify the faulting instruction and any younger instructions already in progress.
        -Update EPC, Cause (and BadVAddr as needed).
        -Clear the user-mode bit in Status to 0.
        -Jump to the exception handler at address 0x8000_0180.  (In a future lab, bit 22 in the  Status register will dictate this address.)
    */

    // Clear the instruction causing the exception, and every flop behind it!
    // Then we have to jump to the exception handler
    always@(*) begin

        // Assume no exception
        excpt__clear_IF_ID  = 1'b0;
        excpt__clear_ID_EX  = 1'b0;
        excpt__clear_EX_MEM = 1'b0;
        excpt__clear_MEM_WB = 1'b0;
        excpt__clear_ALL    = 1'b0;
        excpt__is_jump      = 1'b0;
        excpt__is_epc_write = 1'b0;
        excpt__epc_data     = 32'hxxxxxxxx;
        set_user_mode       = 1'b0;
        set_kernel_mode     = 1'b0;
        excpt__cause_data_in = 5'd0;
        

        // Only valid for syscall...on eret we want to jump to the value in EPC
        excpt__addressToJump = 32'hxxxxxxxx;
            
        // On any EXCEPTION we will store PC into EPC (not ERET as that is not an exception!)
        // Then we will store the address we need to jump to, and say we are jumping
        if(exception_in_WB) begin
            excpt__epc_data = PC;
            excpt__is_epc_write = 1'b1;
            excpt__addressToJump = `EXCEPTION_ADDRESS;
            excpt__is_jump = 1'b1;
            set_kernel_mode = 1'b1;

            // Determine the cause code
            if(DBE__WB) begin
                excpt__cause_data_in = `EX_DBE;
            end
            if(AdES__WB) begin
                excpt__cause_data_in = `EX_ADES;
            end
            if(OV__WB) begin
                excpt__cause_data_in = `EX_OV;
            end
            if(RI__WB) begin
                excpt__cause_data_in = `EX_RI;
            end
            if(Sys__WB) begin
                excpt__cause_data_in = `EX_SYS;
            end
            if(AdEL__WB) begin
                excpt__cause_data_in = `EX_ADEL;
            end
            if(IBE__WB) begin
                excpt__cause_data_in = `EX_IBE;
            end
            if(TLBM__WB) begin
                excpt__cause_data_in = `EX_MOD;
            end         

            // Handle TLB exceptions, note that they jump to a
            // different exception handler
            if(TLBL__WB) begin
                excpt__cause_data_in = `EX_TLBL;
                excpt__addressToJump = `TLB_HANDLER_ADDRESS;    
            end
            if(TLBS__WB) begin
                excpt__cause_data_in = `EX_TLBS;
                excpt__addressToJump = `TLB_HANDLER_ADDRESS;    
            end
        end
            
        // If any bit is set, the exception lines will be non zero, so
        // clear flops behind them in the pipeline
        if (excpt__WB != 0) begin
            excpt__clear_ALL        = 1'b1;
        end
        else if (excpt__MEM != 0) begin
            excpt__clear_IF_ID      = 1'b1;
            excpt__clear_ID_EX      = 1'b1;
            excpt__clear_EX_MEM     = 1'b1;
            excpt__clear_MEM_WB     = 1'b1;
        end
        else if (excpt__EX != 0) begin
            excpt__clear_IF_ID  = 1'b1;
            excpt__clear_ID_EX  = 1'b1;
            excpt__clear_EX_MEM     = 1'b1;
        end
        else if (excpt__ID != 0) begin
            excpt__clear_IF_ID  = 1'b1;
            excpt__clear_ID_EX  = 1'b1;
        end 

        // Handle an ERET exception
        if(ERET__WB == 1) begin
            excpt__is_jump = 1'b1;
            excpt__addressToJump = EPC;
            set_user_mode = 1;
        end
    end
endmodule
