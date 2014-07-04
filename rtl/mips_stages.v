//`default_nettype none
`include "internal_defines.vh"

////////////////////////// Instruction Decode and Write Back Stage /////////////////////////////////////////////
module mips_ID_WB_stage(clk, rst_b,
                        // ID outputs
                        alu__op1_ID, alu__op2_ID, rt_data_ID, rs_data_ID, write_reg_ID_out, inst_ID_out,
                        coprocessor_data_ID, pc__ID_out, epc__ID_out, is_kernel_mode,
                        entryLo__ID, entryHi__ID, index__ID,
                        // ID inputs
                        inst_ID, ctrl_ID, halted_ID, pc__ID_in, set_kernel_mode, set_user_mode,
                        is_forward__rs_data__FWD, is_forward__rt_data__FWD, is_forward__coproc_data__FWD,
                        forwarded_rs_data__FWD, forwarded_rt_data__FWD, forwarded_coproc_data__FWD,
                        is_forward__entryLo__FWD, is_forward__entryHi__FWD, is_forward__index__FWD,
                        forwarded_entryLo__FWD, forwarded_entryHi__FWD, forwarded_index__FWD, load_badVaddr, badVaddr,
                        // WB inputs
                        write_data_WB, write_reg_WB_in, ctrl_WB, epc_data, is_epc_write, cause_data_in);

    // Outputs
    output reg [31:0]   alu__op1_ID, alu__op2_ID; 
    output reg [4:0]    write_reg_ID_out;
    output reg [31:0]   rt_data_ID, rs_data_ID;
    output [31:0]       inst_ID_out, epc__ID_out, pc__ID_out;
    output reg [31:0]   coprocessor_data_ID;
    output              is_kernel_mode;
    output reg [31:0]   entryLo__ID, entryHi__ID;
    output reg [31:0]   index__ID;

    // Inputs
    input           clk, rst_b;
    input [31:0]    inst_ID, pc__ID_in;
    input [9:0]     ctrl_ID;
    input           halted_ID;
    input           set_kernel_mode, set_user_mode;

    // Used for forwarded data
    input           is_forward__rs_data__FWD, is_forward__rt_data__FWD, is_forward__coproc_data__FWD;
    input [31:0]    forwarded_rs_data__FWD, forwarded_rt_data__FWD, forwarded_coproc_data__FWD;
    input           is_forward__entryLo__FWD, is_forward__entryHi__FWD, is_forward__index__FWD;
    input [31:0]    forwarded_entryLo__FWD, forwarded_entryHi__FWD, forwarded_index__FWD;


    // WB
    input [31:0]    write_data_WB;
    input [4:0]     write_reg_WB_in;
    input [4:0]     ctrl_WB;
    input [31:0]    epc_data;
    input           is_epc_write;
    input [4:0]     cause_data_in;
    input           load_badVaddr;
    input [31:0]    badVaddr;


    // ID internal wires!
    wire [31:0]     dcd_imm_ext;
    wire [31:0]     dcd_se_imm, dcd_e_imm;
    wire [4:0]      dcd_rs, dcd_rt, dcd_rd, dcd_shamt;
    wire [15:0]     dcd_imm;
    wire [31:0]     inst, rs_data, rt_data, epc;
    wire [31:0]     entryLo, entryHi;

    // ID Control Signals
    wire [1:0]      alu__op1_sel, alu__op2_sel, write_reg_sel;
    wire [2:0]      coprocessor_read_sel;
    wire            se_imm;

    // WB Control Signal
    wire            rd_we;
    wire [2:0]      coprocessor_write_sel__WB;

    // The value out of the coprocessor. Sometimes we may use this, other
    // times we may use the forwarded value
    wire [31:0]     coprocessor_data_out;

    // Connected to the counter
    wire [3:0]      count;
    wire [31:0]     random_count;

    // Value we are writing to the coprocessor;
    reg [31:0]      coprocessor_data_in;
    reg [2:0]       coprocessor_write_sel;

    // ID Instruction decoding
    // dcd_imm          - The immedaite value as read from instruction
    // dcd_e_imm        - ???
    // dcd_se_imm       - The immediate value, sign extended
    // dcd_imm_ext      - The immediate value, extended
    assign  inst =          inst_ID;
    assign  inst_ID_out =   inst_ID;
    assign  dcd_rs =        inst[25:21];        // rs field
    assign  dcd_rt =        inst[20:16];        // rt field
    assign  dcd_rd =        inst[15:11];        // rd field
    assign  dcd_shamt =     inst[10:6];         // Shift amount
    assign  dcd_imm =       inst[15:0];         // immediate field
    assign  dcd_e_imm =     { 16'h0, dcd_imm }; // zero-extended immediate
    // Sign-extended immediate
    assign  dcd_se_imm = { {16{dcd_imm[15]}}, dcd_imm };
    assign  dcd_imm_ext = (se_imm) ? dcd_se_imm : dcd_e_imm ;


    assign pc__ID_out = pc__ID_in;
    assign epc__ID_out = epc;
    
    // Pseudo random number generator
    fourBitCounter  counter( //Outputs
                            .count(count),
                            // Inputs
                            .clk(clk),
                            .rst_b(rst_b));

    assign random_count = { 28'h0, count };


    always @(*) begin

        // Determine the index for the TLB instructions. If it's random, get
        // the result from the counter. Otherwise, get the result from the
        // coprocessor
        if(inst == `INST_TLBWR) begin
            index__ID = random_count;
        end
        else begin
            index__ID = coprocessor_data_out;
        end

        // Figure out what to write to the coprocessor
        if(is_epc_write) begin
            coprocessor_data_in = epc_data;
            coprocessor_write_sel = `EPC_WRITE;
        end

        // Take data from the write back stage
        else begin
            coprocessor_data_in = write_data_WB;
            coprocessor_write_sel = coprocessor_write_sel__WB;
        end 

        // Questionable, I hacked this so SW would work (also for branches), haven't througly tested it
        // The idea is that if we are forwarding either rs or rt then it must
        // be changed for everything (not just ALU functions)
        if ( is_forward__rs_data__FWD ) begin
            rs_data_ID = forwarded_rs_data__FWD;
        end
        else begin
            rs_data_ID = rs_data;
        end
        
        if ( is_forward__rt_data__FWD ) begin
            rt_data_ID = forwarded_rt_data__FWD;
        end
        else begin
            rt_data_ID = rt_data;
        end

        case(write_reg_sel)
            `WR_REG_RD: write_reg_ID_out = dcd_rd;
            `WR_REG_RT: write_reg_ID_out = dcd_rt;
            `WR_REG_31: write_reg_ID_out = 5'd31;
            default:    write_reg_ID_out = dcd_rd;
        endcase

        // Check if we had to forward, if we did then just set the select's to
        // the forwarded data, if we are using a register
        if( is_forward__rs_data__FWD && (alu__op1_sel == `ALU_OP1_REG) ) begin
            alu__op1_ID = forwarded_rs_data__FWD;
        end

        // First ALU input - can be register rs or shamt
        else begin
            case(alu__op1_sel)
                `ALU_OP1_REG:     alu__op1_ID = rs_data;
                `ALU_OP1_SHIFT:   alu__op1_ID = {27'b0,dcd_shamt};
                default:          alu__op1_ID = rs_data; // WILL CHANGE
            endcase
        end

        if( is_forward__rt_data__FWD && (alu__op2_sel == `ALU_OP2_REG) ) begin
            alu__op2_ID = forwarded_rt_data__FWD;
        end

        // Second ALU input - can be register rt or imm
        else begin
            case(alu__op2_sel)
                `ALU_OP2_REG:  alu__op2_ID = rt_data;
                `ALU_OP2_IMM:  alu__op2_ID = dcd_imm_ext;
                default:       alu__op2_ID = dcd_imm_ext; // WILL CHANGE
            endcase
        end

        // Coprocessor forwarding 
        if ( is_forward__coproc_data__FWD ) begin
            coprocessor_data_ID = forwarded_coproc_data__FWD;
        end
        else begin
            coprocessor_data_ID = coprocessor_data_out;
        end

        // Forward EntryLo
        if( is_forward__entryLo__FWD ) begin
            entryLo__ID = forwarded_entryLo__FWD;
        end
        else begin
            entryLo__ID = entryLo;
        end

        // Forward EntryHi
        if( is_forward__entryHi__FWD ) begin
            entryHi__ID = forwarded_entryHi__FWD;
        end
        else begin
            entryHi__ID = entryHi;
        end

        // Forward Index
        if( is_forward__index__FWD ) begin
            index__ID = forwarded_index__FWD;
        end
        else begin
            index__ID = coprocessor_data_out;
        end
    end

    // Register File
    regfile_3port regfile(  // Outputs
                            .rs_data (rs_data),
                            .rt_data (rt_data),
                            // Inputs
                            .rs_num  (dcd_rs),
                            .rt_num  (dcd_rt),
                            .rd_num  (write_reg_WB_in), // FROM WB
                            .rd_data (write_data_WB),   // FROM WB
                            .rd_we   (rd_we),
                            .clk     (clk),
                            .rst_b   (rst_b),
                            .halted  (halted_ID));

    // Coprocessor Registers
    coprocessor_registers   coregs( // Outputs
                                    .coprocessor_data_out(coprocessor_data_out),
                                    .epc(epc),
                                    .isKernal_mode(is_kernel_mode),
                                    .entryHi(entryHi),
                                    .entryLo(entryLo),
                                    // Inputs
                                    .badVaddr(badVaddr),
                                    .load_badVaddr(load_badVaddr),
                                    .cause_data_in(cause_data_in),
                                    .coprocessor_data_in(coprocessor_data_in),
                                    .pc(/* THIS WILL BE THE PC FROM THE WB STAGE */),
                                    .exception_sel(/* EXCPT SEL FROM WB STAGE */),
                                    .set_user_mode(set_user_mode),
                                    .set_kernel_mode(set_kernel_mode),
                                    .read_sel(coprocessor_read_sel),
                                    .write_sel(coprocessor_write_sel),
                                    .rst_b(rst_b),
                                    .clk(clk));
    

    decode_ctrl_ID decode_ID(   // Outputs
                                .write_reg_sel(write_reg_sel),
                                .alu__op1_sel(alu__op1_sel),
                                .alu__op2_sel(alu__op2_sel),
                                .coprocessor_read_sel(coprocessor_read_sel),
                                .se_imm(se_imm),
                                // Inputs
                                .ctrl_ID(ctrl_ID));

    // WB STUFF
    decode_ctrl_WB decode_WB(   // Outputs
                                .rd_we(rd_we),
                                .coprocessor_write_sel(coprocessor_write_sel__WB),
                                .tlb_we(), // Not needed here
                                // Inputs
                                .ctrl_WB(ctrl_WB));
endmodule


// Registers used during exceptions
module coprocessor_registers(   // Outputs
                                coprocessor_data_out, entryLo, entryHi, isKernal_mode, epc,
                                // Inputs
                                badVaddr, load_badVaddr, coprocessor_data_in, set_user_mode, set_kernel_mode, cause_data_in,
                                pc, exception_sel, read_sel, write_sel, rst_b, clk);

    // Outputs
    output reg  [31:0]  coprocessor_data_out, entryLo, entryHi;
    output      [31:0]  epc;
    output              isKernal_mode;

    // Inputs
    input [31:0]    badVaddr;
    input           load_badVaddr;
    input [31:0]    coprocessor_data_in, pc;
    input           exception_sel, set_user_mode, set_kernel_mode;
    input [2:0]     read_sel;
    input [2:0]     write_sel;
    input           rst_b, clk;
    input [4:0]     cause_data_in;

    reg [31:0]      epc_data, status_data, cause_data, badvaddr_data, index;

    // Bit 4 tells us which mode we are in
    assign isKernal_mode = ~status_data[4]; // TODO
    assign epc = epc_data;

    // We are writing the data into the register give by the select line
    always@(posedge clk or negedge rst_b) begin
        if(~rst_b) begin
            epc_data        <= 32'b0;
            status_data     <= 32'b0;
            cause_data      <= 32'b0;
            badvaddr_data   <= 32'b0;
            entryLo         <= 32'b0;           
            entryHi         <= 32'b0;
            index           <= 32'b0;
        end
        else begin
            if(set_user_mode) status_data[4] <= 1'b1;
            if(set_kernel_mode) status_data[4] <= 1'b0;
            case(write_sel)
                `EPC_WRITE:         epc_data        <= coprocessor_data_in;
                `STATUS_WRITE:      status_data     <= coprocessor_data_in;
                //`CAUSE_WRITE:     cause_data      <= coprocessor_data_in;
                //`BADVADDR_WRITE:  badvaddr_data   <= coprocessor_data_in;
                `ENTRYLO_WRITE:     entryLo         <= coprocessor_data_in;
                `ENTRYHI_WRITE:     entryHi         <= coprocessor_data_in;
                `INDEX_WRITE:       index           <= coprocessor_data_in;             
                default:            /* Do Nothing */ ;
            endcase

            // Write to badVAddr
            if(load_badVaddr) begin
                badvaddr_data   <= badVaddr;
            end
            
            // Write to the cause register
            if(cause_data_in != 0) begin
                cause_data[6:2] <= cause_data_in;
            end         
        end
    end

    // Determine which registers to read
    always @(*) begin
        coprocessor_data_out = 32'hxxxxxxxx;

        case(read_sel)
            `EPC_READ:      coprocessor_data_out = epc_data;
            `STATUS_READ:   coprocessor_data_out = status_data;
            `CAUSE_READ:    coprocessor_data_out = cause_data;
            `BADVADDR_READ: coprocessor_data_out = badvaddr_data;
            `ENTRYLO_READ:  coprocessor_data_out = entryLo;
            `ENTRYHI_READ:  coprocessor_data_out = entryHi;
            `INDEX_READ:    coprocessor_data_out = index;
            default:        coprocessor_data_out = 32'hdeadbeef;
        endcase
    end
endmodule

////////////////////////// Execute Stage /////////////////////////////////////////////
module mips_EX_stage(   // Outputs
                        alu__out_EX, alu__out_mul, write_reg_EX_out, alu__is_zero_EX, alu__is_greater_zero_EX,
                        excpt__EX_out, mem_addr_EX, mem_write_en_EX, mem_data_EX, alu__mul_stall__EX,
                        next_addr, is_non_sequential, pc__EX_out, epc__EX_out, write_data_EX, is_store__EX,
                        // Inputs
                        clk, rst_b, is_kernel_mode, alu__op1_EX, alu__op2_EX, rt_data_EX, rs_data_EX, coprocessor_data_EX, excpt__EX_in,
                        write_reg_EX_in, ctrl_EX, pc__EX_in, epc__EX_in, inst);

    // Ouputs
    output [31:0]       alu__out_EX;
    output [31:0]       alu__out_mul;
    output [4:0]        write_reg_EX_out;
    output              alu__is_zero_EX, alu__is_greater_zero_EX;
    output reg [9:0]    excpt__EX_out;
    output [29:0]       mem_addr_EX;
    output [3:0]        mem_write_en_EX;
    output [31:0]       mem_data_EX;
    output              alu__mul_stall__EX;
    output [31:0]       next_addr;
    output              is_non_sequential;  // True if we are taking a jump or branch
    output [31:0]       pc__EX_out, epc__EX_out;
    output reg [31:0]   write_data_EX;
    output              is_store__EX;

    // Inputs
    input           clk, rst_b;
    input           is_kernel_mode;
    input  [31:0]   alu__op1_EX, alu__op2_EX; 
    input  [31:0]   rt_data_EX, rs_data_EX;
    input  [14:0]   ctrl_EX;
    input  [4:0]    write_reg_EX_in;
    input  [7:0]    excpt__EX_in;
    input  [31:0]   pc__EX_in, epc__EX_in, inst;
    input  [31:0]   coprocessor_data_EX;

    // Internal wires
    //// mem_access_size    (output) - 4 bit memory enable for write (This isn't 4 bits...why the inconsitency??)
    //// is_store           (output) - Asserted if we are a memory store

    wire [4:0]  alu__sel;
    wire [31:0] alu__out;
    wire [1:0]  mem_access_size;
    wire        is_store;
    wire        alu__excpt, mem_write_excpt;
    wire [1:0]  jump_sel;
    wire [2:0]  branch_type;
    wire [1:0]  write_data_sel_EX;
    wire        jump_excpt;
    reg         ades_excpt, adel_excpt;

    assign alu__out_EX = alu__out;
    assign mem_addr_EX = alu__out[31:2];
    assign write_reg_EX_out = write_reg_EX_in;
    assign pc__EX_out = pc__EX_in;
    assign epc__EX_out = epc__EX_in;
    assign is_store__EX = is_store;

    always @(*) begin 
        adel_excpt = 1'b0;
        ades_excpt = 1'b0;
        if(mem_access_size != `SIZE_NONE) begin
            if( ~is_store && mem_write_excpt) begin
                adel_excpt = 1'b1;
            end
            if( is_store && mem_write_excpt) begin
                ades_excpt = 1'b1;
            end
            if( ~is_store && ~is_kernel_mode && (alu__out >= `KSEG0_START)) begin
                adel_excpt = 1'b1;
            end
            else if( is_store && ~is_kernel_mode && (alu__out >= `KSEG0_START)) begin
                ades_excpt = 1'b1;          
            end
        end
    end

    // correctly set exceptions
    always @(*) begin
        excpt__EX_out = {alu__excpt, ades_excpt, excpt__EX_in};

        // We have an adel exception if we had one before, or we have one now
        excpt__EX_out[1] = excpt__EX_in[1] | adel_excpt;
    end

    // PC, set equal
    
    always @(*) begin
        case(write_data_sel_EX)
            `SEL_FROM_ALU: write_data_EX = alu__out_EX;
            `SEL_FROM_PC4: write_data_EX = pc__EX_in+4;
            `SEL_FROM_COPROC: write_data_EX = coprocessor_data_EX;
            default: write_data_EX = alu__out_EX;
        endcase

    end



    // Decoder of the execute stage
    decode_ctrl_EX decode_EX(   // Outputs
                                .alu__sel(alu__sel),
                                .mem_access_size(mem_access_size),
                                .is_store(is_store),
                                .jump_sel(jump_sel),
                                .branch_type(branch_type),
                                .write_data_sel_EX(write_data_sel_EX),
                                // Inputs
                                .ctrl_EX(ctrl_EX)); // WILL CHANGE

    // Arithmetic Logic Unit
    mips_ALU alu(               // Outputs
                                .alu__out(alu__out),
                                .alu__out_mul(alu__out_mul),
                                .alu__is_zero(alu__is_zero_EX),
                                .alu__is_greater_zero(alu__is_greater_zero_EX),
                                .alu__excpt(alu__excpt),
                                .alu__mul_stall(alu__mul_stall__EX),
                                // Inputs
                                .alu__op1(alu__op1_EX), 
                                .alu__op2(alu__op2_EX), 
                                .alu__sel(alu__sel),
                                .clk(clk),
                                .rst_b(rst_b));

    mem_write_decoder mem_decoder( // Outputs
                                .mem_write_en(mem_write_en_EX),
                                .data_out(mem_data_EX),
                                .addr_excpt(mem_write_excpt),
                                // Inputs
                                .data_in(rt_data_EX),
                                .mem_write_size(mem_access_size),
                                .byte_offset(alu__out[1:0]),
                                .is_store(is_store));

    calculate_next_address address_calc(    // Outputs
                                            .next_addr(next_addr),
                                            .is_non_sequential(is_non_sequential),
                                            .addr_excpt(jump_excpt),
                                            // Inputs
                                            .coprocessor_data(coprocessor_data_EX),
                                            .pc(pc__EX_in),
                                            .alu__is_zero(alu__is_zero_EX),
                                            .alu__is_greater_zero(alu__is_greater_zero_EX),
                                            .jump_sel(jump_sel),
                                            .branch_type(branch_type),
                                            .rs_data(rs_data_EX),
                                            .inst(inst));
endmodule


// This module outputs the correct mem_write_en based off the starting byte
// and the size of the instruction (b,h,w)
module mem_write_decoder(is_store, mem_write_size, byte_offset, mem_write_en, addr_excpt, data_in, data_out);
  input             is_store;
  input [1:0]       mem_write_size, byte_offset;
  input [31:0]      data_in;
  output reg [3:0]  mem_write_en;
  output reg        addr_excpt;
  output reg [31:0] data_out;

  wire [4:0]        shift_amount;

  // Use shift amount to store the proper bits. This is just
  // multiplying the byte offset by 4
  assign shift_amount = {3'b0,byte_offset};

  always @(*) begin
    addr_excpt = 0;
    mem_write_en = 4'h0;

   // Synth says that we don't use 2 bits of shift_amount...maybe we don't need them?
   data_out = data_in << (shift_amount << 3);

      case(mem_write_size)
         `SIZE_BYTE: begin
         case(byte_offset)
           2'h0: mem_write_en = 4'b1000;  // BIG ENDIAN
           2'h1: mem_write_en = 4'b0100;
           2'h2: mem_write_en = 4'b0010;
           2'h3: mem_write_en = 4'b0001;
          endcase
        end
        `SIZE_HALFWORD: begin
          case(byte_offset)
            2'h0: mem_write_en = 4'b1100;
            2'h1: addr_excpt   = 1;
            2'h2: mem_write_en = 4'b0011;
            2'h3: addr_excpt   = 1;
          endcase
        end
        `SIZE_WORD: begin
          if(byte_offset==0)
            mem_write_en = 4'b1111;
          else
            addr_excpt = 1;
        end
        default: mem_write_en = 4'h0;
      endcase
      
   // If we are not storing, just force write enable lines to be zero
   if(~is_store) begin
      mem_write_en = 4'h0;
   end
  end
endmodule
        

// Find the proper next address given a target and the PC
module calculate_next_address(  // Outputs
                                next_addr, is_non_sequential, addr_excpt,
                                // Inputs
                                pc, coprocessor_data, alu__is_zero, alu__is_greater_zero, jump_sel, branch_type, rs_data, inst);

    // Outputs
    output reg  [31:0]  next_addr;          // The final jump address
    output reg          is_non_sequential;  // True if we branch or jump
    output reg          addr_excpt;         // Asserted if an invalid address to jump to
    
    // Inputs
    input   [31:0]  pc, coprocessor_data;
    input           alu__is_zero;
    input           alu__is_greater_zero;
    input   [1:0]   jump_sel;
    input   [2:0]   branch_type;
    input   [31:0]  rs_data;        // Used for Jump to Register command
    input   [31:0]  inst;

    wire    [25:0]  target;
    wire    [31:0]  branch_offset;

    wire [31:0]     pc_plus4;

    // The target field from the instruction
    assign target = inst[25:0];

    // The branch offset from the instruction
    assign branch_offset =  {{14{inst[15]}}, inst[15:0], 2'b00};

    assign pc_plus4 = pc + 4;

    always @ (*) begin

        // Assume it will be sequential
        next_addr = pc_plus4 + branch_offset;
        is_non_sequential = 0;
        
        // We are jumping to EPC
        if(jump_sel == `JUMP_EPC) begin
            next_addr = coprocessor_data;
            is_non_sequential = 1;
        end

        // Jump based on target (immediate from instruction)
        if(jump_sel == `JUMP_IMM) begin
            next_addr = {pc_plus4[31:28], (target << 2)};
            is_non_sequential = 1;
        end

        // Jump based on register
        else if(jump_sel == `JUMP_REG) begin
            next_addr = rs_data;
            is_non_sequential = 1;
        end
        
        // We may be branching
        else begin
            case(branch_type)
                `BRC_EQ: begin
                    if( alu__is_zero ) begin
                        is_non_sequential = 1;
                    end
                end
                `BRC_NEQ: begin
                    if( ~alu__is_zero ) begin
                        is_non_sequential = 1;
                    end
                end

                // In these cases, next_addr = offset << 2
                `BRC_GTZ: begin
                    if(alu__is_greater_zero && ~alu__is_zero) begin
                        is_non_sequential = 1;
                    end 
                end
                `BRC_GEZ: begin
                    if( (alu__is_greater_zero || alu__is_zero) ) begin
                        is_non_sequential = 1;
                    end
                end
                `BRC_LTZ: begin
                    if(~alu__is_greater_zero && ~alu__is_zero) begin
                        is_non_sequential = 1;
                    end
                end
                `BRC_LEZ: begin
                    if( (~alu__is_greater_zero || alu__is_zero) ) begin
                        is_non_sequential = 1;
                    end
                end

                // No branch!
                default: is_non_sequential = 0;
            endcase
        end

        // Hacky fix, but we don't actually use this address exception
        // anymore! So just force it to zero
        addr_excpt = 0;
    end
endmodule


////////////////////////// Memory Stage ///////////////////////////////////////
module mips_MEM_stage( // Outputs
                        write_data_MEM, write_reg_MEM_out, excpt__MEM_out, pc__MEM_in, epc__MEM_in,
                        // Inputs
                        pc__MEM_out, epc__MEM_out, mem_data_unmasked, alu__out_MEM, alu__out_mul,
                        write_reg_MEM_in, ctrl_MEM, excpt__MEM_in, write_data_EX, mem_excpt,
                        mem_is_writable, mem_translation_miss, is_store);

    // Outputs
    output reg [31:0]   write_data_MEM;
    output [4:0]        write_reg_MEM_out;
    output reg [11:0]   excpt__MEM_out;
    output [31:0]       pc__MEM_out, epc__MEM_out;

    // Inputs
    input [31:0]        pc__MEM_in, epc__MEM_in, mem_data_unmasked, alu__out_MEM, alu__out_mul, write_data_EX;
    input [4:0]         write_reg_MEM_in;
    input [4:0]         ctrl_MEM;
    input [9:0]         excpt__MEM_in;
    input               mem_excpt;
    input               mem_is_writable, mem_translation_miss, is_store;

    wire [31:0]         mem_data_masked;

    // controls
    wire [1:0]          write_data_sel_MEM, mem_access_size;
    wire                load_se, addr_excpt;
    
    wire                valid_mem_excpt;
    
    // A memory exception is only valid if we are trying to access memory
    assign valid_mem_excpt = (mem_access_size != `SIZE_NONE) && mem_excpt;

    assign pc__MEM_out = pc__MEM_in;
    assign epc__MEM_out = epc__MEM_in;
    assign write_reg_MEM_out = write_reg_MEM_in;
    
    always @(*) begin

        // Add memory exception to exception wires
        // excpt__MEM_out = {valid_mem_excpt, excpt__MEM_in};

        // MSB is the new memory exception. Then we keep the same exceptions
        // for most of the old bits. However, bits 2 and 3 are going to be
        // based on TLB misses and the TLB writable bits.
        excpt__MEM_out[11]  = 0;
        excpt__MEM_out[10]  = valid_mem_excpt;
        excpt__MEM_out[9:4] = excpt__MEM_in[9:4];
        excpt__MEM_out[3]   = 0;
        excpt__MEM_out[2]   = 0;
        excpt__MEM_out[1:0] = excpt__MEM_in[1:0];
    
        // TLBM Exception. If we are storing to an unwritable location, assert
        if(~mem_is_writable && is_store) begin
            excpt__MEM_out[11]  = 1;
        end

        // TLBL Exception. If we are accessing memory (but not storing),
        // and we miss in the TLB, assert the tlbl exception
        if( (mem_access_size != `SIZE_NONE) && mem_translation_miss && ~is_store) begin
            excpt__MEM_out[2] = 1;
        end

        // TLBS If we are accessing memory (AND storing),
        // and we miss in the TLB, assert the tlbs exception.
        if( (mem_access_size != `SIZE_NONE) && mem_translation_miss && is_store) begin
            excpt__MEM_out[3] = 1;
        end

        case(write_data_sel_MEM)
            `SEL_FROM_MEM:  write_data_MEM = mem_data_masked;
            `SEL_FROM_EX:   write_data_MEM = write_data_EX;
            `SEL_FROM_MUL:  write_data_MEM = alu__out_mul;
            default:        write_data_MEM = 32'hcafecafe;
        endcase
    end

    mask_data mem_read_masker( // Outputs
                                .mask_data_out(mem_data_masked),
                                .addr_excpt(addr_excpt),
                                // Inputs
                                .mask_data_in(mem_data_unmasked),
                                .byte_offset(alu__out_MEM[1:0]),
                                .size(mem_access_size),
                                .se(load_se));

    decode_ctrl_MEM decode_MEM( // Outputs
                                .write_data_sel_MEM(write_data_sel_MEM),
                                .mem_access_size(mem_access_size),
                                .load_se(load_se),
                                // Inputs
                                .ctrl_MEM(ctrl_MEM));
endmodule


// Takes in the access size and the byte offset and outputs the correct data
// from memory to write to the register file
module mask_data(mask_data_in, mask_data_out, byte_offset, size, se, addr_excpt);
    
    output reg [31:0] mask_data_out;
    output reg        addr_excpt;       // If exception, assert

    // Data read directly from memory, before it has been adjusted
    input [31:0]      mask_data_in;     
    input [1:0]       byte_offset, size;
    input             se; 

    reg [31:0] data_ze;

    always @(*) begin
        addr_excpt = 0; 
        data_ze = mask_data_in;

        case(size)      
            `SIZE_WORD: begin
                if(byte_offset == 0) begin
                    data_ze = mask_data_in;
                end
                else begin
                    addr_excpt = 1;
                end
                mask_data_out = data_ze;
            end

            `SIZE_HALFWORD: begin
                case(byte_offset)
                    2'h0: data_ze = (`BYTE_0_1_MASK & mask_data_in);
                    2'h1: addr_excpt = 1;
                    2'h2: data_ze = (`BYTE_2_3_MASK & mask_data_in) >> 16;
                    2'h3: addr_excpt = 1;
                endcase
                mask_data_out = se ? {{16{data_ze[15]}}, data_ze[15:0]} : data_ze;
            end

            `SIZE_BYTE: begin
                case(byte_offset)
                    2'h0: data_ze = (`BYTE_0_MASK & mask_data_in);
                    2'h1: data_ze = (`BYTE_1_MASK & mask_data_in) >> 8;
                    2'h2: data_ze = (`BYTE_2_MASK & mask_data_in) >> 16;
                    2'h3: data_ze = (`BYTE_3_MASK & mask_data_in) >> 24;
                endcase
                mask_data_out = se ? {{24{data_ze[7]}}, data_ze[7:0]} : data_ze;   
            end

            default: mask_data_out = mask_data_in;
        endcase
    end
endmodule
