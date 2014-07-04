/*
 *
 * Redistributions of any form whatsoever must retain and/or include the
 * following acknowledgment, notices and disclaimer:
 *
 * This product includes software developed by Carnegie Mellon University. 
 *
 * Copyright (c) 2004 by Babak Falsafi and James Hoe,
 * Computer Architecture Lab at Carnegie Mellon (CALCM), 
 * Carnegie Mellon University.
 *
 * This source file was written and maintained by Jared Smolens 
 * as part of the Two-Way In-Order Superscalar project for Carnegie Mellon's 
 * Introduction to Computer Architecture course, 18-447. The source file
 * is in part derived from code originally written by Herman Schmit and 
 * Diana Marculescu.
 *
 * You may not use the name "Carnegie Mellon University" or derivations 
 * thereof to endorse or promote products derived from this software.
 *
 * If you modify the software you must place a notice on or within any 
 * modified version provided or made available to any third party stating 
 * that you have modified the software.  The notice shall include at least 
 * your name, address, phone number, email address and the date and purpose 
 * of the modification.
 *
 * THE SOFTWARE IS PROVIDED "AS-IS" WITHOUT ANY WARRANTY OF ANY KIND, EITHER 
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO ANYWARRANTY 
 * THAT THE SOFTWARE WILL CONFORM TO SPECIFICATIONS OR BE ERROR-FREE AND ANY 
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, 
 * TITLE, OR NON-INFRINGEMENT.  IN NO EVENT SHALL CARNEGIE MELLON UNIVERSITY 
 * BE LIABLE FOR ANY DAMAGES, INCLUDING BUT NOT LIMITED TO DIRECT, INDIRECT, 
 * SPECIAL OR CONSEQUENTIAL DAMAGES, ARISING OUT OF, RESULTING FROM, OR IN 
 * ANY WAY CONNECTED WITH THIS SOFTWARE (WHETHER OR NOT BASED UPON WARRANTY, 
 * CONTRACT, TORT OR OTHERWISE).
 *
 */

// Include the MIPS constants
`include "mips_defines.vh"
`include "internal_defines.vh"

////
//// mips_decode: Decode MIPS instructions!
////
//// ctrl_ID        (output) - Instruction Decode control signals
//// ctrl_EX        (output) - Execute control signals
//// ctrl_MEM       (output) - Memory read / writes control signals
//// ctrl_WB        (output) - Write back to register file control signals
//// ctrl_OTHER     (output) - Random other control signals (??? what)
//// op             (input)  - Instruction opcode
//// funct2         (input)  - Instruction minor opcode
//// rt             (input)  - Instruction minor opcode
module mips_decode( /*AUTOARG*/
                    // Outputs
                    ctrl_ID, ctrl_EX, ctrl_MEM, ctrl_WB, ctrl_OTHER, reads_rs, reads_rt,
                    // Inputs
                    dcd_op, dcd_funct2, rt, rs, rd, inst);

    // Output control signals in packets depending on which stage they are used 
    input  [5:0]    dcd_op, dcd_funct2;
    input  [31:0]   inst;
    input  [4:0]    rt, rs, rd;
    output [9:0]    ctrl_ID;
    output [14:0]   ctrl_EX;
    output [4:0]    ctrl_MEM;
    output [4:0]    ctrl_WB;
    output [3:0]    ctrl_OTHER; // WILL CHANGE IN FUTURE.
    output reg      reads_rs, reads_rt;

    //// These will be output in packets depending on what stage 
    //// Instruction Decode Stage
    //// write_reg_sel      (output) - Write register is Rd, Rt, or $31
    //// alu__op1_sel       (output) - rs or shamt 
    //// alu__op2_sel       (output) - rt or immediate
    //// se_imm             (output) - Signed extend or zero extend
    reg [1:0]   write_reg_sel, alu__op1_sel, alu__op2_sel;
    reg         se_imm;
    reg [2:0]   coprocessor_read_sel;
    
    //// Execute Stage
    //// alu_sel            (output) - Selects the ALU function
    reg  [4:0]  alu__sel;
    reg  [1:0]  mem_access_size;
    reg         is_store;
    reg  [1:0]  jump_sel;
    reg  [2:0]  branch_type;
    reg  [1:0]  write_data_sel_EX;

    //// Memory Stage   
    //// mem_access_size    (output) - Either Word, Halfword or byte 
    //// is_store           (output) - Asserted if we are a memory store
    //// write_data_sel_MEM (output) - alu, mem, PC + 8, or from multiplier into the regFile
    //// load_se            (output) - ????????????????
    reg  [1:0]  write_data_sel_MEM;
    reg         load_se;

    //// Write Back Stage
    //// rd_we              (output) - Write to the register file
    reg         rd_we, tlb_we; // TODO set this to zero for everything but tlb writes
    reg [2:0]   coprocessor_write_sel;

    //// Other Signals
    //// jump_sel           (output) - Determines what type of jump, if any we are doing 
    //// branch_type        (output) - Determines what type of branch, if any we are doing
    //// ctrl_Sys           (output) - System call exception
    //// ctrl_RI            (output) - Reserved instruction exception
    reg         ctrl_Sys, ctrl_RI, testdone, is_eret;

    //  Combine the signals so we can pipe them through together
    assign ctrl_ID =    {write_reg_sel, alu__op1_sel, alu__op2_sel, se_imm, coprocessor_read_sel};
    assign ctrl_EX =    {alu__sel, mem_access_size, is_store, jump_sel, branch_type, write_data_sel_EX};
    assign ctrl_MEM =   {write_data_sel_MEM, mem_access_size, load_se};
    assign ctrl_WB =    {tlb_we, rd_we, coprocessor_write_sel};
    assign ctrl_OTHER = {testdone, ctrl_Sys, ctrl_RI, is_eret};

    always @(*) begin

        // Not writing to TLB normally
        tlb_we = 1'b0;

        // Normally, this will be a read (as we don't want a write!)
        coprocessor_read_sel = `COPROC_READ_NONE;
        coprocessor_write_sel = `COPROC_WRITE_NONE;
    
        // Assuming we are not doing an ERET instruction
        is_eret = 1'b0;

        // Don't care for ALU - we will set for practically every operation anyways. So just add
        alu__sel = 4'h0;
        
        // We assume we will be writing to the register file
        rd_we = 1'b1;

        // No idea what this is used for, just keep zero for now
        ctrl_Sys = 1'b0;
        ctrl_RI = 1'b0;
        testdone = 1'b0;
        
        // We assume we are doing an I-type instruction
        alu__op1_sel =  `ALU_OP1_REG;
        alu__op2_sel =  `ALU_OP2_IMM;
        write_reg_sel = `WR_REG_RT;

        // Majority of them are sign extended immediates
        se_imm = 1'b1;
    
        // Most of the data writes are from the EX, memory ops and mult are rarer
        write_data_sel_MEM = `SEL_FROM_EX;

        // Assume we are not accessing any memory
        mem_access_size = `SIZE_NONE;

        // Usually not a store
        is_store = 0;
     
        // Most loads are sign extended
        load_se = 1;

        // Most of the time we forward from alu in EX stage
        write_data_sel_EX = `SEL_FROM_ALU;

        // Most of the time, we are not jumping
        jump_sel =    `JUMP_NONE;
        branch_type = `BRC_NONE;

        // assume not using both rs and rt 
        reads_rt = 0;
        reads_rs = 0;
        
        if(inst == `INST_ERET) begin
            is_eret = 1;
        end
        else if(inst == `INST_TESTDONE) begin
            testdone = 1'b1;
            rd_we = 1'b0;
        end

        // TLB instructions
        else if(inst == `INST_TLBWI) begin
            tlb_we = 1'b1;
            rd_we = 1'b0;
            coprocessor_read_sel = `INDEX_READ;
        end
        else if(inst == `INST_TLBWR) begin
            tlb_we = 1'b1;
        end

        else begin

        // Given an opcode, do stuff
        case(dcd_op)

            // These are the R type Instructions
            `OP_OTHER0: begin
                rd_we = 1;
                alu__op2_sel   = `ALU_OP2_REG; // Set op2 for R-type instruction
                write_reg_sel  = `WR_REG_RD;
                write_data_sel_MEM = `SEL_FROM_EX;

                case(dcd_funct2)
                    `OP0_SYSCALL: begin
                        ctrl_Sys = 1'b1;
                        reads_rs = 0;
                        reads_rt = 0;
                    end
                    `OP0_SLL: begin // Logical Shift Left
                        alu__sel     = `ALU_SLL;
                        alu__op1_sel = `ALU_OP1_SHIFT;
                        reads_rs = 0;
                        reads_rt = 1;
                    end
                    `OP0_SRL: begin // Logical Shift Right
                        alu__sel     = `ALU_SRL;
                        alu__op1_sel = `ALU_OP1_SHIFT;
                        reads_rs = 0;
                        reads_rt = 1;
                    end
                    `OP0_SRA: begin // Arithmetic Shift Right
                        alu__sel     = `ALU_SRA;
                        alu__op1_sel = `ALU_OP1_SHIFT;
                        reads_rs = 0;
                        reads_rt = 1;
                    end
                    `OP0_SLLV: begin // Logical Shift Left Variable
                        alu__sel = `ALU_SLL;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_SRAV: begin // Arithmetic Shift Right Variable
                        alu__sel = `ALU_SRA;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_SRLV: begin // Arithmetic Shfit Right Variable
                        alu__sel = `ALU_SRL;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
    
                    // Multiply co-proccessor functions
                    `OP0_MULT: begin 
                        alu__sel = `ALU_MULT;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_MULTU: begin
                        alu__sel = `ALU_MULTU;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_DIV: begin
                        alu__sel = `ALU_DIV;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_DIVU: begin
                        alu__sel = `ALU_DIVU;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_MFHI: begin
                        alu__sel = `ALU_MFHI;
                        reads_rs = 0;
                        reads_rt = 0;
                        write_data_sel_MEM = `SEL_FROM_MUL;
                    end  
                    `OP0_MFLO: begin
                        alu__sel = `ALU_MFLO;
                        reads_rs = 0;
                        reads_rt = 0;
                        write_data_sel_MEM = `SEL_FROM_MUL;
                    end
                    `OP0_MTHI: begin
                        alu__sel = `ALU_MTHI;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
                    `OP0_MTLO: begin
                        alu__sel = `ALU_MTLO;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
    
                    // Arithmetic Instructions
                    `OP0_ADD: begin
                        alu__sel = `ALU_ADD;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_ADDU: begin
                        alu__sel = `ALU_ADDU;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_SUB: begin
                        alu__sel = `ALU_SUB;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_SUBU: begin
                        alu__sel = `ALU_SUBU;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
    
                    // Boolean Instructions
                    `OP0_AND: begin
                        alu__sel = `ALU_AND;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_OR: begin
                        alu__sel = `ALU_OR;
                        reads_rs = 1;   
                        reads_rt = 1;
                    end
                    `OP0_XOR: begin
                        alu__sel = `ALU_XOR;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_NOR: begin
                        alu__sel = `ALU_NOR;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
    
                    // Set Less Then
                    `OP0_SLT: begin
                        alu__sel = `ALU_SLT;
                        reads_rs = 1;
                        reads_rt = 1;
                    end
                    `OP0_SLTU: begin
                        alu__sel = `ALU_SLTU;
                        reads_rs = 1;
                        reads_rt = 1;
                    end

                    // Jumps, but they are marked as register instructions
                    `OP0_JR: begin
                        jump_sel = `JUMP_REG;
                        rd_we = 1'b0;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
    
                    `OP0_JALR: begin
                        jump_sel       = `JUMP_REG;
                        write_reg_sel  = `WR_REG_RD;
                        write_data_sel_EX = `SEL_FROM_PC4;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
                    
                    default: begin
                        ctrl_RI = 1'b1;
                        rd_we   = 1'b0;
                        is_store = 1'b0;
                    end
                endcase
            end // End OP_OTHER0
            // End of R-type Instructions ///////////////////////////////////////
            ////////////////////////////////////////////////////////////////////
    
            // These are the I-type Instructions
            // Arithemetic
            `OP_ADDI: begin
                alu__sel = `ALU_ADD;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_ADDIU: begin
                alu__sel = `ALU_ADDU;
                reads_rs = 1;
                reads_rt = 0;
            end
    
            // Boolean
            `OP_ANDI: begin
                se_imm = 0;
                alu__sel = `ALU_AND;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_ORI: begin
                se_imm = 0;
                alu__sel = `ALU_OR;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_XORI: begin
                se_imm = 0;
                alu__sel = `ALU_XOR;
                reads_rs = 1;
                reads_rt = 0;
            end
    
            `OP_SLTI: begin  // Sign extended imm
                alu__sel = `ALU_SLT;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_SLTIU: begin // Sign extended imm
                alu__sel = `ALU_SLTU;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_LUI: begin // Load imm into upper regester
                alu__sel = `ALU_LUI;
                reads_rs = 0;
                reads_rt = 0;
            end
    
            // Loads
            `OP_LB: begin
                write_data_sel_MEM  = `SEL_FROM_MEM;
                alu__sel        = `ALU_ADD; // MIGHT NEED TO BE UNSIGNED
                mem_access_size = `SIZE_BYTE;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_LH: begin
                write_data_sel_MEM  = `SEL_FROM_MEM;
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_HALFWORD;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_LW: begin
                write_data_sel_MEM  = `SEL_FROM_MEM;
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_WORD;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_LBU: begin
                write_data_sel_MEM  = `SEL_FROM_MEM;
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_BYTE;
                load_se = 0;  
                reads_rs = 1;
                reads_rt = 0;   
            end
            `OP_LHU: begin
                write_data_sel_MEM  = `SEL_FROM_MEM;
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_HALFWORD;
                load_se = 0;
                reads_rs = 1;
                reads_rt = 0;
            end
    
            // Stores
            `OP_SB: begin
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_BYTE;
                rd_we = 0;
                is_store = 1;
                reads_rs = 1;
                reads_rt = 1;
            end
            `OP_SH: begin
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_HALFWORD;
                rd_we = 0;
                is_store = 1;
                reads_rs = 1;
                reads_rt = 1;
            end
            `OP_SW: begin
                alu__sel        = `ALU_ADD;
                mem_access_size = `SIZE_WORD;
                is_store = 1;
                rd_we = 0;
                reads_rs = 1;
                reads_rt = 1;
            end
            // End of I-type Instructions //////////////////////////
            ////////////////////////////////////////////////////////
            
            // Jumps
            `OP_J: begin
                jump_sel = `JUMP_IMM;
                rd_we = 0;
                reads_rs = 0;
                reads_rt = 0;
            end
            `OP_JAL: begin
                jump_sel       = `JUMP_IMM;
                write_reg_sel  = `WR_REG_31;
                write_data_sel_EX = `SEL_FROM_PC4;
                rd_we = 1;
                reads_rs = 0;
                reads_rt = 0;
            end
    
            // Branches
            `OP_BEQ: begin
                branch_type  = `BRC_EQ;
                alu__sel     = `ALU_SUB;
                alu__op2_sel = `ALU_OP2_REG;
                rd_we = 0;
                reads_rs = 1;
                reads_rt = 1;
            end
            `OP_BNE: begin
                branch_type  = `BRC_NEQ;
                alu__sel     = `ALU_SUB;
                alu__op2_sel = `ALU_OP2_REG; 
                rd_we = 0;
                reads_rs = 1;
                reads_rt = 1;
            end
            `OP_BLEZ: begin
                branch_type = `BRC_LEZ;
                alu__sel    = `ALU_BRC;
                rd_we = 0;
                reads_rs = 1;
                reads_rt = 0;
            end
            `OP_BGTZ:  begin
                branch_type = `BRC_GTZ;
                alu__sel    = `ALU_BRC;
                rd_we = 0;
                reads_rs = 1;
                reads_rt = 0;
            end
    
            // These are secondary branch instructions
            `OP_OTHER1: begin
                case(rt)
                    // Branches
                    `OP1_BLTZ: begin
                        branch_type = `BRC_LTZ;
                        alu__sel    = `ALU_BRC;
                        rd_we = 0;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
                    `OP1_BGEZ: begin
                        branch_type = `BRC_GEZ;
                        alu__sel    = `ALU_BRC;
                        rd_we = 0;
                        reads_rs = 1;
                        reads_rt = 0;
                    end     
                    `OP1_BLTZAL: begin
                        branch_type    = `BRC_LTZ;
                        alu__sel       = `ALU_BRC;
                        write_reg_sel  = `WR_REG_31;
                        write_data_sel_EX = `SEL_FROM_PC4;
                        alu__op1_sel   = `ALU_OP1_REG;
                        rd_we = 1;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
                    `OP1_BGEZAL: begin
                        branch_type    = `BRC_GEZ;
                        alu__sel       = `ALU_BRC;
                        write_reg_sel  = `WR_REG_31;
                        write_data_sel_EX = `SEL_FROM_PC4;
                        alu__op1_sel   = `ALU_OP1_REG;
                        rd_we = 1;
                        reads_rs = 1;
                        reads_rt = 0;
                    end
                endcase
            end

            // These are for the coprocessor / exceptions
            `OP_Z0: begin   
                case(rs)
                    `OPZ_MFCZ: begin

                        // We will be writing to the register file
                        rd_we = 1'b1;
                        write_reg_sel = `WR_REG_RT;

                        // We will be using the values from the coprocessor
                        // register to write back eventually
                        write_data_sel_EX = `SEL_FROM_COPROC;
                        case(rd)
                            `CR_EPC:        coprocessor_read_sel = `EPC_READ;
                            `CR_BADVADDR:   coprocessor_read_sel = `BADVADDR_READ;
                            `CR_STATUS:     coprocessor_read_sel = `STATUS_READ;
                            `CR_CAUSE:      coprocessor_read_sel = `CAUSE_READ;
                            `CR_ENTRYLO:    coprocessor_read_sel = `ENTRYLO_READ;
                            `CR_ENTRYHI:    coprocessor_read_sel = `ENTRYHI_READ;
                            `CR_INDEX:      coprocessor_read_sel = `INDEX_READ;
                            default:        coprocessor_read_sel = `CAUSE_READ;
                        endcase
                    end
                    `OPZ_MTCZ: begin

                        // We are passing through op2 (rt)
                        alu__sel =      `ALU_PASS_OP2;
                        alu__op2_sel =  `ALU_OP2_REG;

                        // We won't be writing to the register file
                        rd_we = 1'b0;
            
                        // Data from EX
                        write_data_sel_MEM = `SEL_FROM_EX;
        
                        // Data from ALU
                        write_data_sel_EX = `SEL_FROM_ALU;
    
                        reads_rt = 1;
                        case(rd)
                            `CR_EPC:        coprocessor_write_sel = `EPC_WRITE;
                            `CR_BADVADDR:   coprocessor_write_sel = `BADVADDR_WRITE;
                            `CR_STATUS:     coprocessor_write_sel = `STATUS_WRITE;
                            `CR_CAUSE:      coprocessor_write_sel = `CAUSE_WRITE;
                            `CR_ENTRYLO:    coprocessor_write_sel = `ENTRYLO_WRITE;
                            `CR_ENTRYHI:    coprocessor_write_sel = `ENTRYHI_WRITE;
                            `CR_INDEX:      coprocessor_write_sel = `INDEX_WRITE;
                            default:        coprocessor_write_sel = `COPROC_WRITE_NONE;
                        endcase
                    end                     
                endcase 
            end
            default: begin
                ctrl_RI = 1'b1;
                rd_we = 1'b0;
                is_store = 1'b0;
            end
        endcase // case(op)
        end
    end
endmodule


// Instruction Decode Stage - 10 bits wide
module decode_ctrl_ID(  // Outputs
                        write_reg_sel, alu__op1_sel, alu__op2_sel, se_imm, coprocessor_read_sel,
                        // Inputs
                        ctrl_ID);
   
    output [1:0] write_reg_sel, alu__op1_sel, alu__op2_sel;
    output [2:0] coprocessor_read_sel;
    output       se_imm;
    input  [9:0] ctrl_ID;

    assign write_reg_sel = ctrl_ID[9:8];
    assign alu__op1_sel  = ctrl_ID[7:6];
    assign alu__op2_sel  = ctrl_ID[5:4];
    assign se_imm        = ctrl_ID[3];
    assign coprocessor_read_sel = ctrl_ID[2:0];
endmodule


// Execute Stage - 14 bits wide
module decode_ctrl_EX(  // Outputs
                        alu__sel,
                        mem_access_size,
                        is_store,
                        jump_sel,
                        branch_type,
                        write_data_sel_EX,
                        // Inputs
                        ctrl_EX);
                        
    output [4:0] alu__sel;
    output [1:0] mem_access_size;
    output       is_store; 
    output [1:0] jump_sel;
    output [2:0] branch_type;
    output [1:0] write_data_sel_EX;
    input  [14:0] ctrl_EX;
    
    assign alu__sel = ctrl_EX[14:10];
    assign mem_access_size = ctrl_EX[9:8];
    assign is_store = ctrl_EX[7];
    assign jump_sel = ctrl_EX[6:5];
    assign branch_type = ctrl_EX[4:2];
    assign write_data_sel_EX = ctrl_EX[1:0];
    
endmodule


// Memory Stage - 5 bits wide
module decode_ctrl_MEM( // Outputs
                        write_data_sel_MEM, mem_access_size, load_se,
                        // Inputs
                        ctrl_MEM);

    output [1:0] write_data_sel_MEM, mem_access_size;
    output       load_se;
    input  [4:0] ctrl_MEM;
  
    assign write_data_sel_MEM = ctrl_MEM[4:3];
    assign mem_access_size = ctrl_MEM[2:1];
    assign load_se = ctrl_MEM[0];

endmodule


// Write Back Stage - 5 bits wide
module decode_ctrl_WB(  // Outputs
                        tlb_we, rd_we, coprocessor_write_sel,
                        // Inputs
                        ctrl_WB);

    output          tlb_we, rd_we;
    output  [2:0]   coprocessor_write_sel;
    input   [4:0]   ctrl_WB;
    
    assign tlb_we = ctrl_WB[4];
    assign rd_we = ctrl_WB[3];
    assign coprocessor_write_sel = ctrl_WB[2:0];
endmodule


// Other Signals - 4 bits wide
module decode_ctrl_OTHER(   // Outputs
                            ctrl_Sys, ctrl_RI, testdone, is_eret,
                            // Inputs
                            ctrl_OTHER);

    output       ctrl_Sys, ctrl_RI, testdone, is_eret;
    input  [3:0] ctrl_OTHER;
    
    assign testdone = ctrl_OTHER[3];
    assign ctrl_Sys = ctrl_OTHER[2];
    assign ctrl_RI = ctrl_OTHER[1];
    assign is_eret = ctrl_OTHER[0];
endmodule
