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

//////
////// MIPS 447: A single-cycle MIPS ISA simulator
//////

// Include the MIPS constants
// `default_nettype none
`include "mips_defines.vh"
`include "internal_defines.vh"

////
////   The MIPS standalone processor module
////
////   inst_addr    (output) - Address of instruction to load
////   mem_addr     (output) - Address of data to load
////   mem_data_in  (output) - Data for memory store
////   mem_write_en (output) - Memory write mask
////   halted       (output) - Processor halted
////   allow_kernel (output) - Asserted when we are in kernel mode
////   clk          (input)  - The clock
////   reset        (input)  - Reset the processor
////   inst_excpt   (input)  - inst_addr not valid
////   mem_excpt    (input)  - mem_addr not valid
////   inst         (input)  - Instruction from memory
////   mem_data_out (input)  - Data from memory load

module mips_core(/*AUTOARG*/
                 // Outputs
                 inst_addr, mem_addr, mem_data_in, mem_write_en, halted, allow_kernel,
                 // Inputs
                 clk, rst_b, inst_excpt, mem_excpt, inst, mem_data_out);

    parameter text_start  = `KERNEL_START; /* Initial value of $pc */
  
    // Core Interface
    output [29:0] inst_addr;
    output [29:0] mem_addr;
    output [31:0] mem_data_in;
    output [3:0]  mem_write_en;
    output        halted;
    output        allow_kernel;
    input         clk, rst_b, inst_excpt, mem_excpt;
    input  [31:0] inst, mem_data_out;

    // Internal Signals
    // pc               - Program Counter
    // nextpc           - The next value of the program counter
    // branch_mispred   - Asserted when we mispredict a branch
    // internal_ halt   - Used for testdone instruction
    wire [31:0]  pc, pc_plus4;
    reg  [31:0]  nextpc;
    reg          branch_mispredict_clear;
    wire         internal_halt;
    
    // Wires used for memory translations
    wire [31:0] inst_physical_address, mem_physical_address;
    wire [31:0] inst_virtual_address, mem_virtual_address;
    wire        mem_translation_miss, mem_is_writable;

    // Decode signals
    wire [5:0]  dcd_op, dcd_funct2;
    wire [4:0]  dcd_rt, dcd_rs, dcd_rd;
    
    // ctrl_RI - Reserved instruction exception
    wire        testdone, ctrl_RI;
    
    // Wire for PC
    wire        pc_enable;

    // From memory decoder
    wire [9:0]  ctrl_ID;
    wire [3:0]  ctrl_OTHER; // ctrl_OTHER__EX, ctrl_OTHER__MEM, ctrl_OTHER__WB;
    wire        reads_rs, reads_rt;

    // Wires from forwarding unit
    wire [31:0] forwarded_rs_data__FWD, forwarded_rt_data__FWD, forwarded_coproc_data__FWD;
    wire        is_forward__rs_data__FWD, is_forward__rt_data__FWD, is_forward__coproc_data__FWD;
    wire [31:0] forwarded_entryLo__FWD, forwarded_entryHi__FWD, forwarded_index__FWD;
    wire        is_forward__entryLo__FWD, is_forward__entryHi__FWD, is_forward__index__FWD;
        
    // Wires from staller
    wire        stall;

    // Wires for exceptions
    wire        is_kernel_mode;
    wire [3:0]  excpt__IF, excpt__ID_in;
    wire [7:0]  excpt__ID_out, excpt__EX_in;
    wire [9:0]  excpt__EX_out, excpt__MEM_in;
    wire [11:0] excpt__MEM_out, excpt__WB;
    wire        excpt__clear_IF_ID, excpt__clear_ID_EX, excpt__clear_EX_MEM, excpt__clear_MEM_WB;
    wire        excpt__clear_ALL;
    wire [31:0] excpt__addressToJump;
    wire        excpt__is_jump;
    wire        excpt__is_epc_write;
    wire [31:0] excpt__epc_data;
    wire [4:0]  excpt__cause_data_in;
    wire        tlb_miss__WB;

    // Control wire ctrl_WB
    wire [4:0]  ctrl_WB, ctrl_WB__ID, ctrl_WB__EX, ctrl_WB__MEM, ctrl_WB__WB;

    // Control wire ctrl_MEM
    wire [4:0]  ctrl_MEM, ctrl_MEM__ID, ctrl_MEM__EX, ctrl_MEM__MEM;

    // Control wire ctrl_EX
    wire [14:0] ctrl_EX, ctrl_EX__ID, ctrl_EX__EX;

    // Wires for ID stage
    
    // Wires for IF to ID
    wire [31:0] inst_ID;
    wire [31:0] pc__ID_in;

    // Wires for Instruction Decode to Execute Stage
    wire [31:0] inst_ID_out, inst_EX;
    wire [31:0] alu__op1__ID, alu__op1__EX;
    wire [31:0] alu__op2__ID, alu__op2__EX;
    wire [31:0] rt_data__ID, rt_data__EX;
    wire [31:0] rs_data__ID, rs_data__EX;
    wire [4:0]  write_reg__ID, write_reg__EX_in;
    wire [31:0] pc__ID_out, pc__EX_in;
    wire [31:0] epc__ID_out, epc__EX_in;
    wire [31:0] coprocessor_data_ID, coprocessor_data_EX;
    wire [31:0] entryLo__ID;
    wire [31:0] entryHi__ID;
    wire [31:0] index__ID;

    // Wires for Execute Stage to Memory Stage
    wire [31:0] alu__out__EX, alu__out__MEM, alu__out_mul;
    wire [4:0]  write_reg__EX_out, write_reg__MEM_in;
    wire [29:0] mem_addr__EX, mem_addr__MEM;
    wire [3:0]  mem_write_en__EX, mem_write_en__MEM;
    wire [31:0] mem_data__EX, mem_data__MEM;
    wire        alu__is_zero__EX, alu__is_greater_zero__EX, alu__excpt__EX;
    wire        alu__mul_stall__EX;
    wire [31:0] pc__EX_out, pc__MEM_in;
    wire [31:0] epc__EX_out, epc__MEM_in;
    wire [31:0] write_data_EX__EX, write_data_EX__MEM; // Data forwarded out of EX stage
    wire [31:0] entryLo__EX;
    wire [31:0] entryHi__EX;
    wire [31:0] index__EX;
    wire        is_store__EX;

    // Wires for the Memory Stage to Write Back Stage
    wire [31:0] write_data__MEM, write_data__WB;
    wire [4:0]  write_reg__MEM_out, write_reg__WB;
    wire [31:0] pc__MEM_out, pc__WB;
    wire [31:0] epc__MEM_out, epc__WB;
    wire [31:0] entryLo__MEM, entryLo__WB;
    wire [31:0] entryHi__MEM, entryHi__WB;
    wire [31:0] index__MEM, index__WB;
    wire [31:0] badVaddr__WB;
    wire        is_store__MEM;


    // Used for finding the next pc
    wire        is_non_sequential;
    wire [31:0] branch_address;

    // Tells us if we are in kernel space
    assign allow_kernel = is_kernel_mode;

    ////////////////// Program Counter ////////////////////////////////////////          
    // If we assert enable or stall, we want the pc to STOP incrementing
    assign testdone = excpt__WB[7];
    assign pc_enable = !(internal_halt || stall || testdone);
    register #(32, text_start)   PCReg(pc, nextpc, clk, pc_enable, rst_b);

    // Always assume PC + 4
    adder calculate_pc_plus4(.sum(pc_plus4),
                             .in1(pc),
                             .in2(32'h4));
                        
    // Set next pc based on the branch address, and then figure out when
    // to clear flops on a branch misprediction               
    always @(*) begin
        if(excpt__is_jump) begin
            nextpc = excpt__addressToJump;
            branch_mispredict_clear = 1'b0;     
        end
        else if(is_non_sequential) begin
            nextpc = branch_address;
            branch_mispredict_clear = 1'b1;
        end
        else begin
            nextpc = pc_plus4;
            branch_mispredict_clear = 1'b0;
        end
    end

    // adel and ebe exceptions
    reg adel_exception;

    always @(*) begin
        if( (pc[1:0] != 0) || (~is_kernel_mode && (pc >= `KSEG0_START)) ) begin
            adel_exception = 1'b1;
        end
        else begin
            adel_exception = 1'b0;
        end
    end

    assign excpt__IF[0] = inst_excpt;
    assign excpt__IF[1] = adel_exception;

    // For Lab5, we don't have to support TLB exceptions in the IF stage
    assign excpt__IF[2] = 0;
    assign excpt__IF[3] = 0;

    // Halts for test done
    assign internal_halt = testdone;
    register #(1, 0)  Halt(halted, internal_halt, clk, 1'b1, rst_b);

    // IF to ID assigns
    assign ctrl_WB__ID  = ctrl_WB;
    assign ctrl_MEM__ID = ctrl_MEM;
    assign ctrl_EX__ID  = ctrl_EX;

    // EX to Memory
    assign mem_addr     = mem_physical_address[31:2];
    assign mem_virtual_address = {mem_addr__MEM, 2'b00};
    assign mem_write_en = mem_write_en__MEM;
    assign mem_data_in  = mem_data__MEM;

    ///////////////////////////////////////////////////////////////////////////
    // Generate control signals

    // Translate the PC virtual address to physical addresses
    assign        inst_addr = inst_physical_address[31:2];
    assign        inst_virtual_address = pc;

    // Instruction decoding
    assign        dcd_op =     inst_ID[31:26];  // Opcode
    assign        dcd_rt =     inst_ID[20:16];  // rt field
    assign        dcd_rs =     inst_ID[25:21];  // NOTE, I ADDED THIS. ROSS IS THIS OK?? for stall...rs field
    assign        dcd_rd =     inst_ID[15:11];  // rd field
    assign        dcd_funct2 = inst_ID[5:0];    // funct field; secondary opcode

    assign excpt__ID_out = {ctrl_OTHER, excpt__ID_in};

    // Decoder
    mips_decode         Decoder(    // Outputs
                                    .ctrl_ID(ctrl_ID),
                                    .ctrl_EX(ctrl_EX),
                                    .ctrl_MEM(ctrl_MEM),
                                    .ctrl_WB(ctrl_WB),
                                    .ctrl_OTHER(ctrl_OTHER),
                                    .reads_rs(reads_rs),
                                    .reads_rt(reads_rt),
                                    // Inputs
                                    .inst(inst_ID),
                                    .dcd_op(dcd_op),
                                    .dcd_funct2(dcd_funct2),
                                    .rt(dcd_rt),
                                    .rs(dcd_rs),
                                    .rd(dcd_rd));

    // Figures out when to stall
    stall_logic         Stalls(     // Outputs
                                    .stall(stall),
                                    // Inputs
                                    .dcd_rs(dcd_rs),
                                    .dcd_rt(dcd_rt),
                                    .write_reg__EX(write_reg__EX_in),
                                    .reads_rs(reads_rs),
                                    .reads_rt(reads_rt),
                                    .ctrl_EX(ctrl_EX__EX),   
                                    .alu__mul_stall__EX(alu__mul_stall__EX));
    
    // Figures out when to forward data
    forward_logic       Forwards(   // Outputs
                                    .forwarded_rs_data__FWD(forwarded_rs_data__FWD),
                                    .forwarded_rt_data__FWD(forwarded_rt_data__FWD),
                                    .forwarded_coproc_data__FWD(forwarded_coproc_data__FWD),
                                    .is_forward__rs_data__FWD(is_forward__rs_data__FWD),
                                    .is_forward__rt_data__FWD(is_forward__rt_data__FWD),
                                    .is_forward__coproc_data__FWD(is_forward__coproc_data__FWD),
                                    .forwarded_entryLo__FWD(forwarded_entryLo__FWD),    
                                    .forwarded_entryHi__FWD(forwarded_entryHi__FWD),
                                    .forwarded_index__FWD(forwarded_index__FWD),
                                    .is_forward__entryLo__FWD(is_forward__entryLo__FWD),
                                    .is_forward__entryHi__FWD(is_forward__entryHi__FWD),
                                    .is_forward__index__FWD(is_forward__index__FWD),
                                    // Inputs
                                    .dcd_rs(dcd_rs),
                                    .dcd_rt(dcd_rt),
                                    .reads_rs(reads_rs),
                                    .reads_rt(reads_rt),
                                    .coprocessor_read_sel__ID(ctrl_ID[2:0]), // Ugly, we can fix this later
                                    .ctrl_WB__EX(ctrl_WB__EX),
                                    .ctrl_WB__MEM(ctrl_WB__MEM),
                                    .ctrl_WB__WB(ctrl_WB__WB),
                                    .write_reg__EX(write_reg__EX_out),
                                    .write_reg__MEM(write_reg__MEM_out),
                                    .write_reg__WB(write_reg__WB),
                                    .write_data_EX(write_data_EX__EX),
                                    .write_data__MEM(write_data__MEM),
                                    .write_data__WB(write_data__WB));
    
    // Figures out what to do when an exception occurs
    mips_exception_unit ExceptionUnit(  // Outputs
                                        .excpt__clear_IF_ID(excpt__clear_IF_ID), 
                                        .excpt__clear_ID_EX(excpt__clear_ID_EX),
                                        .excpt__clear_EX_MEM(excpt__clear_EX_MEM),
                                        .excpt__clear_MEM_WB(excpt__clear_MEM_WB),
                                        .excpt__clear_ALL(excpt__clear_ALL),
                                        .excpt__addressToJump(excpt__addressToJump),
                                        .excpt__is_jump(excpt__is_jump),
                                        .excpt__is_epc_write(excpt__is_epc_write),
                                        .excpt__epc_data(excpt__epc_data),
                                        .excpt__cause_data_in(excpt__cause_data_in),
                                        .set_user_mode(set_user_mode),
                                        .set_kernel_mode(set_kernel_mode),
                                        .tlb_miss(tlb_miss__WB),
                                        // Inputs
                                        .excpt__IF(excpt__IF),
                                        .excpt__ID(excpt__ID_out),
                                        .excpt__EX(excpt__EX_out),
                                        .excpt__MEM(excpt__MEM_out),
                                        .excpt__WB(excpt__WB),
                                        .PC(pc__WB),
                                        .EPC(epc__ID_out));

    // TLB Interface, used by both instructions memory and data memory  
    tlb_interface       TLB(        // Outputs
                                    .inst_physical_address(inst_physical_address),
                                    .mem_physical_address(mem_physical_address),
                                    .inst_translation_miss(/* TODO TODO */),
                                    .mem_translation_miss(mem_translation_miss),
                                    .mem_is_writable(mem_is_writable),
                                    // Inputs
                                    .inst_virtual_address(inst_virtual_address),
                                    .mem_virtual_address(mem_virtual_address),
                                    .writeIndex(index__WB[4:0]),    // Index, but only 5 lsb
                                    .writeTag(entryHi__WB),
                                    .writeData(entryLo__WB),
                                    .writeEnable(ctrl_WB__WB[4]),   // 5th bit of ctrl_WB line
                                    .clk(clk),
                                    .rst_b(rst_b));

    ///////////////////////////////////////////////////////////////////////////
    // Registers betwen Fetch and Decode stage
    if_id_flop          if_id_Flop( // Outputs
                                    .inst__ID(inst_ID),
                                    .pc__ID(pc__ID_in),
                                    .excpt__ID(excpt__ID_in),
                                    // Inputs
                                    .clk(clk),
                                    .rst_b(rst_b),
                                    .en( ~stall ),  // Stall
                                    .clr_all(branch_mispredict_clear || excpt__clear_ALL),
                                    .clr_excpt(excpt__clear_IF_ID),
                                    .inst(inst),
                                    .pc(pc),
                                    .excpt__IF(excpt__IF));

    // Instruction Decode Stage
    mips_ID_WB_stage    id_wb_stage(// ID Outputs
                                    .alu__op1_ID(alu__op1__ID),
                                    .alu__op2_ID(alu__op2__ID),
                                    .rt_data_ID(rt_data__ID),
                                    .rs_data_ID(rs_data__ID),
                                    .write_reg_ID_out(write_reg__ID),
                                    .inst_ID_out(inst_ID_out),
                                    .coprocessor_data_ID(coprocessor_data_ID),
                                    .is_kernel_mode(is_kernel_mode),
                                    .epc__ID_out(epc__ID_out),
                                    .pc__ID_out(pc__ID_out),
                                    .entryLo__ID(entryLo__ID),          
                                    .entryHi__ID(entryHi__ID),
                                    .index__ID(index__ID),
                                    // Inputs
                                    .clk(clk),
                                    .rst_b(rst_b),
                                    // ID Inputs
                                    .inst_ID(inst_ID),
                                    .pc__ID_in(pc__ID_in),          
                                    .ctrl_ID(ctrl_ID),
                                    .load_badVaddr(tlb_miss__WB),
                                    .badVaddr(badVaddr__WB),
                                    .set_kernel_mode(set_kernel_mode),
                                    .set_user_mode(set_user_mode),
                                    .halted_ID(halted), 
                                    .is_forward__rs_data__FWD(is_forward__rs_data__FWD),    
                                    .is_forward__rt_data__FWD(is_forward__rt_data__FWD),
                                    .is_forward__coproc_data__FWD(is_forward__coproc_data__FWD),
                                    .forwarded_rs_data__FWD(forwarded_rs_data__FWD),
                                    .forwarded_rt_data__FWD(forwarded_rt_data__FWD),
                                    .forwarded_coproc_data__FWD(forwarded_coproc_data__FWD),                
                                    .is_forward__entryLo__FWD(is_forward__entryLo__FWD), 
                                    .is_forward__entryHi__FWD(is_forward__entryHi__FWD),
                                    .is_forward__index__FWD(is_forward__index__FWD),
                                    .forwarded_entryLo__FWD(forwarded_entryLo__FWD),
                                    .forwarded_entryHi__FWD(forwarded_entryHi__FWD),
                                    .forwarded_index__FWD(forwarded_index__FWD),
                                    // WB Inputs
                                    .write_data_WB(write_data__WB),
                                    .write_reg_WB_in(write_reg__WB),
                                    .ctrl_WB(ctrl_WB__WB),
                                    .epc_data(excpt__epc_data),
                                    .is_epc_write(excpt__is_epc_write),
                                    .cause_data_in(excpt__cause_data_in));

    // Registers between Decode and Execute stage
    id_ex_flop          id_ex_Flop( // Outputs
                                    .ctrl_WB__EX(ctrl_WB__EX),
                                    .ctrl_MEM__EX(ctrl_MEM__EX),
                                    .ctrl_EX__EX(ctrl_EX__EX),
                                    .alu__op1__EX(alu__op1__EX),
                                    .alu__op2__EX(alu__op2__EX),
                                    .rt_data__EX(rt_data__EX),
                                    .rs_data__EX(rs_data__EX),
                                    .write_reg__EX(write_reg__EX_in),
                                    .inst__EX(inst_EX),
                                    .pc__EX(pc__EX_in),
                                    .excpt__EX(excpt__EX_in),
                                    .coprocessor_data_EX(coprocessor_data_EX),
                                    .epc__EX(epc__EX_in),   
                                    .entryLo_EX(entryLo__EX),   
                                    .entryHi_EX(entryHi__EX),
                                    .index_EX(index__EX),   
                                    // Inputs
                                    .clk(clk),
                                    .rst_b(rst_b),                                  
                                    .stall(stall),
                                    .alu__mul_stall(alu__mul_stall__EX),
                                    .branch_mispredict_clear(branch_mispredict_clear),
                                    .clr_excpt(excpt__clear_ID_EX),
                                    .clr_all_from_excpt(excpt__clear_ALL),
                                    .ctrl_WB__ID(ctrl_WB__ID),
                                    .ctrl_MEM__ID(ctrl_MEM__ID),
                                    .ctrl_EX__ID(ctrl_EX__ID),
                                    .alu__op1__ID(alu__op1__ID),
                                    .alu__op2__ID(alu__op2__ID),
                                    .rt_data__ID(rt_data__ID),
                                    .rs_data__ID(rs_data__ID),          
                                    .write_reg__ID(write_reg__ID),
                                    .inst__ID(inst_ID_out),
                                    .pc__ID(pc__ID_out),
                                    .excpt__ID(excpt__ID_out),
                                    .epc__ID(epc__ID_out),
                                    .coprocessor_data_ID(coprocessor_data_ID),
                                    .entryLo_ID(entryLo__ID),           
                                    .entryHi_ID(entryHi__ID),
                                    .index_ID(index__ID));

    // Execute Stage
    mips_EX_stage       ex_stage(   // Outputs
                                    .alu__out_EX(alu__out__EX),
                                    .alu__out_mul(alu__out_mul),
                                    .write_reg_EX_out(write_reg__EX_out),
                                    .mem_addr_EX(mem_addr__EX),
                                    .mem_write_en_EX(mem_write_en__EX),
                                    .mem_data_EX(mem_data__EX),
                                    .alu__is_zero_EX(alu__is_zero__EX),
                                    .alu__is_greater_zero_EX(alu__is_greater_zero__EX),
                                    .alu__mul_stall__EX(alu__mul_stall__EX),
                                    .next_addr(branch_address),
                                    .is_non_sequential(is_non_sequential),
                                    .pc__EX_out(pc__EX_out),
                                    .epc__EX_out(epc__EX_out),
                                    .write_data_EX(write_data_EX__EX),
                                    .excpt__EX_out(excpt__EX_out),
                                    .is_store__EX(is_store__EX),
                                    // Inputs
                                    .clk(clk),
                                    .rst_b(rst_b),
                                    .is_kernel_mode(is_kernel_mode),
                                    .alu__op1_EX(alu__op1__EX),
                                    .alu__op2_EX(alu__op2__EX),
                                    .rt_data_EX(rt_data__EX),
                                    .rs_data_EX(rs_data__EX),               
                                    .write_reg_EX_in(write_reg__EX_in),
                                    .ctrl_EX(ctrl_EX__EX),
                                    .pc__EX_in(pc__EX_in),
                                    .epc__EX_in(epc__EX_in),                            
                                    .inst(inst_EX),
                                    .excpt__EX_in(excpt__EX_in),
                                    .coprocessor_data_EX(coprocessor_data_EX));

    // Registers between Execute and Memory Stage
    ex_mem_flop         ex_mem_Flop(// Outputs
                                    .ctrl_WB__MEM(ctrl_WB__MEM),
                                    .ctrl_MEM__MEM(ctrl_MEM__MEM),
                                    .alu__out__MEM(alu__out__MEM),
                                    .write_reg__MEM(write_reg__MEM_in),
                                    .mem_addr__MEM(mem_addr__MEM),
                                    .mem_write_en__MEM(mem_write_en__MEM),
                                    .mem_data__MEM(mem_data__MEM),
                                    .pc__MEM(pc__MEM_in),
                                    .epc__MEM(epc__MEM_in),
                                    .excpt__MEM(excpt__MEM_in),
                                    .write_data_EX__MEM(write_data_EX__MEM),
                                    .entryLo__MEM(entryLo__MEM),        
                                    .entryHi__MEM(entryHi__MEM),
                                    .index__MEM(index__MEM),
                                    .is_store__MEM(is_store__MEM),
                                    // Inputs
                                    .clk(clk),
                                    .rst_b(rst_b),
                                    .en( 1'b1 ),    // Stall
                                    .clr_excpt(excpt__clear_EX_MEM),
                                    .clr_all(excpt__clear_ALL),
                                    .ctrl_WB__EX(ctrl_WB__EX),
                                    .ctrl_MEM__EX(ctrl_MEM__EX),
                                    .alu__out__EX(alu__out__EX),
                                    .write_reg__EX(write_reg__EX_out),
                                    .mem_addr__EX(mem_addr__EX),
                                    .mem_write_en__EX(mem_write_en__EX),
                                    .mem_data__EX(mem_data__EX),
                                    .pc__EX(pc__EX_out),
                                    .epc__EX(epc__EX_out),
                                    .excpt__EX(excpt__EX_out),
                                    .write_data_EX__EX(write_data_EX__EX),
                                    .entryLo__EX(entryLo__EX),      
                                    .entryHi__EX(entryHi__EX),
                                    .index__EX(index__EX),
                                    .is_store__EX(is_store__EX));

    // Memory Stage
    mips_MEM_stage      mem_stage(  // Outputs
                                    .write_data_MEM(write_data__MEM),
                                    .write_reg_MEM_out(write_reg__MEM_out),
                                    .pc__MEM_out(pc__MEM_out),
                                    .epc__MEM_out(epc__MEM_out),
                                    .excpt__MEM_out(excpt__MEM_out),
                                    // Inputs
                                    .mem_data_unmasked(mem_data_out),
                                    .alu__out_MEM(alu__out__MEM),
                                    .alu__out_mul(alu__out_mul),
                                    .write_reg_MEM_in(write_reg__MEM_in),
                                    .ctrl_MEM(ctrl_MEM__MEM),
                                    .write_data_EX(write_data_EX__MEM),
                                    .pc__MEM_in(pc__MEM_in),
                                    .epc__MEM_in(epc__MEM_in),
                                    .mem_excpt(mem_excpt),
                                    .excpt__MEM_in(excpt__MEM_in),
                                    .mem_is_writable(mem_is_writable),
                                    .mem_translation_miss(mem_translation_miss),
                                    .is_store(is_store__MEM));

    // Registers between Memory and Write Back stage    
    mem_wb_flop         mem_wb_Flop(// Outputs
                                    .ctrl_WB__WB(ctrl_WB__WB),
                                    .write_reg__WB(write_reg__WB),
                                    .write_data__WB(write_data__WB),
                                    .pc__WB(pc__WB),
                                    .epc__WB(epc__WB),
                                    .excpt__WB(excpt__WB),
                                    .entryLo__WB(entryLo__WB),  
                                    .entryHi__WB(entryHi__WB),
                                    .index__WB(index__WB),
                                    .badVaddr__WB(badVaddr__WB),
                                    // Inputs
                                    .clk(clk),
                                    .rst_b(rst_b),
                                    .en( 1'b1 ),    //Stall
                                    .clr_excpt(excpt__clear_MEM_WB),
                                    .clr_all(excpt__clear_ALL),
                                    .ctrl_WB__MEM(ctrl_WB__MEM),
                                    .write_reg__MEM(write_reg__MEM_out),
                                    .write_data__MEM(write_data__MEM),
                                    .pc__MEM(pc__MEM_out),
                                    .epc__MEM(epc__MEM_out),
                                    .excpt__MEM(excpt__MEM_out),
                                    .entryLo__MEM(entryLo__MEM),    
                                    .entryHi__MEM(entryHi__MEM),
                                    .index__MEM(index__MEM),
                                    .badVaddr__MEM(mem_virtual_address));



    ///////////////////// DEBUGGING SECTION ///////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////
    // synthesis translate_off
    always @(posedge clk) begin
        // useful for debugging, you will want to comment this out for long programs

        if (rst_b) begin
          // $display ( "\n=== Simulation Cycle %d ===", $time );
          // $display("clk=%x,pc=%x,nextpc=%x,en=%x",clk,pc,nextpc,pc_enable);
        end
    end
    // synthesis translate_on
endmodule // mips_core


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// Flops Between Stages ///////////////////////////////////////////////
// Flop between the fetch and decode stage
module if_id_flop(  // Outputs
                    inst__ID, pc__ID, excpt__ID,
                    // Inputs
                    clk, rst_b, en, clr_all, clr_excpt, inst, pc, excpt__IF);

    // Outputs
    output [31:0]   inst__ID, pc__ID;
    output [3:0]    excpt__ID;
    // Inputs

    input           clk, rst_b, en, clr_all, clr_excpt;
    input [31:0]    inst, pc;
    input [3:0]     excpt__IF;  

    // Clear all registers most of the time, but on exceptions don't clear the exception bits
    registerWithClear #(32, 0)  inst_reg(     .q(inst__ID),  .d(inst),      .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  pc_reg  (     .q(pc__ID),    .d(pc),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
    registerWithClear #(4, 0)   excpt__ID_EX( .q(excpt__ID), .d(excpt__IF), .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
endmodule

// Flop bewteen the decode and execute stage
module id_ex_flop(  // Outputs
                    ctrl_WB__EX, ctrl_MEM__EX, ctrl_EX__EX,
                    alu__op1__EX, alu__op2__EX, rt_data__EX, rs_data__EX, write_reg__EX, coprocessor_data_EX,
                    inst__EX, pc__EX, epc__EX, excpt__EX, entryLo_EX, entryHi_EX, index_EX,
                    // Inputs
                    clk, rst_b, stall, alu__mul_stall, branch_mispredict_clear, clr_excpt, clr_all_from_excpt,
                    ctrl_WB__ID, ctrl_MEM__ID, ctrl_EX__ID,
                    alu__op1__ID, alu__op2__ID, rt_data__ID, rs_data__ID, write_reg__ID, coprocessor_data_ID,
                    inst__ID, pc__ID, epc__ID, excpt__ID, entryLo_ID, entryHi_ID, index_ID);
                    
    // Outputs
    output [4:0]        ctrl_WB__EX;
    output [4:0]        ctrl_MEM__EX;
    output reg [14:0]   ctrl_EX__EX;
    output [31:0]       alu__op1__EX, alu__op2__EX;
    output [31:0]       rt_data__EX, rs_data__EX;
    output [4:0]        write_reg__EX;
    output [31:0]       inst__EX, pc__EX, epc__EX, coprocessor_data_EX;
    output [7:0]        excpt__EX;
    output [31:0]       entryLo_EX, entryHi_EX, index_EX;

    // Inputs
    input               clk, rst_b, clr_excpt, clr_all_from_excpt;
    input               stall, alu__mul_stall, branch_mispredict_clear;
    input [4:0]         ctrl_WB__ID;
    input [4:0]         ctrl_MEM__ID;
    input [14:0]        ctrl_EX__ID;
    input [31:0]        alu__op1__ID, alu__op2__ID;
    input [31:0]        rt_data__ID, rs_data__ID;
    input [4:0]         write_reg__ID;
    input [31:0]        inst__ID, pc__ID, epc__ID, coprocessor_data_ID;
    input [7:0]         excpt__ID;
    input [31:0]        entryLo_ID, entryHi_ID, index_ID;
    
    wire en, clr_except_alu_sel, clr_all;

    // If we're not stalling, enabled
    assign en = ~stall;

    // This creates a bubble if we had just stalled (but not because of MULT), or if we had a misprediction
    assign clr_except_alu_sel = (stall && ~alu__mul_stall) || branch_mispredict_clear;

    // We want to clear everything when we mispredict a branch
    assign clr_all = branch_mispredict_clear || clr_all_from_excpt;
    
    // Instantiate the registers for the individual signals
    // We need the abilty to add a bubble. Note that en == !stall When we hit a stall, we will clear this register to create a bubble.
    // On exceptions we will clear everything but those registers involved with exceptions (pc, epc, and exception bits)
    registerWithClear #(5, 0)  ctrl_WB(          .q(ctrl_WB__EX),         .d(ctrl_WB__ID),         .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all)); 
    registerWithClear #(5, 0)  ctrl_MEM(         .q(ctrl_MEM__EX),        .d(ctrl_MEM__ID),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) alu__op1(         .q(alu__op1__EX),        .d(alu__op1__ID),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) alu__op2(         .q(alu__op2__EX),        .d(alu__op2__ID),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) rt_data(          .q(rt_data__EX),         .d(rt_data__ID),         .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) rs_data(          .q(rs_data__EX),         .d(rs_data__ID),         .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) inst(             .q(inst__EX),            .d(inst__ID),            .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(5, 0)  write_reg(        .q(write_reg__EX),       .d(write_reg__ID),       .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) coprocessor_data( .q(coprocessor_data_EX), .d(coprocessor_data_ID), .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) entryLo(          .q(entryLo_EX),          .d(entryLo_ID),          .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) entryHi(          .q(entryHi_EX),          .d(entryHi_ID),          .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) index(            .q(index_EX),            .d(index_ID),            .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_excpt || clr_all));
    registerWithClear #(32, 0) pc(               .q(pc__EX),              .d(pc__ID),              .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_all));
    registerWithClear #(32, 0) epc(              .q(epc__EX),             .d(epc__ID),             .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_all));
    registerWithClear #(8, 0)  excpt__ID_EX(     .q(excpt__EX),           .d(excpt__ID),           .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_except_alu_sel || clr_all));  


    // Also have to clear out the is_store bit in ctrl_EX in bubbles. So
    // create a seperate register and mask it when we have to clear it, so
    // only is_store is zeroed out. Only want to clear when we're writing to the register   
    always @(posedge clk or negedge rst_b) begin 
        if (~rst_b) begin
            ctrl_EX__EX <= 0;
        end

        // Clear it completely on clear all and when we have an exception to
        // nullify this instruction
        else if (clr_all || clr_excpt) begin
            ctrl_EX__EX <= 14'b0;
        end

        // Enabled, take from ID stage
        else if (clr_except_alu_sel & en) begin
            ctrl_EX__EX <= (ctrl_EX__ID & `CTRL_EX_MASK);
        end

        // Not enabled, keep from EX stage
        else if (clr_except_alu_sel & ~en) begin
            ctrl_EX__EX <= (ctrl_EX__EX & `CTRL_EX_MASK);
        end
        else if (en) begin
            ctrl_EX__EX <= ctrl_EX__ID;
        end
    end 
endmodule


// Flop bewteen the execute and memory stage
module ex_mem_flop( // Outputs
                    ctrl_WB__MEM, ctrl_MEM__MEM,
                    alu__out__MEM, write_reg__MEM,
                    mem_addr__MEM, mem_write_en__MEM, mem_data__MEM, write_data_EX__MEM,
                    pc__MEM, epc__MEM, excpt__MEM, entryLo__MEM, entryHi__MEM, index__MEM, is_store__MEM,
                    // Inputs
                    clk, rst_b, en, clr_excpt, clr_all,
                    ctrl_WB__EX, ctrl_MEM__EX,
                    alu__out__EX, write_reg__EX,
                    mem_addr__EX, mem_write_en__EX, mem_data__EX, write_data_EX__EX,
                    pc__EX, epc__EX, excpt__EX, entryLo__EX, entryHi__EX, index__EX, is_store__EX);
                    
    // Outputs
    output [4:0]  ctrl_WB__MEM;
    output [4:0]  ctrl_MEM__MEM;
    output [31:0] alu__out__MEM;
    output [4:0]  write_reg__MEM;
    output [29:0] mem_addr__MEM;
    output [3:0]  mem_write_en__MEM;
    output [31:0] mem_data__MEM;
    output [31:0] pc__MEM, epc__MEM;
    output [31:0] write_data_EX__MEM;
    output [9:0]  excpt__MEM;
    output [31:0] entryLo__MEM, entryHi__MEM, index__MEM;
    output        is_store__MEM;

    // Inputs
    input        clk, rst_b, en, clr_excpt, clr_all;
    input [4:0]  ctrl_WB__EX;
    input [4:0]  ctrl_MEM__EX;
    input [31:0] alu__out__EX;
    input [4:0]  write_reg__EX;
    input [29:0] mem_addr__EX;
    input [3:0]  mem_write_en__EX;
    input [31:0] mem_data__EX;
    input [31:0] write_data_EX__EX;
    input [31:0] pc__EX, epc__EX;
    input [9:0]  excpt__EX;
    input [31:0] entryLo__EX, entryHi__EX, index__EX;
    input        is_store__EX;

    // Instantiate the registers for the individual signals!
    // We want to clear everything when we hit an exception, except those bits involved with exceptions!
    registerWithClear #(5, 0)   ctrl_WB(        .q(ctrl_WB__MEM),       .d(ctrl_WB__EX),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(5, 0)   ctrl_MEM(       .q(ctrl_MEM__MEM),      .d(ctrl_MEM__EX),       .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  alu__out(       .q(alu__out__MEM),      .d(alu__out__EX),       .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(5, 0)   write_reg(      .q(write_reg__MEM),     .d(write_reg__EX),      .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(30, 0)  mem_addr(       .q(mem_addr__MEM),      .d(mem_addr__EX),       .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(4, 0)   mem_write_en(   .q(mem_write_en__MEM),  .d(mem_write_en__EX),   .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  mem_data(       .q(mem_data__MEM),      .d(mem_data__EX),       .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  write_data(     .q(write_data_EX__MEM), .d(write_data_EX__EX),  .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  entryLo(        .q(entryLo__MEM),       .d(entryLo__EX),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  entryHi(        .q(entryHi__MEM),       .d(entryHi__EX),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  index(          .q(index__MEM),         .d(index__EX),          .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(1, 0)   is_store(       .q(is_store__MEM),      .d(is_store__EX),       .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  pc(             .q(pc__MEM),            .d(pc__EX),             .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
    registerWithClear #(32, 0)  epc(            .q(epc__MEM),           .d(epc__EX),            .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));    
    registerWithClear #(10, 0)  excpt__EX_MEM(  .q(excpt__MEM),         .d(excpt__EX),          .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
endmodule

// Flop bewteen the execute and memory stage
module mem_wb_flop( // Outputs
                    ctrl_WB__WB, pc__WB, epc__WB,
                    write_reg__WB, write_data__WB, excpt__WB,
                    entryLo__WB, entryHi__WB, index__WB, badVaddr__WB,
                    // Inputs
                    clk, rst_b, en, clr_excpt, clr_all,
                    pc__MEM, epc__MEM,
                    ctrl_WB__MEM, excpt__MEM,
                    write_reg__MEM, write_data__MEM,
                    entryLo__MEM, entryHi__MEM, index__MEM, badVaddr__MEM);
                    
    // Outputs
    output [4:0]    ctrl_WB__WB;
    output [4:0]    write_reg__WB;
    output [31:0]   write_data__WB;
    output [11:0]   excpt__WB;
    output [31:0]   pc__WB, epc__WB;
    output [31:0]   entryLo__WB, entryHi__WB, index__WB, badVaddr__WB;

    // Inputs
    input           clk, rst_b, en, clr_excpt, clr_all;
    input [4:0]     ctrl_WB__MEM;
    input [4:0]     write_reg__MEM;
    input [31:0]    write_data__MEM;
    input [11:0]    excpt__MEM;
    input [31:0]    pc__MEM, epc__MEM;
    input [31:0]    entryLo__MEM, entryHi__MEM, index__MEM, badVaddr__MEM;
    
    // Instantiate the registers for the individual signals
    registerWithClear #(5, 0)   ctrl_WB(       .q(ctrl_WB__WB),    .d(ctrl_WB__MEM),    .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(5, 0)   write_reg(     .q(write_reg__WB),  .d(write_reg__MEM),  .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  write_data(    .q(write_data__WB), .d(write_data__MEM), .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  entryLo(       .q(entryLo__WB),    .d(entryLo__MEM),    .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));   
    registerWithClear #(32, 0)  entryHi(       .q(entryHi__WB),    .d(entryHi__MEM),    .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));   
    registerWithClear #(32, 0)  badVaddr(      .q(badVaddr__WB),   .d(badVaddr__MEM),   .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
    registerWithClear #(32, 0)  index(         .q(index__WB),      .d(index__MEM),      .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all || clr_excpt));
    registerWithClear #(32, 0)  pc(            .q(pc__WB),         .d(pc__MEM),         .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
    registerWithClear #(32, 0)  epc(           .q(epc__WB),        .d(epc__MEM),        .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));    
    registerWithClear #(12, 0)  excpt__MEM_WB( .q(excpt__WB),      .d(excpt__MEM),      .clk(clk), .enable(en), .rst_b(rst_b), .clear(clr_all));
endmodule


// Figure out when to stall
module stall_logic( // Outputs
                    stall,
                    // Inputs
                    dcd_rs, dcd_rt, write_reg__EX, reads_rs, reads_rt, ctrl_EX, alu__mul_stall__EX);
    // Outputs
    output reg stall;

    // Inputs
    input [4:0]     dcd_rs, dcd_rt, write_reg__EX;
    input           reads_rs, reads_rt;
    input [14:0]    ctrl_EX;
    input           alu__mul_stall__EX;
    
    // Wires
    wire [4:0] alu__sel;
    wire [1:0] mem_access_size;
    wire       is_store;
    wire [1:0] jump_sel;
    wire [2:0] branch_type;
    wire [1:0] write_data_sel_EX;

    // Decode the ctrl_EX wire
    decode_ctrl_EX decode_EX(   // Outputs
                                .alu__sel(alu__sel),
                                .mem_access_size(mem_access_size),
                                .is_store(is_store),
                                .jump_sel(jump_sel),
                                .branch_type(branch_type),
                                .write_data_sel_EX(write_data_sel_EX),
                                // Inputs
                                .ctrl_EX(ctrl_EX));

    always @(*) begin
        
        // We normally don't stall
        stall = 0 ;
    
        // If our instruction is move from hi or low, then we have to stall
        // once as the coprocessor is fetching the register
        if ( (alu__sel == `ALU_MFHI || alu__sel == `ALU_MFLO) &&
             ( ((dcd_rs != 0) && (dcd_rs == write_reg__EX) && reads_rs) ||
               ((dcd_rt != 0) && (dcd_rt == write_reg__EX) && reads_rt) ) ) begin
            stall = 1;
        end

        // Only other time we stall is if we have a load in the exectuion 
        // stage, and the current instruction is accessing the register
        // to be loaded
        if ( (is_store == 0 && mem_access_size != `SIZE_NONE) && 
             ( ((dcd_rs != 0) && (dcd_rs == write_reg__EX) && reads_rs) ||
               ((dcd_rt != 0) && (dcd_rt == write_reg__EX) && reads_rt) ) ) begin
            stall = 1;
        end



        // If multiplier is stalling, stall
        if ( alu__mul_stall__EX ) begin
            stall = 1;
        end
    end
endmodule


// Figure out when to forward
module forward_logic(   // Outputs
                        forwarded_rs_data__FWD, forwarded_rt_data__FWD, forwarded_coproc_data__FWD,
                        is_forward__rs_data__FWD, is_forward__rt_data__FWD, is_forward__coproc_data__FWD,
                        forwarded_entryLo__FWD, forwarded_entryHi__FWD, forwarded_index__FWD,
                        is_forward__entryLo__FWD, is_forward__entryHi__FWD, is_forward__index__FWD,
                        // Inputs
                        dcd_rs, dcd_rt, reads_rs, reads_rt,
                        coprocessor_read_sel__ID, ctrl_WB__EX, ctrl_WB__MEM, ctrl_WB__WB,
                        write_reg__EX, write_reg__MEM, write_reg__WB,
                        write_data_EX, write_data__MEM, write_data__WB);
    
    // Outputs
    // Data that will be forwarded to ID stage
    output reg [31:0]   forwarded_rs_data__FWD, forwarded_rt_data__FWD, forwarded_coproc_data__FWD;    
    
    // The select line to choose if the ALU will use the forwarded data    
    output reg          is_forward__rs_data__FWD, is_forward__rt_data__FWD, is_forward__coproc_data__FWD;  

    // Data forwarded for coproc
    output reg [31:0]   forwarded_entryLo__FWD, forwarded_entryHi__FWD, forwarded_index__FWD;       

    // Selects if coproc data is forwarded    
    output reg          is_forward__entryLo__FWD, is_forward__entryHi__FWD, is_forward__index__FWD;         
    
    // Inputs
    // The current registers the instruction is using. If using rs, set alu_op1. If using rt, set alu_op2.
    input [4:0] dcd_rs, dcd_rt;
    input       reads_rs, reads_rt;
    input [4:0] ctrl_WB__EX, ctrl_WB__MEM, ctrl_WB__WB;
    input [2:0] coprocessor_read_sel__ID;
    
    // Tells us what registers the stages are writing to
    input [4:0]  write_reg__EX, write_reg__MEM, write_reg__WB;

    // Tells us what data the stages contain
    input [31:0] write_data_EX, write_data__MEM, write_data__WB;    

    // Tells us if any of the stages are writing to the register file
    wire        rd_we__EX, rd_we__MEM, rd_we__WB;                   
    wire [2:0]  coprocessor_write_sel__EX, coprocessor_write_sel__MEM, coprocessor_write_sel__WB;
    
    // Decode Ctrl_WB signal. For ID stage, only care if reading. Other stages we care if we are writing or not
    decode_ctrl_WB decode_WB__EX(   // Outputs
                                    .rd_we(rd_we__EX),
                                    .coprocessor_write_sel(coprocessor_write_sel__EX),
                                    .tlb_we(), // Not needed here
                                    // Inputs
                                    .ctrl_WB(ctrl_WB__EX));

    decode_ctrl_WB decode_WB__MEM(  // Outputs
                                    .rd_we(rd_we__MEM),
                                    .coprocessor_write_sel(coprocessor_write_sel__MEM),
                                    .tlb_we(), // Not needed here
                                    // Inputs
                                    .ctrl_WB(ctrl_WB__MEM));

    decode_ctrl_WB decode_WB__WB(   // Outputs
                                    .rd_we(rd_we__WB),
                                    .coprocessor_write_sel(coprocessor_write_sel__WB),
                                    .tlb_we(), // Not needed here
                                    // Inputs
                                    .ctrl_WB(ctrl_WB__WB));

    always @ (*) begin

        // Assume no forwarding
        is_forward__rs_data__FWD = 0;
        is_forward__rt_data__FWD = 0;
        is_forward__coproc_data__FWD = 0;
        is_forward__entryLo__FWD = 0;
        is_forward__entryHi__FWD = 0;
        is_forward__index__FWD = 0;

        // If we're not forwarding, data output doesn't matter
        forwarded_rs_data__FWD = 32'bx;
        forwarded_rt_data__FWD = 32'bx;
        forwarded_coproc_data__FWD = 32'bx;
        forwarded_entryLo__FWD = 32'bx;
        forwarded_entryHi__FWD = 32'bx;
        forwarded_index__FWD = 32'bx;

        // One of the stages is writing back, therefore see if they are writing to a register that we are using.
        // Order matters! We want to use the most recent first, so check WB, then MEM, then EX.

        // Forwarding RS
        // Check Execute Stage
        if( rd_we__EX && (dcd_rs != 0) && reads_rs && (dcd_rs == write_reg__EX) ) begin
            forwarded_rs_data__FWD = write_data_EX;
            is_forward__rs_data__FWD = 1;
        end 

        // Check Memory Stage
        else if( rd_we__MEM  && (dcd_rs != 0) && reads_rs && (dcd_rs == write_reg__MEM) ) begin
            forwarded_rs_data__FWD = write_data__MEM;
            is_forward__rs_data__FWD = 1;
        end
    
        // Check Write Back Stage
        else if( rd_we__WB  && (dcd_rs != 0) && reads_rs && (dcd_rs == write_reg__WB) ) begin
            forwarded_rs_data__FWD = write_data__WB;
            is_forward__rs_data__FWD = 1;
        end
    
        // Forwarding RT
        // Check Execute Stage
        if( rd_we__EX && (dcd_rt != 0) && reads_rt && (dcd_rt == write_reg__EX) ) begin
            forwarded_rt_data__FWD = write_data_EX;
            is_forward__rt_data__FWD = 1;
        end

        // Check Memory Stage           
        else if( rd_we__MEM && (dcd_rt != 0) && reads_rt && (dcd_rt == write_reg__MEM) ) begin
            forwarded_rt_data__FWD = write_data__MEM;
            is_forward__rt_data__FWD = 1;
        end

        // Check Write Back Stage
        else if( rd_we__WB && (dcd_rt != 0) && reads_rt && (dcd_rt == write_reg__WB) ) begin
            forwarded_rt_data__FWD = write_data__WB;
            is_forward__rt_data__FWD = 1;
        end

        // Forwarding Coproc
        // Check Execute Stage
        if( (coprocessor_read_sel__ID != `COPROC_READ_NONE) && (coprocessor_write_sel__EX == coprocessor_read_sel__ID) ) begin
            forwarded_coproc_data__FWD = write_data_EX;
            is_forward__coproc_data__FWD = 1;
        end

        // Check Memory Stage           
        else if( (coprocessor_read_sel__ID != `COPROC_READ_NONE) && (coprocessor_write_sel__MEM == coprocessor_read_sel__ID) ) begin
            forwarded_coproc_data__FWD = write_data__MEM;
            is_forward__coproc_data__FWD = 1;
        end

        // Check Write Back Stage
        else if( (coprocessor_read_sel__ID != `COPROC_READ_NONE) && (coprocessor_write_sel__WB == coprocessor_read_sel__ID) ) begin
            forwarded_coproc_data__FWD = write_data__WB;
            is_forward__coproc_data__FWD = 1;
        end
    
        // For EntryLo, EntryHi, and Index...just always forward if we are writing to it in any stage
        // Entry Lo
        // Check Execute Stage
        if(coprocessor_write_sel__EX == `ENTRYLO_WRITE) begin
            forwarded_entryLo__FWD = write_data_EX;
            is_forward__entryLo__FWD = 1;
        end

        // Check Memory Stage           
        else if(coprocessor_write_sel__MEM == `ENTRYLO_WRITE) begin
            forwarded_entryLo__FWD = write_data__MEM;
            is_forward__entryLo__FWD = 1;
        end

        // Check Write Back Stage
        else if(coprocessor_write_sel__WB == `ENTRYLO_WRITE) begin
            forwarded_entryLo__FWD = write_data__WB;
            is_forward__entryLo__FWD = 1;
        end
   
        // Entry Hi
        // Check Execute Stage
        if(coprocessor_write_sel__EX == `ENTRYHI_WRITE) begin
            forwarded_entryHi__FWD = write_data_EX;
            is_forward__entryHi__FWD = 1;
        end

        // Check Memory Stage           
        else if(coprocessor_write_sel__MEM == `ENTRYHI_WRITE) begin
            forwarded_entryHi__FWD = write_data__MEM;
            is_forward__entryHi__FWD = 1;
        end

        // Check Write Back Stage
        else if(coprocessor_write_sel__WB == `ENTRYHI_WRITE) begin
            forwarded_entryHi__FWD = write_data__WB;
            is_forward__entryHi__FWD = 1;
        end

        // Index
        // Check Execute Stage
        if(coprocessor_write_sel__EX == `INDEX_WRITE) begin
            forwarded_index__FWD = write_data_EX;
            is_forward__index__FWD = 1;
        end

        // Check Memory Stage           
        else if(coprocessor_write_sel__MEM == `INDEX_WRITE) begin
            forwarded_index__FWD = write_data__MEM;
            is_forward__index__FWD = 1;
        end

        // Check Write Back Stage
        else if(coprocessor_write_sel__WB == `INDEX_WRITE) begin
            forwarded_index__FWD = write_data__WB;
            is_forward__index__FWD = 1;
        end
  
    end
endmodule


//// register: A register which may be reset to an arbirary value
////
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module register(q, d, clk, enable, rst_b);

    parameter
        width = 32,
        reset_value = 0;
    
    output [(width-1):0] q;
    reg    [(width-1):0] q;
    input  [(width-1):0] d;
    input  clk, enable, rst_b;
    
    always @(posedge clk or negedge rst_b)
        if (~rst_b)
            q <= reset_value;
        else if (enable)
            q <= d;
endmodule // register


//// register with syncronous clear:
//// A register which may be reset to an arbirary value
//// and can be cleared syncrounsly
//// q      (output) - Current value of register
//// d      (input)  - Next value of register
//// clk    (input)  - Clock (positive edge-sensitive)
//// enable (input)  - Load new value?
//// reset  (input)  - System reset
////
module registerWithClear(q, d, clk, enable, rst_b, clear);

    parameter
        width = 32,
        reset_value = 0;
    
    output [(width-1):0] q;
    reg    [(width-1):0] q;
    input  [(width-1):0] d;
    input  clk, enable, rst_b, clear;
    
    always @(posedge clk or negedge rst_b)
        if (~rst_b || clear)
            q <= reset_value;
        else if (enable)
            q <= d;
endmodule // register


// Adder - Adds two inputs
module adder(sum, in1, in2);

    parameter
        width = 32;

    output [(width-1):0] sum;
    input  [(width-1):0] in1;
    input  [(width-1):0] in2;

    assign sum = in1 + in2;
endmodule

// Local Variables:
// verilog-library-directories:("." "../447rtl")
// End:
