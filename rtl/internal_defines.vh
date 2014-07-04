////
//// Internal signal constants
////

// ALU Control Signals
`define ALU_ADD             5'h00
`define ALU_SUB             5'h01
`define ALU_ADDU            5'h02
`define ALU_SUBU            5'h03
`define ALU_AND             5'h04
`define ALU_OR              5'h05
`define ALU_XOR             5'h06
`define ALU_SLL             5'h07
`define ALU_SRL             5'h08
`define ALU_SRA             5'h09
`define ALU_MULT            5'h0a
`define ALU_MULTU           5'h0b
`define ALU_DIV             5'h0c
`define ALU_DIVU            5'h0d
`define ALU_NOR             5'h0e
`define ALU_SLT             5'h0f
`define ALU_SLTU            5'h10
`define ALU_BRC             5'h11
`define ALU_LUI             5'h12
`define ALU_MFHI            5'h13
`define ALU_MFLO            5'h14
`define ALU_MTHI            5'h15
`define ALU_MTLO            5'h16
`define ALU_PASS_OP2        5'h17

// Memory Write Sizes
`define SIZE_NONE           2'h0
`define SIZE_BYTE           2'h1 
`define SIZE_HALFWORD       2'h2
`define SIZE_WORD           2'h3

// ALU OP2 Selects
`define ALU_OP2_REG         2'h0
`define ALU_OP2_IMM         2'h1

// ALU OP1 Selects
`define ALU_OP1_REG         2'h0
`define ALU_OP1_SHIFT       2'h1 

// Masks for Memory Sizes
`define MASK_BYTE_SE        3'h4
`define MASK_BYTE_ZE        3'h3
`define MASK_HALFWORD_SE    3'h2
`define MASK_HALFWORD_ZE    3'h1
`define MASK_WORD           3'h0

// Write Data Select Line in
`define SEL_FROM_MEM        2'h0
`define SEL_FROM_EX         2'h1
`define SEL_FROM_MUL        2'h2

// Forward Data select in EX
`define SEL_FROM_ALU        2'h0
`define SEL_FROM_COPROC     2'h1
`define SEL_FROM_PC4        2'h2    

// Branch Types
`define BRC_NONE            3'h0
`define BRC_GTZ             3'h1
`define BRC_GEZ             3'h2
`define BRC_LTZ             3'h3
`define BRC_LEZ             3'h4
`define BRC_EQ              3'h5
`define BRC_NEQ             3'h6

// Write Registers
`define WR_REG_RT           2'h0
`define WR_REG_RD           2'h1
`define WR_REG_31           2'h2

// Jump types
`define JUMP_NONE           2'h0
`define JUMP_IMM            2'h1
`define JUMP_REG            2'h2
`define JUMP_EPC            2'h3

// This is used to clear all but the alu_sel bits in the EX stage
`define CTRL_EX_MASK        15'b11111_00_0_00_000_00

// Coprocessor Instruction OP Codes
`define COPROC_READ_NONE    3'd0
`define EPC_READ            3'd1
`define STATUS_READ         3'd2
`define CAUSE_READ          3'd3
`define BADVADDR_READ       3'd4
`define ENTRYLO_READ        3'd5
`define ENTRYHI_READ        3'd6
`define INDEX_READ          3'd7

`define COPROC_WRITE_NONE   3'd0
`define EPC_WRITE           3'd1
`define STATUS_WRITE        3'd2
`define CAUSE_WRITE         3'd3
`define BADVADDR_WRITE      3'd4
`define ENTRYLO_WRITE       3'd5
`define ENTRYHI_WRITE       3'd6
`define INDEX_WRITE         3'd7

`define SET_NONE            2'h0
`define SET_KERNAL_MODE     2'h1
`define SET_USER_MODE       2'h2

`define KERNEL_START        32'hA000_2000
`define EXCEPTION_ADDRESS   32'hA000_2180
`define TLB_HANDLER_ADDRESS 32'hA000_2200

// Used to determine mapped / unmapped parts of memory
`define KSEG0_START         32'h8000_0000
`define KSEG0_END           32'h9FFF_FFFF
`define KSEG1_START         32'hA000_0000
`define KSEG1_END           32'hBFFF_FFFF
