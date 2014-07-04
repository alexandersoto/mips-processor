// This contains the logic needed for our TLB

// Module that the core will use to interface with the TLB
module tlb_interface(   // Outputs
                        output reg [31:0] inst_physical_address, mem_physical_address,
                        output reg        inst_translation_miss, mem_translation_miss, mem_is_writable,
                        // Input
                        input [31:0]      inst_virtual_address, mem_virtual_address,
                        input [4:0]       writeIndex,
                        input [31:0]      writeTag, writeData,
                        input             writeEnable, clk, rst_b);

    // These contian the PFN's 
    wire [31:0] inst_tlb_data, mem_tlb_data;
    wire        inst_hit, mem_hit;

    // Crete the TLB itself
    mips_tlb    TLB_Memory( // Outputs  
                            .readData_1(inst_tlb_data),
                            .readData_2(mem_tlb_data), 
                            .hit_1(inst_hit),
                            .hit_2(mem_hit),
                            // Inputs           
                            .readTag_1(inst_virtual_address),
                            .readTag_2(mem_virtual_address),
                            .writeIndex(writeIndex),    
                            .writeTag(writeTag),
                            .writeData(writeData), 
                            .writeEnable(writeEnable),
                            .clk(clk), 
                            .rst_b(rst_b)); 

    // Logic to differentiate between unmapped and mapped address locations
    always @ (*) begin

        // Assume we are not accessing the TLB, therefore not missing
        inst_translation_miss = 0;
        mem_translation_miss  = 0;
        mem_is_writable       = 1;

        // If instuction memory is unmapped, just pass it through after
        // removing the offsets
        if( (inst_virtual_address >= `KSEG0_START) &&
            (inst_virtual_address <= `KSEG0_END)) begin
            inst_physical_address = (inst_virtual_address - `KSEG0_START);

        end
        else if( (inst_virtual_address >= `KSEG1_START) &&
                 (inst_virtual_address <= `KSEG1_END)) begin
            inst_physical_address = (inst_virtual_address - `KSEG1_START);
        end
        else begin

            // The physical address is the PFN plus the offset from the virtual
            // address (last 12 bits)
            inst_physical_address = { inst_tlb_data[31:12], inst_virtual_address[11:0] };

            // If we miss, while we are actually accessing the TLB assert that we missed
            inst_translation_miss = ~inst_hit;
        end
    
        // If mem memory is unmapped, just pass it through
        if( (mem_virtual_address >= `KSEG0_START) &&
            (mem_virtual_address <= `KSEG0_END)) begin
            mem_physical_address = (mem_virtual_address - `KSEG0_START);

        end
        else if( (mem_virtual_address >= `KSEG1_START) &&
                 (mem_virtual_address <= `KSEG1_END)) begin
            mem_physical_address = (mem_virtual_address - `KSEG1_START);
        end
        else begin

            // The physical address is the PFN plus the offset from the virtual
            // address (last 12 bits)
            mem_physical_address = { mem_tlb_data[31:12], mem_virtual_address[11:0] };

            // If we miss, while we are actually accessing the TLB assert that we missed
            mem_translation_miss  = ~mem_hit;

            // It is writable only if the D bit is asserted, and we've
            // accessed the TLB
            mem_is_writable = mem_tlb_data[10];         
        end
    end
endmodule


// This is the actual TLB memory
module mips_tlb( // Outputs
                output reg [31:0]   readData_1, readData_2, 
                output reg          hit_1, hit_2,
                // Inputs           
                input [31:0]        readTag_1, readTag_2,
                input [4:0]         writeIndex, 
                input [31:0]        writeTag, writeData, 
                input               writeEnable, clk, rst_b); 

    // Define our two data elements for the cache
    reg [31:0] tag [0:31]; 
    reg [31:0] data [0:31];
    integer i; 
    integer j; 
    
    // This resets both the tag and data if rst_b is 0
    // Otherwise, if we have enable asserted then write both
    // the tag and data
    always @ (posedge clk) begin 
        if (rst_b == 1'b0) begin 
            for(i = 0; i < 32; i = i+1) begin
                tag[i]  <= 32'b0; 
                data[i] <= 32'b0;
            end
        end 
        else if (writeEnable) begin
            tag[writeIndex]  <= writeTag; 
            data[writeIndex] <= writeData; 
        end 
    end 

    // Reading the TLB, we will return the data but it is only valid
    // if the tag matches and we assert hit
    // Not sure if to be sensitive to selected inputs or *. If selected inputs
    // are put in list then synth tool gives warning.
    // If star is put, then simulator gives warning.
    //  always @(*) begin
    always @(readTag_1, readTag_2) begin
    
        // We assume no hit
        hit_1 = 1'b0; 
        hit_2 = 1'b0;

        // Don't care if hit is zero
        readData_1 = 32'hxxxx_xxxx;
        readData_2 = 32'hxxxx_xxxx;

        // If a tag matches what we are looking for AND its valid bit is set
        // then set hit and output the proper data
        for(j = 0; j < 32; j = j+1) begin

            // Only care if the first 20 bits match, as the other info is useless in this lab
            if ( (tag[j][31:12] == readTag_1[31:12]) && data[j][9] ) begin
                readData_1 = data[j];
                hit_1 = 1'b1; 
            end
            if ( (tag[j][31:12] == readTag_2[31:12]) && data[j][9]) begin 
                readData_2 = data[j];
                hit_2 = 1'b1; 
            end     
        end
    end 
endmodule


// Simple counter that counts to 4
module fourBitCounter( // Outputs
                       output [3:0] count,
                       // Inputs
                       input        clk, rst_b);

    reg [3:0] CS, NS;
    assign count = CS;

    // Unless we hit the max, our current state is just an increment
    always @(*) begin
        case(CS)
            4'b1111: NS = 4'b0000;
            default: NS = CS + 1;
        endcase
    end

    // On reset go to state 0, otherwise current state gets next state
    always @(posedge clk) begin
        if (rst_b == 1'b0) begin
            CS <= 4'b0000;
        end
        else begin
            CS <= NS;
        end
    end
endmodule
