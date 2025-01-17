`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly
// Engineer: Asha Smith
// 
// Create Date: 09/24/2018 08:37:20 AM
// Design Name: 
// Module Name: simTemplate
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
module simTemplate;

    // Signals
    logic CLK = 0;
    logic BTNL, BTNC, PS2Clk, PS2Data, VGA_HS, VGA_VS, Tx;
    logic [15:0] SWITCHES, LEDS;
    logic [7:0] CATHODES, VGA_RGB;
    logic [3:0] ANODES;

    // Program Counter and Multi Address
    logic [31:0] pc;          // Program Counter signal
    logic [31:0] multi_addr;  // Address of Multi label

    // Instantiate the DUT (Design Under Test)
    OTTER_Wrapper DUT (
        .* // Connect all signals by name
    );

    // Clock generation
    initial forever #10 CLK = !CLK;

    // Stimulus
    initial begin
        BTNC = 1;
        #600;
        BTNC = 0;
        SWITCHES = 15'd0;

        //$finish; // Uncomment if simulation should stop after this
    end

    // Define Multi address
    assign multi_addr = 32'h40; // Example address for Multi

    // Monitor Program Counter for Multi subroutine
    always @(pc) begin
        if (pc == multi_addr) begin
            $display("%t: Multi subroutine called. PC = %h", $time, pc);
        end
    end

    // Uncomment this block for additional debugging
    /*
    initial begin
        if (ld_use_hazard)
            $display("%t -------> Stall ", $time);
        if (branch_taken)
            $display("%t -------> Branch taken", $time);
    end
    */
endmodule
