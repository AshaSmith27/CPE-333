`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes, Asha Smith, Jack Sevigny
// 
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR);   
                
                        
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, next_pc, jalr_pc, branch_pc, jump_pc, int_pc,A,B,
        I_immed,S_immed,U_immed,aluBin,aluAin,aluResult,rfIn,csr_reg, mem_data;
    
    wire [31:0] IR;
    wire memRead1,memRead2;
    
    wire pcWrite,regWrite,memWrite, op1_sel,mem_op,IorD,pcWriteCond,memRead;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    logic [1:0] pc_sel;
    wire [3:0]alu_fun;
    wire opA_sel;
    
    logic br_lt, br_eq, br_ltu;
            
//==== Instruction Fetch ===========================================
//not done

     logic [31:0] if_de_pc;
     
     //Create RISC_V PC
     OTTER_PC PC (.CLK(CLK), .RST(pc_rst), .PC_WRITE(pc_write), .PC_SOURCE(pc_source),
        .JALR(jalr), .JAL(jal), .BRANCH(branch), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc_out), .PC_OUT_INC(pc_out_inc));
        
     //Creates RISC-V MEM
     OTTER_MEMORY Memory (.MEM_CLK(CLK), .MEM_RDEN1(mem_rden1), .MEM_RDEN2(mem_rden2), 
        .MEM_WE2(mem_we2), .MEM_ADDR1(pc_out), .MEM_ADDR2(IOBUS_ADDR), .MEM_DIN2(IOBUS_OUT), .MEM_SIZE(size),
         .MEM_SIGN(sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(ir), .MEM_DOUT2(dout2));   
    
     always_ff @(posedge CLK) begin //Pipeline register 
                if_de_pc <= pc;
     end
     
     assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
     assign memRead1 = 1'b1; 	//Fetch new instruction every cycle
     
     
//==== Instruction Decode ===========================================
    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

    instr_t de_ex_inst, de_inst; //
    
    OTTER_DCDR CU_DCDR (.IR_30(ir30), .IR_OPCODE(opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), .BR_LT(br_lt),
     .BR_LTU(br_ltu), .ALU_FUN(alu_fun), .ALU_SRCA(alu_src_a), .ALU_SRCB(alu_src_b), .PC_SOURCE(pc_source),
      .RF_WR_SEL(rf_wr_sel));
      
    ImmediateGenerator OTTER_IMGEN(.IR(imgen_ir), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
        .B_TYPE(Btype), .J_TYPE(Jtype));
    
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(reg_wr), .ADR1(reg_adr1), .ADR2(reg_adr2), .WA(reg_wa), 
        .WD(wd), .RS1(rs1), .RS2(IOBUS_OUT));
    
    OTTER_ALU_MUXA TwoMux (.ALU_SRC_A(alu_src_a), .RS1(rs1), .U_TYPE(Utype), .SRC_A(srcA));
    
    OTTER_ALU_MUXB FourMux (.SEL(alu_src_b), .ZERO(IOBUS_OUT), .ONE(Itype), .TWO(Stype), .THREE(pc_out), .OUT(srcB));
    
    opcode_t OPCODE;
    assign OPCODE_t = opcode_t'(opcode);
    
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.opcode=OPCODE;
   
    assign de_inst.rs1_used=    de_inst.rs1 != 0 //Example logic to see if rs1 is used for an instruction
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;

     
     always_ff @(posedge CLK) begin //Pipeline register (IF -> EX)
        de_ex_inst <= de_inst;
     end 
	
	
//==== Execute ======================================================
     logic [31:0] ex_mem_rs2;
     logic ex_mem_aluRes = 0;
     instr_t ex_mem_inst;
     logic [31:0] opA_forwarded;
     logic [31:0] opB_forwarded;
     
     instr_t ex_mem_inst, de_ex_inst; //
     
    // Creates a RISC-V ALU
    OTTER_ALU ALU (de_ex_inst.alu_fun, de_ex_opA, de_ex_opB, aluResult); // the ALU
    
    OTTER_BCG BCG (.RS1(rs1), .RS2(IOBUS_OUT), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
   
    OTTER_BAG BAG (.RS1(rs1), .I_TYPE(Itype), .J_TYPE(Jtype), .B_TYPE(Btype), .FROM_PC(pc_out),
         .JAL(jal), .JALR(jalr), .BRANCH(branch));
         
     always_ff @(posedge CLK) begin //Pipeline register 
        ex_mem_inst <= de_ex_inst;
     end 

//==== Memory ======================================================
     
    instr_t mem_wb_inst, ex_mem_inst;
     
    OTTER_REG_MUX FourMux1 (.SEL(rf_wr_sel), .ZERO(pc_out_inc), .ONE(32'b0), .TWO(dout2), .THREE(IOBUS_ADDR),
        .OUT(wd));
    
    OTTER_REG_FILE REG_FILE (.CLK(CLK), .EN(reg_wr), .ADR1(reg_adr1), .ADR2(reg_adr2), .WA(reg_wa), 
        .WD(wd), .RS1(rs1), .RS2(IOBUS_OUT));
        
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    //assign de_ex_inst.ir = 32'b0 //Access stuff
 
 
     always_ff @(posedge CLK) begin //Pipeline register 
        mem_wb_inst <= ex_mem_inst;
     end 
     
//==== Write Back ==================================================
     


 
 

       
            
endmodule
