`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/18 15:28:42
// Design Name: 
// Module Name: RV32I_TOP
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
module CPU_TOP (
    input logic clk,
    input logic reset
);
    logic [31:0] Inst_addr;
    logic [31:0] Inst_rdata;

    RV32I_TOP U_RV32I (
        .clk(clk),
        .reset(reset),
        .Instr_Addr(Inst_addr),
        .Instr_rdata(Inst_rdata)
    );

    IMEM U_ROM (
        .Instr_Addr (Inst_addr),
        .Instr_rdata(Inst_rdata)
    );


endmodule



module RV32I_TOP (

    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] Instr_rdata,
    output logic [31:0] Instr_Addr,
    output logic [31:0] alu_result
);

    logic [3:0] alu_en;
    logic reg_write_en;
    logic imm_en;

    datapath U_DP (
        .Instr_rdata(Instr_rdata),
        .clk(clk),
        .reset(reset),
        .alu_control_en(alu_en),
        .register_write_en(reg_write_en),
        .Instr_Addr(Instr_Addr),
        .alu_result(alu_result),
        .imm_en(imm_en)
    );

    control U_CTRL (
        .Instr_rdata(Instr_rdata),
        .register_write_en(reg_write_en),
        .alu_control_en(alu_en),
        .imm_en(imm_en)
    );

endmodule
