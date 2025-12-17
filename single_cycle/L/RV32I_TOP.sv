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
    logic [ 2:0] s_type;
    logic        mem_write_en;
    logic [31:0] mem_addr;
    logic [31:0] mem_wdata;
    logic [ 2:0] L_type_data;
    logic [31:0] mem_rdata;

    RV32I_TOP U_RV32I (
        .clk         (clk),
        .reset       (reset),
        .Instr_Addr  (Inst_addr),
        .Instr_rdata (Inst_rdata),
        .S_type_data (s_type),
        .mem_write_en(mem_write_en),
        .alu_result  (mem_addr),
        .mem_wdata   (mem_wdata),
        .L_type_data (L_type_data),
        .mem_rdata   (mem_rdata)
    );

    IMEM U_ROM (
        .Instr_Addr (Inst_addr),
        .Instr_rdata(Inst_rdata)
    );

    dMEM U_dMEM (
        .clk         (clk),
        .mem_write_en(mem_write_en),
        .s_type      (s_type),
        .mem_addr    (mem_addr),
        .mem_wdata   (mem_wdata),
        .mem_rdata   (mem_rdata),
        .l_type      (L_type_data)
    );

endmodule



module RV32I_TOP (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] Instr_rdata,
    output logic [31:0] Instr_Addr,
    output logic [31:0] alu_result,
    output logic [ 2:0] S_type_data,
    output logic        mem_write_en,
    output logic [31:0] mem_wdata,  
    output logic [ 2:0] L_type_data,
    input  logic [31:0] mem_rdata
);

    logic [3:0] alu_en;
    logic       reg_write_en;
    logic       imm_en;
    logic [1:0] rd_mux_en;

    datapath U_DP (
        .Instr_rdata      (Instr_rdata),
        .clk              (clk),
        .reset            (reset),
        .alu_control_en   (alu_en),
        .register_write_en(reg_write_en),
        .Instr_Addr       (Instr_Addr),
        .alu_result       (alu_result),
        .imm_en           (imm_en),
        .mem_wdata        (mem_wdata),
        .rd_mux_en        (rd_mux_en),
        .mem_rdata        (mem_rdata)
    );

    control U_CTRL (
        .Instr_rdata      (Instr_rdata),
        .register_write_en(reg_write_en),
        .alu_control_en   (alu_en),
        .mem_write_en     (mem_write_en),
        .imm_en           (imm_en),
        .S_type_data      (S_type_data),
        .L_type_data      (L_type_data),
        .rd_mux_en        (rd_mux_en)
    );
endmodule
