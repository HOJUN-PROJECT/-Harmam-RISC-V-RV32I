`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/18 14:18:56
// Design Name: 
// Module Name: IMEM
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


module IMEM (
    input  logic [31:0] Instr_Addr,
    output logic [31:0] Instr_rdata
);

    logic [31:0] mem[0:15];

    initial begin
        mem[0] = 32'h003100B3; // ADD x1, x2, x3
        mem[1] = 32'h40628233; // SUB x4, x5, x6
        mem[2] = 32'h009447B3; // AND x7, x8, x9
        mem[3] = 32'h00C5A533; // OR  x10,x11,x12
        mem[4] = 32'h00F6C6B3; // XOR x13,x14,x15
        mem[5] = 32'h003110B3; // SLL x1, x2, x3
        mem[6] = 32'h003150B3; // SRL x1, x2, x3
        mem[7] = 32'h403150B3; // SRA x1, x2, x3
        mem[8] = 32'h003120B3; // SLT x1, x2, x3
        mem[9] = 32'h003130B3; // SLTU x1, x2, x3
    end

    assign Instr_rdata = mem[Instr_Addr[5:2]]; //byte단위 pc하위 2비트는 0 <= 명령어 성택에 필요없음

endmodule
