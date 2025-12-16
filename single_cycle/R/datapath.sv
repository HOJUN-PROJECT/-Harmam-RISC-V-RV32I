`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2025/11/18 14:17:10
// Design Name: 
// Module Name: datapath
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


module datapath (
    input  logic [31:0] Instr_rdata,
    input  logic        clk,
    input  logic        reset,
    input  logic [ 3:0] alu_control_en,
    input  logic        register_write_en,
    output logic [31:0] Instr_Addr
);

    logic [31:0] regster_data1, regster_data2, alu_result, pc_out, pc_in;

    assign Instr_Addr = pc_out;

    program_counter U_PC (
        .clk(clk),
        .reset(reset),
        .q_in(pc_in),
        .pc(pc_out)
    );

    regfile U_REGFILE (
        .clk(clk),
        .reset(reset),
        .RA1(Instr_rdata[19:15]),
        .RA2(Instr_rdata[24:20]),
        .WA(Instr_rdata[11:7]),
        .WD(alu_result),
        .WE(register_write_en),
        .RD1(regster_data1),
        .RD2(regster_data2)
    );

    ALU U_ALU (
        .RS1(regster_data1),
        .RS2(regster_data2),
        .alu_control_en(alu_control_en),
        .alu_result(alu_result)
    );

    adder U_ADD (
        .a(pc_out),
        .b(32'd4),
        .y(pc_in)
    );

endmodule

module program_counter (
    input  logic        clk,
    input  logic        reset,
    input  logic [31:0] q_in,
    output logic [31:0] pc
);

    always @(posedge clk, posedge reset) begin
        if (reset) begin
            pc <= 32'd0;
        end else begin
            pc <= q_in;
        end
    end

endmodule

module regfile (
    input  logic        clk,
    input  logic        reset,
    input  logic [ 4:0] RA1,
    input  logic [ 4:0] RA2,
    input  logic [ 4:0] WA,
    input  logic [31:0] WD,
    input  logic        WE,
    output logic [31:0] RD1,
    output logic [31:0] RD2
);

    logic [31:0] reg_file[0:31];
    integer i;
    integer j;

    initial begin
        for (j = 0; j < 32; j++) begin
            reg_file[j] = j;
        end
    end

    always_ff @(posedge clk or posedge reset) begin
        if (WE) begin
            reg_file[WA] <= WD;
        end
    end


    assign RD1 = reg_file[RA1];
    assign RD2 = reg_file[RA2];

endmodule

module ALU (
    input  logic [31:0] RS1,
    input  logic [31:0] RS2,
    input  logic [ 3:0] alu_control_en,
    output logic [31:0] alu_result
);

    always @(*) begin
        alu_result = 32'd0;
        case (alu_control_en)
            4'b0000: begin  //add
                alu_result = RS1 + RS2;
            end
            4'b0001: begin  //sll
                alu_result = RS1 << RS2;
            end
            4'b0010: begin  //
                alu_result = (RS1 < RS2) ? 1 : 0;
            end
            4'b0011: begin
                alu_result = ($unsigned(RS1) < $unsigned(RS2)) ? 1 : 0;
            end
            4'b0100: begin
                alu_result = RS1 ^ RS2;
            end
            4'b0101: begin
                alu_result = RS1 >> RS2;
            end
            4'b0110: begin
                alu_result = RS1 | RS2;
            end
            4'b0111: begin
                alu_result = RS1 & RS2;
            end
            4'b1000: begin
                alu_result = RS1 - RS2;
            end
            4'b1101: begin
                alu_result = $signed(RS1) >>> $signed(RS2);
            end
        endcase
    end
endmodule

module adder (
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] y
);

    assign y = a + b;

endmodule
