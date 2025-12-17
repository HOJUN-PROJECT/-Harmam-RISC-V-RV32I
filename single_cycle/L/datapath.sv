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
    input  logic        imm_en,
    input  logic        register_write_en,
    output logic [31:0] Instr_Addr,
    output logic [31:0] alu_result,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    input  logic [ 1:0] rd_mux_en
);


    logic [31:0]
        regster_data1,
        regster_data2,
        pc_out,
        pc_in,
        imm_addr,
        mux_rd_data,
        mux_regster_data2;

    assign Instr_Addr = pc_out;
    assign mem_wdata  = regster_data2;

    program_counter U_PC (
        .clk  (clk),
        .reset(reset),
        .q_in (pc_in),
        .pc   (pc_out)
    );

    regfile U_REGFILE (
        .clk(clk),
        // .reset(reset),
        .RA1(Instr_rdata[19:15]),
        .RA2(Instr_rdata[24:20]),
        .WA (Instr_rdata[11:7]),
        .WD (mux_rd_data),
        .WE (register_write_en),
        .RD1(regster_data1),
        .RD2(regster_data2)
    );

    ALU U_ALU (
        .RS1           (regster_data1),
        .RS2           (mux_regster_data2),
        .alu_control_en(alu_control_en),
        .alu_result    (alu_result)
    );

    adder U_ADD (
        .a(pc_out),
        .b(32'd4),
        .y(pc_in)
    );

    imm_extend U_IMM_EXTEND (
        .instr_opcode(Instr_rdata),
        .imm_addr    (imm_addr)
    );

    MUX_2x1 U_IMM_MUX (
        .a (imm_addr),
        .b (regster_data2),
        .en(imm_en),
        .y (mux_regster_data2)
    );

    MUX_2x1 U_RD_MUX (
        .a (mem_rdata),
        .b (alu_result),
        .en(rd_mux_en),
        .y (mux_rd_data)
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
    input  logic [ 4:0] RA1,
    input  logic [ 4:0] RA2,
    input  logic [ 4:0] WA,
    input  logic [31:0] WD,
    input  logic        WE,
    output logic [31:0] RD1,
    output logic [31:0] RD2
);

    logic [31:0] reg_file[0:31];
    integer j;

    initial begin
        for (j = 0; j < 32; j++) begin
            reg_file[j] = j;
        end
    end

    always_ff @(posedge clk) begin
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
                alu_result = RS1 << RS2[4:0];
            end
            4'b0010: begin  //
                alu_result = ($signed(RS1) < $signed(RS2)) ? 1 : 0;
            end
            4'b0011: begin
                alu_result = ($unsigned(RS1) < $unsigned(RS2)) ? 1 : 0;
            end
            4'b0100: begin
                alu_result = RS1 ^ RS2;
            end
            4'b0101: begin
                alu_result = RS1 >> RS2[4:0];
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
                alu_result = $signed(RS1) >>> (RS2[4:0]);
            end
        endcase
    end
endmodule

module imm_extend (
    input  logic [31:0] instr_opcode,
    output logic [31:0] imm_addr
);

    logic [2:0] funt_3 = instr_opcode[14:12];

    always_comb begin
        imm_addr = 32'b0;
        if (instr_opcode[6:0] == 7'b0010011) begin
            case (funt_3)
                3'b001, 3'b101:  // SLLI, SRLI, SRAI
                imm_addr = {27'b0, instr_opcode[24:20]};
                3'b011: imm_addr = {20'b0, instr_opcode[31:20]};
                default:
                imm_addr = {{20{instr_opcode[31]}}, instr_opcode[31:20]};
            endcase
        end
        if (instr_opcode[6:0] == 7'b0100011) begin
            imm_addr = {
                {20{instr_opcode[31]}}, instr_opcode[31:25], instr_opcode[11:7]
            };
        end
        if (instr_opcode[6:0] == 7'b0000011) begin
            imm_addr = {{20{instr_opcode[31]}}, instr_opcode[31:20]};
        end
    end

endmodule


module adder (
    input  logic [31:0] a,
    input  logic [31:0] b,
    output logic [31:0] y
);

    assign y = a + b;

endmodule


module MUX_2x1 (
    input logic [31:0] a,
    input logic [31:0] b,
    input logic en,
    output logic [31:0] y
);

    assign y = en ? a : b;

endmodule

