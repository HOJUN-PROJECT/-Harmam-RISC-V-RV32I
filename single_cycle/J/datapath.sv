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
    input  logic [ 2:0] B_type_data,
    input  logic        register_write_en,
    output logic [31:0] Instr_Addr,
    output logic [31:0] alu_result,
    output logic [31:0] mem_wdata,
    input  logic [31:0] mem_rdata,
    input  logic [ 2:0] rd_mux_en,
    input  logic        jump_en
);

    logic [31:0]
        regster_data1,
        regster_data2,
        pc_out,
        pc_in,
        pc_plus4,
        imm_addr,
        mux_rd_data,
        mux_regster_data2,
        branch_target,
        mux_brach_pc_in,
        lui_result,
        alui_result,
        j_result_pc,
        j_result_rd;

    logic branch_en;
    logic [31:0] jump_pc_in;


    assign Instr_Addr = pc_out;
    assign mem_wdata  = regster_data2;
    assign lui_result = imm_addr;

    // PC + 4
    adder U_ADD (
        .a(pc_out),
        .b(32'd4),
        .y(pc_plus4)
    );

    assign branch_target = pc_out + imm_addr;
    assign alui_result   = pc_out + imm_addr;
    assign j_result_pc   = pc_out + imm_addr;
    assign j_result_rd   = pc_plus4;

    MUX_2x1 U_PC_IMM_MUX (
        .a (branch_target),  // branch taken
        .b (pc_plus4),       // normal
        .en(branch_en),
        .y (pc_in)
    );

    MUX_2x1 U_PC_JUMP_MUX (
        .a (j_result_pc),  // branch taken
        .b (pc_in),        // normal
        .en(jump_en),
        .y (jump_pc_in)
    );

    program_counter U_PC (
        .clk  (clk),
        .reset(reset),
        .q_in (jump_pc_in),
        .pc   (pc_out)
    );

    regfile U_REGFILE (
        .clk(clk),
        .RA1(Instr_rdata[19:15]),
        .RA2(Instr_rdata[24:20]),
        .WA (Instr_rdata[11:7]),
        .WD (mux_rd_data),
        .WE (register_write_en),
        .RD1(regster_data1),
        .RD2(regster_data2)
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

    ALU U_ALU (
        .RS1           (regster_data1),
        .RS2           (mux_regster_data2),
        .alu_control_en(alu_control_en),
        .B_type_data   (B_type_data),
        .alu_result    (alu_result),
        .branch_en     (branch_en)
    );

    MUX_5x1 U_RD_MUX_5 (
        .a  (alu_result),
        .b  (mem_rdata),
        .c  (lui_result),
        .d  (alui_result),
        .e  (j_result_rd),
        .sel(rd_mux_en),
        .y  (mux_rd_data)
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
        if (WE && WA != 0) begin
            reg_file[WA] <= WD;
        end
    end

    assign RD1 = (RA1 == 0) ? 32'b0 : reg_file[RA1];
    assign RD2 = (RA2 == 0) ? 32'b0 : reg_file[RA2];

endmodule

module ALU (
    input  logic [31:0] RS1,
    input  logic [31:0] RS2,
    input  logic [ 3:0] alu_control_en,
    input  logic [ 2:0] B_type_data,
    output logic [31:0] alu_result,
    output logic        branch_en
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

    always @(*) begin
        case (B_type_data)
            3'b000: branch_en = (RS1 == RS2) ? 1'b1 : 1'b0;
            3'b001: branch_en = (RS1 != RS2) ? 1'b1 : 1'b0;
            3'b100: branch_en = ($signed(RS1) < $signed(RS2)) ? 1'b1 : 1'b0;
            3'b101: branch_en = ($signed(RS1) >= $signed(RS2)) ? 1'b1 : 1'b0;
            3'b110: branch_en = ($unsigned(RS1) < $unsigned(RS2)) ? 1'b1 : 1'b0;
            3'b111:
            branch_en = ($unsigned(RS1) >= $unsigned(RS2)) ? 1'b1 : 1'b0;
            default: branch_en = 1'b0;
        endcase
    end
endmodule

module imm_extend (
    input  logic [31:0] instr_opcode,
    output logic [31:0] imm_addr
);

    logic [2:0] funct_3;
    assign funct_3 = instr_opcode[14:12];

    always_comb begin
        imm_addr = 32'b0;

        // OP-IMM
        if (instr_opcode[6:0] == 7'b0010011) begin
            case (funct_3)
                3'b001, 3'b101:  // SLLI, SRLI, SRAI
                imm_addr = {27'b0, instr_opcode[24:20]};
                3'b011:  // SLTIU
                imm_addr = {20'b0, instr_opcode[31:20]};
                default:  // ADDI, ANDI, ORI, XORI, SLTI
                imm_addr = {{20{instr_opcode[31]}}, instr_opcode[31:20]};
            endcase
        end

        // STORE (S-type)
        if (instr_opcode[6:0] == 7'b0100011) begin
            imm_addr = {
                {20{instr_opcode[31]}}, instr_opcode[31:25], instr_opcode[11:7]
            };
        end

        // LOAD (I-type)
        if (instr_opcode[6:0] == 7'b0000011) begin
            imm_addr = {{20{instr_opcode[31]}}, instr_opcode[31:20]};
        end

        // BRANCH (B-type)
        if (instr_opcode[6:0] == 7'b1100011) begin
            imm_addr = {
                {19{instr_opcode[31]}},
                instr_opcode[31],
                instr_opcode[7],
                instr_opcode[30:25],
                instr_opcode[11:8],
                1'b0
            };
        end

        // LUI
        if (instr_opcode[6:0] == 7'b0110111) begin
            imm_addr = {instr_opcode[31:12], 12'b0};
        end

        // AUIPC
        if (instr_opcode[6:0] == 7'b0010111) begin
            imm_addr = {instr_opcode[31:12], 12'b0};
        end

        // JAL (J-type)
        if (instr_opcode[6:0] == 7'b1101111) begin
            imm_addr = {
                {11{instr_opcode[31]}},
                instr_opcode[31],
                instr_opcode[19:12],
                instr_opcode[20],
                instr_opcode[30:21],
                1'b0
            };
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

module MUX_5x1 (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic [31:0] c,
    input  logic [31:0] d,
    input  logic [31:0] e,
    input  logic [ 2:0] sel,
    output logic [31:0] y
);

    always_comb begin
        case (sel)
            3'b000:  y = a;
            3'b001:  y = b;
            3'b010:  y = c;
            3'b011:  y = d;
            3'b100:  y = e;
            default: y = 32'bx;
        endcase
    end

endmodule
