`timescale 1ns / 1ps


module IMEM (
    input  logic [31:0] Instr_Addr,
    output logic [31:0] Instr_rdata
);

    logic [31:0] mem[0:31];

    initial begin
        // mem[0] = 32'h00310093; // ADDI x1, x2, 3
        // mem[1] = 32'hFFD28113; // ADDI x2, x5, -3 (SUBI 흉내)
        // mem[2] = 32'h00944493; // XORI x9, x8, 9
        // mem[3] = 32'h00C5A513; // ORI  x10, x11, 12
        // mem[4] = 32'h00F6C613; // ANDI x12, x13, 15
        // mem[5] = 32'h00311013; // SLLI x0, x2, 3
        // mem[6] = 32'h00315013; // SRLI x0, x2, 3
        // mem[7] = 32'h40315013; // SRAI x0, x2, 3
        // mem[8] = 32'h00312013; // SLTI x0, x2, 3
        // mem[9] = 32'h00313013; // SLTIU x0, x2, 3

        // mem[0] = 32'h0020A023;  // SW x2, 0(x1)
        // mem[1] = 32'h00208223;  // SB x2, 4(x1)
        // mem[2] = 32'h00209423;  // SH x2, 8(x1)

        // mem[0] = 32'h0000A303;  // LW  x6, 0(x1)
        // mem[1] = 32'h00408383;  // LB  x7, 4(x1)
        // mem[2] = 32'h0040C403;  // LBU x8, 4(x1)
        // mem[3] = 32'h00809483;  // LH  x9, 8(x1)
        // mem[4] = 32'h0080D503;  // LHU x10, 8(x1)

        // mem[0] = 32'h00110463;  // BEQ  x2, x1, +8
        // mem[1] = 32'h00209663;  // BNE  x1, x2, +8
        // mem[2] = 32'h0020C663;  // BLT  x1, x2, +8
        // mem[3] = 32'h0020D663;  // BGE  x1, x2, +8
        // mem[4] = 32'h0020E663;  // BLTU x1, x2, +8
        // mem[5] = 32'h0020F663;  // BGEU x1, x2, +8
        // mem[1] = 32'h00209663;  // BNE  x1, x2, +8
        // mem[2] = 32'h0020C663;  // BLT  x1, x2, +8
        // mem[3] = 32'h0020D663;  // BGE  x1, x2, +8
        // mem[4] = 32'h0020E663;  // BLTU x1, x2, +8
        // mem[5] = 32'h0020F663;  // BGEU x1, x2, +8

        mem[0] = 32'h123450B7;
        mem[1] = 32'hFFFFF137;


    end

    assign Instr_rdata = mem[Instr_Addr[5:2]];

endmodule
