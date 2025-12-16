`timescale 1ns / 1ps

interface cpu_if (
    input logic clk
);
    logic reset;
    logic [31:0] instr_code;
    logic [31:0] alu_result;
endinterface

class transaction;
    rand bit [ 2:0] funct3;
    rand bit [ 4:0] rs1,        rs2, rd;
     bit      [11:0] imm;
    rand bit           funct7b5;  // funct7[5]
    bit      [31:0] rs1_val;
    bit      [31:0] rs2_val;
    bit      [31:0] instr_code;
    bit      [31:0] exp_result;
    bit      [ 6:0] opcode;
    bit      [ 6:0] funct7;


    function void build_instr();  // I-type(OP-IMM)濡� �궗�슜
        opcode = 7'b0010011;  // OP-IMM
        if (funct3 inside {3'b001, 3'b101}) begin
            imm[4:0] = rs2[4:0];
            imm[11:5] = (funct3 == 3'b101 && funct7b5) ? 7'b0100000 : 7'b0000000;
        end else begin
            imm = $urandom_range(0, 4095);
        end
        instr_code = {imm, rs1, funct3, rd, opcode};
    endfunction


    function void calc_expected();
        bit [6:0] opc = instr_code[6:0];
        bit [2:0] f3 = instr_code[14:12];
        bit signed [31:0] simm;
        bit [4:0] shamt;
        bit [6:0] imm_hi;

        simm = $signed({{20{instr_code[31]}},
                        instr_code[31:20]});  // sign-extend imm
        shamt = instr_code[24:20];
        imm_hi = instr_code[31:25];

        if (opc != 7'b0010011) begin
            exp_result = 32'hDEADBEEF;
            return;
        end

        case (f3)
            3'b000:  exp_result = rs1_val + simm;  // ADDI
            3'b010:  exp_result = ($signed(rs1_val) < simm) ? 1 : 0;  // SLTI
            3'b011:  exp_result = (rs1_val < simm) ? 1 : 0;  // SLTIU
            3'b100:  exp_result = rs1_val ^ simm;  // XORI
            3'b110:  exp_result = rs1_val | simm;  // ORI
            3'b111:  exp_result = rs1_val & simm;  // ANDI
            3'b001:  exp_result = rs1_val << shamt;  // SLLI
            3'b101: begin  // SRLI / SRAI
                if (imm_hi == 7'b0100000)
                    exp_result = $signed(rs1_val) >>> shamt;  // SRAI
                else exp_result = rs1_val >> shamt;  // SRLI
            end
            default: exp_result = 32'hDEADBEEF;
        endcase
    endfunction


    constraint funct3_c {
        funct3 inside {3'b000,3'b111,3'b110,3'b100,3'b010,3'b011,3'b001,3'b101};
        if (funct3 == 3'b001)
        funct7b5 inside {0};
        else
        if (funct3 == 3'b101)
        funct7b5 inside {0, 1};
        else
        funct7b5 == 0;
    }

    constraint reg_addr {
        rs1 inside {[1 : 31]};
        rs2 inside {[1 : 31]};
        rd inside {[1 : 31]};
        rs1 != rs2;
    }
endclass

class generator;
    mailbox #(transaction) gen2drv;
    event gen_next_event;
    bit [6:0] f7;
    bit [6:0] imm_hi;
    bit [2:0] f3;
    int unsigned cnt_add, cnt_sub, cnt_and, cnt_or, cnt_xor;
    int unsigned cnt_slt, cnt_sltu, cnt_sll, cnt_srl, cnt_sra;

    function new(mailbox#(transaction) gen2drv, event gen_next_event);
        this.gen2drv = gen2drv;
        this.gen_next_event = gen_next_event;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr = new();
            assert (tr.randomize());
            tr.build_instr();

            imm_hi = tr.instr_code[31:25];
            f7 = tr.funct7;
            f3 = tr.funct3;

            case (tr.funct3)
                3'b000: cnt_add++;  // ADDI
                3'b111: cnt_and++;  // ANDI
                3'b110: cnt_or++;  // ORI
                3'b100: cnt_xor++;  // XORI
                3'b010: cnt_slt++;  // SLTI
                3'b011: cnt_sltu++;  // SLTIU
                3'b001: cnt_sll++;  // SLLI
                3'b101: begin
                    if (imm_hi == 7'b0100000) cnt_sra++;  // SRAI
                    else cnt_srl++;  // SRLI
                end
            endcase

            $display(
                "[GEN] instr=%h rs1=%0d imm=%0d rd=%0d funct3=%0b",
                tr.instr_code, tr.rs1, tr.imm, tr.rd, tr.funct3);

            gen2drv.put(tr);
            @(gen_next_event);
        end

        $display(
            "ADD=%0d SUB=%0d AND=%0d OR=%0d XOR=%0d SLT=%0d SLTU=%0d SLL=%0d SRL=%0d SRA=%0d",
            cnt_add, cnt_sub, cnt_and, cnt_or, cnt_xor, cnt_slt, cnt_sltu,
            cnt_sll, cnt_srl, cnt_sra);
    endtask
endclass

class driver;
    virtual cpu_if cif;
    mailbox #(transaction) gen2drv;
    mailbox #(transaction) drv2mon;

    function new(mailbox#(transaction) gen2drv, mailbox#(transaction) drv2mon,
                 virtual cpu_if cif);
        this.gen2drv = gen2drv;
        this.drv2mon = drv2mon;
        this.cif = cif;
    endfunction

    task reset();
        cif.reset = 1;
        repeat (5) @(posedge cif.clk);
        cif.reset = 0;
    endtask

    task run();
        #1;
        forever begin
            transaction tr;
            gen2drv.get(tr);
            cif.instr_code = tr.instr_code;
            drv2mon.put(tr);
        end
    endtask
endclass

class monitor;
    virtual cpu_if cif;
    mailbox #(transaction) drv2mon;
    mailbox #(transaction) mon2scb;

    logic [31:0] mem[0:31];

    function memoery_Gen();
        foreach (mem[i]) mem[i] = i;
    endfunction

    function new(mailbox#(transaction) drv2mon, mailbox#(transaction) mon2scb,
                 virtual cpu_if cif);
        this.drv2mon = drv2mon;
        this.mon2scb = mon2scb;
        this.cif = cif;
        memoery_Gen();
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            drv2mon.get(tr);

            tr.rs1_val = mem[tr.rs1];
            tr.rs2_val = mem[tr.rs2];
            tr.calc_expected();

            @(posedge cif.clk);
            if (tr.rd != 0) mem[tr.rd] = cif.alu_result;

            mon2scb.put(tr);
        end
    endtask
endclass

class scoreboard;
    virtual cpu_if cif;
    mailbox #(transaction) mon2scb;
    event gen_next_event;

    int pass_count = 0;
    int fail_count = 0;

    function new(mailbox#(transaction) mon2scb, event gen_next_event,
                 virtual cpu_if cif);
        this.mon2scb = mon2scb;
        this.gen_next_event = gen_next_event;
        this.cif = cif;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            mon2scb.get(tr);

            if (cif.alu_result === tr.exp_result) begin
                pass_count++;
                $display("[%0t ns] [SCB] PASS  ALU=%0d  EXP=%0d", $time,
                         cif.alu_result, tr.exp_result);
            end else begin
                fail_count++;
                $display("[%0t ns] [SCB] FAIL  ALU=%0d  EXP=%0d", $time,
                         cif.alu_result, tr.exp_result);
            end

            ->gen_next_event;
        end

        $display("PASS=%0d FAIL=%0d", pass_count, fail_count);
    endtask
endclass

class env;
    generator gen;
    driver drv;
    monitor mon;
    scoreboard scb;

    mailbox #(transaction) gen2drv, drv2mon, mon2scb;
    event gen_next_event;

    virtual cpu_if cif;

    function new(virtual cpu_if cif);
        this.cif = cif;
        gen2drv = new();
        drv2mon = new();
        mon2scb = new();
        gen = new(gen2drv, gen_next_event);
        drv = new(gen2drv, drv2mon, cif);
        mon = new(drv2mon, mon2scb, cif);
        scb = new(mon2scb, gen_next_event, cif);
    endfunction

    task run(int n);
        drv.reset();
        fork
            gen.run(n);
            drv.run();
            mon.run(n);
            scb.run(n);
        join_any
        $finish;
    endtask
endclass

module tb_rv32i;
    logic clk;
    cpu_if cif (clk);
    env e;

    RV32I_TOP dut (
        .clk(clk),
        .reset(cif.reset),
        .Instr_rdata(cif.instr_code),
        .alu_result(cif.alu_result)
    );

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        e = new(cif);
        e.run(500);
    end
endmodule
