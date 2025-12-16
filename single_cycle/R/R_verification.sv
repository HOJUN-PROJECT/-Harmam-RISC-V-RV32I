`timescale 1ns / 1ps

interface cpu_if (
    input logic clk
);
    logic reset;
    logic [31:0] instr_code;
    logic [31:0] alu_result;
    logic [31:0] exp_result;
endinterface

class transaction;
    rand bit [ 2:0] funct3;
    rand bit        funct7b5;
    rand bit [ 4:0] rs1,        rs2, rd;

    bit      [31:0] rs1_val;
    bit      [31:0] rs2_val;
    bit      [31:0] instr_code;
    bit      [31:0] exp_result;
    bit      [ 6:0] opcode;
    bit      [ 6:0] funct7;

    function void build_instr();
        opcode = 7'b0110011;
        funct7 = (funct7b5) ? 7'b0100000 : 7'b0000000;
        instr_code = {funct7, rs2, rs1, funct3, rd, opcode};
    endfunction

    function void calc_expected();
        bit [6:0] f7 = instr_code[31:25];
        bit [2:0] f3 = instr_code[14:12];

        case ({
            f7[5], f3
        })
            4'b0000: exp_result = rs1_val + rs2_val;
            4'b1000: exp_result = rs1_val - rs2_val;
            4'b0111: exp_result = rs1_val & rs2_val;
            4'b0110: exp_result = rs1_val | rs2_val;
            4'b0100: exp_result = rs1_val ^ rs2_val;
            4'b0010: exp_result = ($signed(rs1_val) < $signed(rs2_val)) ? 1 : 0;
            4'b0011: exp_result = (rs1_val < rs2_val) ? 1 : 0;
            4'b0001: exp_result = rs1_val << rs2_val[4:0];
            4'b0101: exp_result = rs1_val >> rs2_val[4:0];
            4'b1101: exp_result = $signed(rs1_val) >>> rs2_val[4:0];
            default: exp_result = 32'hDEADBEEF;
        endcase
    endfunction

    constraint funct7_c {funct7 inside {7'b0100000, 7'b0000000};}

    constraint funct3_c {
        funct3 inside {3'b000,3'b111,3'b110,3'b100,3'b010,3'b011,3'b001,3'b101};
        if (funct3 == 3'b000)
        funct7b5 inside {0, 1};
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

            f7 = tr.funct7;
            f3 = tr.funct3;

            case ({
                f7[5], f3
            })
                4'b0000: cnt_add++;
                4'b1000: cnt_sub++;
                4'b0111: cnt_and++;
                4'b0110: cnt_or++;
                4'b0100: cnt_xor++;
                4'b0010: cnt_slt++;
                4'b0011: cnt_sltu++;
                4'b0001: cnt_sll++;
                4'b0101: cnt_srl++;
                4'b1101: cnt_sra++;
            endcase

            $display(
                "[GEN] instr=%h rs1=%0d rs2=%0d rd=%0d funct3=%0d funct7b5=%0d",
                tr.instr_code, tr.rs1, tr.rs2, tr.rd, tr.funct3, tr.funct7b5);

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


module tb_CPU_TOP;
    logic clk;
    logic reset;

    CPU_TOP dut (
        .clk  (clk),
        .reset(reset)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        reset = 1;
        #5;
        reset = 0;
    end
endmodule
