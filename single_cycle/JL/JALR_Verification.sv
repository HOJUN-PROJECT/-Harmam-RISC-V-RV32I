`timescale 1ns / 1ps

`define OP_JALR 7'b1100111


interface cpu_if (
    input logic clk
);
    logic        reset;
    logic [31:0] instr_code;
    logic [31:0] instr_addr;  // PC 관측
endinterface

class transaction;
    rand logic [4:0] rd;
    rand logic [4:0] rs1;
    rand logic signed [11:0] imm;

    logic [31:0] instr_code;
    logic [31:0] pc_val;

    logic [31:0] exp_rd_data;  // PC+4
    logic [31:0] exp_pc;

    constraint rd_c {rd inside {[0 : 31]};}
    constraint rs1_c {rs1 inside {[1 : 31]};}
    constraint imm_c {
        imm inside {[0 : 12]};
        imm % 4 == 0;
    }


    // JALR encoding (I-type)
    function void build_instr();
        instr_code = {
            imm[11:0],  // imm
            rs1,  // rs1
            3'b000,  // funct3
            rd,  // rd
            `OP_JALR
        };
    endfunction

    function void calc_expected(input logic [31:0] rs1_val);
        exp_rd_data = pc_val + 32'd4;
        exp_pc      = (rs1_val + imm) & 32'hFFFF_FFFE;
    endfunction
endclass


class generator;
    mailbox #(transaction) gen2drv;
    event gen_next_event;

    function new(mailbox#(transaction) m, event e);
        gen2drv = m;
        gen_next_event = e;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr = new();
            assert (tr.randomize());
            tr.build_instr();
            gen2drv.put(tr);
            @(gen_next_event);
        end
    endtask
endclass

class driver;
    virtual cpu_if vif;
    mailbox #(transaction) gen2drv, drv2mon;

    function new(mailbox#(transaction) g2d, mailbox#(transaction) d2m,
                 virtual cpu_if vif);
        this.gen2drv = g2d;
        this.drv2mon = d2m;
        this.vif     = vif;
    endfunction

    task reset_and_init_regs();
        vif.reset      = 1;
        vif.instr_code = 32'h0000_0013;  // NOP
        repeat (3) @(posedge vif.clk);
        vif.reset = 0;

        for (int i = 0; i < 32; i++) tb_jalr.dut.U_DP.U_REGFILE.reg_file[i] = i;
        tb_jalr.dut.U_DP.U_REGFILE.reg_file[0] = 0;
    endtask

    task run();
        forever begin
            transaction tr;
            gen2drv.get(tr);

            @(negedge vif.clk);
            tr.pc_val      = vif.instr_addr;
            vif.instr_code = tr.instr_code;

            drv2mon.put(tr);
        end
    endtask
endclass

class monitor;
    virtual cpu_if vif;
    mailbox #(transaction) drv2mon, mon2scb;

    function new(mailbox#(transaction) d2m, mailbox#(transaction) m2s,
                 virtual cpu_if vif);
        this.drv2mon = d2m;
        this.mon2scb = m2s;
        this.vif     = vif;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            drv2mon.get(tr);
            @(posedge vif.clk);
            mon2scb.put(tr);
        end
    endtask
endclass


class scoreboard;
    mailbox #(transaction) mon2scb;
    event gen_next_event;

    int pass_count, fail_count;

    function new(mailbox#(transaction) m2s, event e);
        mon2scb = m2s;
        gen_next_event = e;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            logic [31:0] rd_val;
            logic [31:0] rs1_val;

            mon2scb.get(tr);

            rs1_val = tb_jalr.dut.U_DP.U_REGFILE.reg_file[tr.rs1];
            tr.calc_expected(rs1_val);

            #1;

            // PC 검증
            if (tb_jalr.dut.Instr_Addr !== tr.exp_pc) begin
                $display(
                    "FAIL(PC): rs1=x%0d(0x%08h) imm=%0d exp_pc=0x%08h got=0x%08h",
                    tr.rs1, rs1_val, tr.imm, tr.exp_pc, tb_jalr.dut.Instr_Addr);
                fail_count++;
            end

            // RD 검증
            if (tr.rd != 0) begin
                rd_val = tb_jalr.dut.U_DP.U_REGFILE.reg_file[tr.rd];
                if (rd_val !== tr.exp_rd_data) begin
                    $display("FAIL(RD): rd=x%0d exp=0x%08h got=0x%08h", tr.rd,
                             tr.exp_rd_data, rd_val);
                    fail_count++;
                end else begin
                    $display(
                        "PASS: JALR rd=x%0d rs1=x%0d imm=%0d | PC_old=0x%08h -> PC_new=0x%08h rd=0x%08h",
                        tr.rd, tr.rs1, tr.imm, tr.pc_val, tr.exp_pc, rd_val);
                    pass_count++;
                end
            end else begin
                $display(
                    "PASS: JALR rd=x0 (ignored) rs1=x%0d imm=%0d PC_new=0x%08h",
                    tr.rs1, tr.imm, tr.exp_pc);
                pass_count++;
            end

            ->gen_next_event;
        end

        $display("======================================");
        $display("JALR RESULT :: PASS=%0d FAIL=%0d", pass_count, fail_count);
        $display("======================================");
    endtask
endclass


class environment;
    generator              gen;
    driver                 drv;
    monitor                mon;
    scoreboard             scb;

    mailbox #(transaction) gen2drv,        drv2mon, mon2scb;
    event                  gen_next_event;
    virtual cpu_if         vif;

    function new(virtual cpu_if vif);
        this.vif = vif;
        gen2drv = new();
        drv2mon = new();
        mon2scb = new();

        gen = new(gen2drv, gen_next_event);
        drv = new(gen2drv, drv2mon, vif);
        mon = new(drv2mon, mon2scb, vif);
        scb = new(mon2scb, gen_next_event);
    endfunction

    task run(int n);
        drv.reset_and_init_regs();
        fork
            gen.run(n);
            drv.run();
            mon.run(n);
            scb.run(n);
        join_any
        $finish;
    endtask
endclass

module tb_jalr;
    logic clk;
    cpu_if vif (clk);
    environment env;

    logic [31:0] inst_addr, inst_rdata;
    logic [31:0] mem_addr, mem_wdata, mem_rdata;
    logic mem_write_en;
    logic [2:0] s_type, l_type;

    RV32I_TOP dut (
        .clk         (clk),
        .reset       (vif.reset),
        .Instr_rdata (inst_rdata),
        .Instr_Addr  (inst_addr),
        .alu_result  (mem_addr),
        .S_type_data (s_type),
        .mem_write_en(mem_write_en),
        .mem_wdata   (mem_wdata),
        .L_type_data (l_type),
        .mem_rdata   (mem_rdata)
    );

    assign inst_rdata     = vif.instr_code;
    assign vif.instr_addr = inst_addr;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        env = new(vif);
        env.run(200);
    end
endmodule
