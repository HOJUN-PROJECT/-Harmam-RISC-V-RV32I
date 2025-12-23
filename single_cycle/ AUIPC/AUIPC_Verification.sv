`timescale 1ns/1ps

`define OP_AUIPC 7'b0010111

//====================================================
// Interface
//====================================================
interface cpu_if (input logic clk);
    logic        reset;
    logic [31:0] instr_code;

    // 관측용
    logic [31:0] instr_addr;
endinterface

//====================================================
// Transaction (AUIPC)
//====================================================
class transaction;
    rand logic [4:0]  rd;
    rand logic [19:0] imm;

    logic [31:0] instr_code;

    // PC 캡처
    logic [31:0] pc_val;

    // 기대값
    logic [31:0] exp_rd_data;

    constraint rd_c  { rd inside {[1:31]}; }   // x0 제외
    constraint imm_c { imm inside {[0:20'h00A]}; } // 필요 시 확대

    function void build_instr();
        // [31:12]=imm, [11:7]=rd, [6:0]=opcode
        instr_code = {imm, rd, `OP_AUIPC};
    endfunction

    function void calc_expected();
        exp_rd_data = pc_val + (imm << 12);
    endfunction
endclass

//====================================================
// Generator
//====================================================
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
            assert(tr.randomize());
            tr.build_instr();
            gen2drv.put(tr);
            @(gen_next_event);
        end
    endtask
endclass

//====================================================
// Driver
//====================================================
class driver;
    virtual cpu_if vif;
    mailbox #(transaction) gen2drv, drv2mon;

    function new(mailbox#(transaction) g2d,
                 mailbox#(transaction) d2m,
                 virtual cpu_if vif);
        this.gen2drv = g2d;
        this.drv2mon = d2m;
        this.vif     = vif;
    endfunction

    task reset_only();
        vif.reset      = 1;
        vif.instr_code = 32'h0000_0013; // NOP
        repeat (3) @(posedge vif.clk);
        vif.reset = 0;
    endtask

    task run();
        forever begin
            transaction tr;
            gen2drv.get(tr);

            @(negedge vif.clk);
            tr.pc_val      = vif.instr_addr; // ★ AUIPC 핵심
            vif.instr_code = tr.instr_code;

            drv2mon.put(tr);
        end
    endtask
endclass

//====================================================
// Monitor
//====================================================
class monitor;
    virtual cpu_if vif;
    mailbox #(transaction) drv2mon, mon2scb;

    function new(mailbox#(transaction) d2m,
                 mailbox#(transaction) m2s,
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

//====================================================
// Scoreboard
//====================================================
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

            mon2scb.get(tr);

            tr.calc_expected();

            #1;
            rd_val = tb_auipc.dut.U_DP.U_REGFILE.reg_file[tr.rd];

            if (rd_val === tr.exp_rd_data) begin
                pass_count++;
                $display("PASS: AUIPC rd=x%0d PC=0x%08h imm=0x%05h exp=0x%08h got=0x%08h",
                         tr.rd, tr.pc_val, tr.imm, tr.exp_rd_data, rd_val);
            end else begin
                fail_count++;
                $display("FAIL: AUIPC rd=x%0d PC=0x%08h imm=0x%05h exp=0x%08h got=0x%08h instr=0x%08h",
                         tr.rd, tr.pc_val, tr.imm, tr.exp_rd_data, rd_val, tr.instr_code);
            end

            ->gen_next_event;
        end

        $display("======================================");
        $display("RESULT :: PASS=%0d FAIL=%0d", pass_count, fail_count);
        $display("======================================");
    endtask
endclass

//====================================================
// Environment
//====================================================
class environment;
    generator  gen;
    driver     drv;
    monitor    mon;
    scoreboard scb;

    mailbox #(transaction) gen2drv, drv2mon, mon2scb;
    event gen_next_event;
    virtual cpu_if vif;

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
        drv.reset_only();
        fork
            gen.run(n);
            drv.run();
            mon.run(n);
            scb.run(n);
        join_any
        $finish;
    endtask
endclass

//====================================================
// Top Testbench (AUIPC)
//====================================================
module tb_auipc;
    logic clk;
    cpu_if vif(clk);
    environment env;

    logic [31:0] inst_addr, inst_rdata;
    logic [31:0] mem_addr, mem_wdata, mem_rdata;
    logic        mem_write_en;
    logic [2:0]  s_type, l_type;

    RV32I_TOP dut (
        .clk          (clk),
        .reset        (vif.reset),
        .Instr_rdata  (inst_rdata),
        .Instr_Addr   (inst_addr),
        .alu_result   (mem_addr),
        .S_type_data  (s_type),
        .mem_write_en (mem_write_en),
        .mem_wdata    (mem_wdata),
        .L_type_data  (l_type),
        .mem_rdata    (mem_rdata)
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
