`timescale 1ns / 1ps

`define BEQ 3'b000
`define BNE 3'b001
`define BLT 3'b100
`define BGE 3'b101
`define BLTU 3'b110
`define BGEU 3'b111
`define OP_B_TYPE 7'b1100011

//====================================================
// Interface
//====================================================
interface cpu_if (
    input logic clk
);
    logic        reset;
    logic [31:0] instr_code;

    // 관측용(탑에서 assign으로 연결)
    logic [31:0] instr_addr;
endinterface

//====================================================
// Transaction (B-Type)
//====================================================
class transaction;
    rand logic [ 4:0] rs1,         rs2;
    rand logic [ 2:0] funct3;
    // B-type immediate는 13-bit(imm[12:0])이며 imm[0]=0(2바이트 정렬)
    rand logic [12:0] imm;

    logic      [31:0] instr_code;

    // 관측/기대값
    logic      [31:0] cur_pc;
    logic      [31:0] rs1_val;
    logic      [31:0] rs2_val;
    logic             exp_taken;
    logic      [31:0] exp_next_pc;

    // constraint
    constraint imm_c {
        imm[1:0] == 2'b00;
        imm inside {[0 : 8]};
    }

    constraint funct3_c {funct3 inside {`BEQ, `BNE, `BLT, `BGE, `BLTU, `BGEU};}

    function void build_instr();
        instr_code = {
            imm[12], imm[10:5], rs2, rs1, funct3, imm[4:1], imm[11], `OP_B_TYPE
        };
    endfunction

    function void calc_expected(input logic [31:0] golden_rs1,
                                input logic [31:0] golden_rs2,
                                input logic [31:0] pc_now);
        rs1_val = golden_rs1;
        rs2_val = golden_rs2;
        cur_pc  = pc_now;

        unique case (funct3)
            `BEQ: exp_taken = (rs1_val == rs2_val);
            `BNE: exp_taken = (rs1_val != rs2_val);
            `BLT: exp_taken = ($signed(rs1_val) < $signed(rs2_val));
            `BGE: exp_taken = ($signed(rs1_val) >= $signed(rs2_val));
            `BLTU: exp_taken = (rs1_val < rs2_val);
            `BGEU: exp_taken = (rs1_val >= rs2_val);
            default: exp_taken = 1'b0;
        endcase

        exp_next_pc = cur_pc + (exp_taken ? $signed(imm) : 32'd4);
    endfunction
endclass

//====================================================
// Generator
//====================================================
class generator;
    mailbox #(transaction) gen2drv;
    event gen_next_event;

    function new(mailbox#(transaction) m, event e);
        gen2drv        = m;
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

//====================================================
// Driver
//====================================================
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
        vif.instr_code = 32'h0000_0013;  // NOP(ADDI x0,x0,0)
        repeat (3) @(posedge vif.clk);
        vif.reset = 0;

        for (int i = 0; i < 32; i++) begin
            tb_b_type.dut.U_DP.U_REGFILE.reg_file[i] = i[31:0];
        end
        tb_b_type.dut.U_DP.U_REGFILE.reg_file[0] = 32'h0;
    endtask

    task run();
        forever begin
            transaction tr;
            gen2drv.get(tr);

            // instr_addr(현재 PC) 관측 후, 다음 네거티브에서 명령 주입
            @(negedge vif.clk);
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
            tr.cur_pc = vif.instr_addr;

            // 다음 포지티브에서 실행되었다고 가정
            @(posedge vif.clk);
            mon2scb.put(tr);
        end
    endtask
endclass

//====================================================
// Scoreboard (Golden Model)
//====================================================
class scoreboard;
    mailbox #(transaction) mon2scb;
    event gen_next_event;

    logic [31:0] golden_reg[0:31];
    int pass_count, fail_count;

    function new(mailbox#(transaction) m2s, event e);
        mon2scb        = m2s;
        gen_next_event = e;

        for (int i = 0; i < 32; i++) golden_reg[i] = i[31:0];
        golden_reg[0] = 32'h0;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            logic [31:0] dut_next_pc;

            mon2scb.get(tr);

            // 기대값 계산(현재 PC는 monitor가 캡처한 값 사용)
            tr.calc_expected(golden_reg[tr.rs1], golden_reg[tr.rs2], tr.cur_pc);

            // DUT의 다음 PC(= Instr_Addr) 관측
            #1;
            dut_next_pc = tb_b_type.inst_addr;

            if (dut_next_pc === tr.exp_next_pc) begin
                pass_count++;
                $display(
                    "PASS: funct3=%b rs1=x%0d(0x%08h) rs2=x%0d(0x%08h) imm=%0d cur_pc=0x%08h -> next_pc=0x%08h",
                    tr.funct3, tr.rs1, tr.rs1_val, tr.rs2, tr.rs2_val,
                    $signed(tr.imm), tr.cur_pc, dut_next_pc);
            end else begin
                fail_count++;
                $display(
                    "FAIL: funct3=%b rs1=x%0d(0x%08h) rs2=x%0d(0x%08h) imm=%0d cur_pc=0x%08h | exp_taken=%0d exp_next_pc=0x%08h got_next_pc=0x%08h",
                    tr.funct3, tr.rs1, tr.rs1_val, tr.rs2, tr.rs2_val,
                    $signed(tr.imm), tr.cur_pc, tr.exp_taken, tr.exp_next_pc,
                    dut_next_pc);
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

//====================================================
// Top Testbench
//====================================================
module tb_b_type;
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
