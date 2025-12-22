`timescale 1ns / 1ps

`define LB  3'b000
`define LH  3'b001
`define LW  3'b010
`define LBU 3'b100
`define LHU 3'b101
`define OP_L_TYPE 7'b0000011

//====================================================
// Interface
//====================================================
interface cpu_if (input logic clk);
    logic        reset;
    logic [31:0] instr_code;
endinterface

//====================================================
// Transaction
//====================================================
class transaction;
    rand logic [4:0]  rs1, rd;
    rand logic [11:0] imm;
    rand logic [2:0]  funct3;

    logic [31:0] instr_code;
    logic [31:0] rs1_val;
    logic [31:0] exp_addr;
    logic [31:0] exp_rd_data;

    constraint imm_c    { imm inside {[0:10]}; }
    constraint funct3_c { funct3 inside {`LB, `LH, `LW, `LBU, `LHU}; }
    constraint reg_c    { rs1 inside {[1:31]}; rd inside {[1:31]}; }
    constraint align_c {
        if (funct3 == `LH || funct3 == `LHU) imm[0] == 0;
        if (funct3 == `LW) imm[1:0] == 0;
    }

    function void build_instr();
        instr_code = {imm, rs1, funct3, rd, `OP_L_TYPE};
    endfunction

    function void calc_expected(input logic [31:0] golden_rs1);
        logic [31:0] word;
        logic [1:0]  byte_sel;

        rs1_val  = golden_rs1;
        exp_addr = rs1_val + $signed(imm);
        byte_sel = exp_addr[1:0];

        word = tb_l_type.U_DMEM.data_memory[exp_addr[4:2]];

        case (funct3)
            `LB: begin
                case (byte_sel)
                    2'b00: exp_rd_data = {{24{word[7]}},  word[7:0]};
                    2'b01: exp_rd_data = {{24{word[15]}}, word[15:8]};
                    2'b10: exp_rd_data = {{24{word[23]}}, word[23:16]};
                    2'b11: exp_rd_data = {{24{word[31]}}, word[31:24]};
                endcase
            end
            `LBU: begin
                case (byte_sel)
                    2'b00: exp_rd_data = {24'b0, word[7:0]};
                    2'b01: exp_rd_data = {24'b0, word[15:8]};
                    2'b10: exp_rd_data = {24'b0, word[23:16]};
                    2'b11: exp_rd_data = {24'b0, word[31:24]};
                endcase
            end
            `LH: begin
                if (exp_addr[1] == 0)
                    exp_rd_data = {{16{word[15]}}, word[15:0]};
                else
                    exp_rd_data = {{16{word[31]}}, word[31:16]};
            end
            `LHU: begin
                if (exp_addr[1] == 0)
                    exp_rd_data = {16'b0, word[15:0]};
                else
                    exp_rd_data = {16'b0, word[31:16]};
            end
            `LW: exp_rd_data = word;
        endcase
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

    task reset();
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
// Scoreboard (Golden Model)
//====================================================
class scoreboard;
    mailbox #(transaction) mon2scb;
    event gen_next_event;

    logic [31:0] golden_reg[0:31];
    int pass_count, fail_count;

    function new(mailbox#(transaction) m2s, event e);
        mon2scb = m2s;
        gen_next_event = e;

        for (int i = 0; i < 32; i++)
            golden_reg[i] = i;
        golden_reg[0] = 0;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            logic [31:0] rd_val;

            mon2scb.get(tr);

            tr.calc_expected(golden_reg[tr.rs1]);
            #1;
            rd_val = tb_l_type.dut.U_DP.U_REGFILE.reg_file[tr.rd];

            if (rd_val === tr.exp_rd_data) begin
                pass_count++;
                $display("PASS: rd=x%0d exp=0x%0h got=0x%0h",
                         tr.rd, tr.exp_rd_data, rd_val);
            end else begin
                fail_count++;
                $display("FAIL: rd=x%0d exp=0x%0h got=0x%0h | rs1=x%0d rs1_val=0x%0h imm=%0d funct3=%b addr=0x%0h",
                         tr.rd, tr.exp_rd_data, rd_val,
                         tr.rs1, golden_reg[tr.rs1],
                         $signed(tr.imm), tr.funct3, tr.exp_addr);
            end

            if (tr.rd != 0)
                golden_reg[tr.rd] = tr.exp_rd_data;

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

//====================================================
// Top Testbench
//====================================================
module tb_l_type;
    logic clk;
    cpu_if vif(clk);
    environment env;

    logic [31:0] inst_addr, inst_rdata;
    logic [31:0] mem_addr, mem_wdata, mem_rdata;
    logic        mem_write_en;
    logic [2:0]  s_type, l_type;

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

    dMEM U_DMEM (
        .clk         (clk),
        .reset       (vif.reset),
        .mem_write_en(mem_write_en),
        .s_type      (s_type),
        .l_type      (l_type),
        .mem_addr    (mem_addr),
        .mem_wdata   (mem_wdata),
        .mem_rdata   (mem_rdata)
    );

    assign inst_rdata = vif.instr_code;

    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        env = new(vif);
        env.run(200);
    end
endmodule
