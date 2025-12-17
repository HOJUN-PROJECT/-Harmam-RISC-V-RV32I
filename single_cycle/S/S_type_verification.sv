`timescale 1ns / 1ps


`define SB 3'b000
`define SH 3'b001
`define SW 3'b010
`define OP_S_TYPE 7'b0100_011

interface cpu_if (
    input logic clk
);
    logic        reset;
    logic [31:0] instr_code;
    // DUT outputs to be monitored
    logic        d_wr_en;
    logic [31:0] dAddr;
    logic [ 2:0] store_type;
    logic [31:0] dWdata;
endinterface

class transaction;
    rand logic [4:0] rs1, rs2;
    rand logic [11:0] imm;
    rand logic [2:0] funct3;

    logic [31:0] instr_code;
    logic [31:0] rs1_val, rs2_val;
    logic [31:0] exp_addr;
    logic [31:0] exp_dWdata;

    constraint funct3_c {funct3 inside {`SB, `SH, `SW};}
    constraint imm_c {
        imm inside {[0 : 10]};
    }  // Limit offset to fit in data_ram
    constraint rs_c {
        rs1 inside {[1 : 21]};
        rs2 inside {[1 : 31]};
    }

    function void build_instr();
        instr_code = {imm[11:5], rs2, rs1, funct3, imm[4:0], `OP_S_TYPE};
    endfunction

    function void calc_expected();
        exp_addr   = rs1_val + $signed(imm);
        exp_dWdata = rs2_val;
    endfunction

    function string get_instr_name();
        case (funct3)
            `SB: return "SB";
            `SH: return "SH";
            `SW: return "SW";
            default: return "UNKNOWN";
        endcase
    endfunction

    task display(string tag);
        $display(
            "%0t [%s] instr=0x%0h(%s) | rs1=x%0d(val=0x%0d) | rs2=x%0d(val=0x%0d) | imm=%0d | exp_addr=0x%0d | exp_data=0x%0d",
            $time, tag, instr_code, get_instr_name(), rs1, rs1_val, rs2,
            rs2_val, $signed(imm), exp_addr, exp_dWdata);
    endtask
endclass

class generator;
    mailbox #(transaction) gen2drv;
    event gen_next_event;
    int total_count;

    function new(mailbox#(transaction) gen2drv, event gen_next_event);
        this.gen2drv = gen2drv;
        this.gen_next_event = gen_next_event;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr = new();
            assert (tr.randomize());

            tr.rs1_val = tb_s_type.dut.U_DP.U_REGFILE.reg_file[tr.rs1];
            tr.rs2_val = tb_s_type.dut.U_DP.U_REGFILE.reg_file[tr.rs2];


            tr.build_instr();
            tr.calc_expected();

            gen2drv.put(tr);
            tr.display("GEN");
            total_count++;

            @(gen_next_event);
        end
    endtask
endclass

class driver;
    virtual cpu_if vif;
    mailbox #(transaction) gen2drv, drv2mon;

    function new(mailbox#(transaction) gen2drv, mailbox#(transaction) drv2mon,
                 virtual cpu_if vif);
        this.gen2drv = gen2drv;
        this.drv2mon = drv2mon;
        this.vif     = vif;
    endfunction

    task reset();
        vif.reset = 1;
        vif.instr_code = 32'bx;  // Use NOP for reset
        repeat (3) @(posedge vif.clk);
        vif.reset = 0;
    endtask

    task run();
        forever begin
            transaction tr;
            gen2drv.get(tr);

            @(posedge vif.clk);
            vif.instr_code = tr.instr_code;

            $display("%0t [DRV] Driving instr=%0d (%s)", $time, tr.instr_code,
                     tr.get_instr_name());
            drv2mon.put(tr);
        end
    endtask
endclass

class monitor;
    virtual cpu_if vif;
    mailbox #(transaction) drv2mon, mon2scb;

    function new(mailbox#(transaction) drv2mon, mailbox#(transaction) mon2scb,
                 virtual cpu_if vif);
        this.drv2mon = drv2mon;
        this.mon2scb = mon2scb;
        this.vif     = vif;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            drv2mon.get(tr);
            @(posedge vif.clk);

            tr.rs1_val = tb_s_type.dut.U_DP.U_REGFILE.reg_file[tr.rs1];
            tr.rs2_val = tb_s_type.dut.U_DP.U_REGFILE.reg_file[tr.rs2];

            tr.calc_expected();

            $display("%0t [MON] Monitored: dAddr=%0d, dWdata=%0d", $time,
                     vif.dAddr, vif.dWdata);

            mon2scb.put(tr);
        end
    endtask
endclass

class scoreboard;
    virtual cpu_if vif;
    mailbox #(transaction) mon2scb;
    event gen_next_event;
    int pass_count, fail_count;

    function new(mailbox#(transaction) mon2scb, event gen_next_event,
                 virtual cpu_if vif);
        this.mon2scb = mon2scb;
        this.gen_next_event = gen_next_event;
        this.vif = vif;
    endfunction

    task run(int n);
        repeat (n) begin
            transaction tr;
            mon2scb.get(tr);

            #1ps;

            if (vif.d_wr_en) begin
                if (vif.dAddr === tr.exp_addr && vif.dWdata === tr.exp_dWdata && vif.store_type === tr.funct3) begin
                    pass_count++;
                    $display("SCB PASS: : Type match");
                end else begin
                    fail_count++;
                    $display("SCB FAIL: instr=%h (%s)", tr.instr_code,
                             tr.get_instr_name());
                    if (vif.dAddr !== tr.exp_addr)
                        $display(
                            "  ADDR MISMATCH: exp=0x%h got=0x%h",
                            tr.exp_addr,
                            vif.dAddr
                        );
                    if (vif.dWdata !== tr.exp_dWdata)
                        $display(
                            "  DATA MISMATCH: exp=0x%h got=0x%h",
                            tr.exp_dWdata,
                            vif.dWdata
                        );
                    if (vif.store_type !== tr.funct3)
                        $display(
                            "  TYPE MISMATCH: exp=%b got=%b",
                            tr.funct3,
                            vif.store_type
                        );
                end
            end else begin
                fail_count++;
                $display(
                    "SCB FAIL: instr=%h (%s) d_wr_en was not asserted by DUT",
                    tr.instr_code, tr.get_instr_name());
            end

            ->gen_next_event;
        end

        $display("------------------------------------------------");
        $display("SUMMARY :: TOTAL=%0d | PASS=%0d | FAIL=%0d",
                 pass_count + fail_count, pass_count, fail_count);
        $display("------------------------------------------------");
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
        // FIX: Added missing drv2mon argument to monitor's constructor
        mon = new(drv2mon, mon2scb, vif);
        scb = new(mon2scb, gen_next_event, vif);
    endfunction

    task run(int n = 5);
        drv.reset();
        fork
            gen.run(n);
            drv.run();
            mon.run(n);
            scb.run(n);
        join_any
        #2000;
        $stop;
    endtask
endclass

module tb_s_type;
    logic clk;
    cpu_if vif (clk);
    environment env;

    // Instantiate the top-level DUT
    RV32I_TOP dut (
        .clk(clk),
        .reset(vif.reset),
        .Instr_rdata(vif.instr_code)
    );

    assign vif.d_wr_en    = dut.U_CTRL.mem_write_en;
    assign vif.dAddr      = dut.U_DP.alu_result;
    assign vif.dWdata     = dut.U_DP.mem_wdata;
    assign vif.store_type = dut.U_CTRL.S_type_data;


    initial clk = 0;
    always #5 clk = ~clk;

    initial begin
        env = new(vif);
        env.run(500);
    end
endmodule

