module control (
    input  logic [31:0] Instr_rdata,
    output logic        register_write_en,
    output logic [3:0]  alu_control_en
);


    typedef enum logic [3:0] {
        ADD  = 4'b0000,
        SUB  = 4'b1000,
        AND = 4'b0111,
        OR  = 4'b0110,
        XOR = 4'b0100,
        SLL  = 4'b0001,
        SRL  = 4'b0101,
        SRA  = 4'b1101,
        SLT  = 4'b0010,
        SLTU = 4'b0011
    } alu_op_t;


    logic [6:0] opcode;
    logic [2:0] func3;
    logic func7_5;
    alu_op_t sim_alu_control_en; 

    assign alu_control_en = sim_alu_control_en;
    assign opcode  = Instr_rdata[6:0];
    assign func3   = Instr_rdata[14:12];
    assign func7_5 = Instr_rdata[30];


    always_comb begin
        register_write_en = 1'b0;
        alu_control_en    = 4'd0;

        case (opcode)
            7'b0110011: begin  // R-type
                register_write_en = 1'b1;

                case ({func7_5, func3})
                    4'b0_000: sim_alu_control_en = ADD;
                    4'b1_000: sim_alu_control_en = SUB;
                    4'b0_111: sim_alu_control_en = AND;
                    4'b0_110: sim_alu_control_en = OR;
                    4'b0_100: sim_alu_control_en = XOR;
                    4'b0_001: sim_alu_control_en = SLL;
                    4'b0_101: sim_alu_control_en = SRL;
                    4'b1_101: sim_alu_control_en = SRA;
                    4'b0_010: sim_alu_control_en = SLT;
                    4'b0_011: sim_alu_control_en = SLTU;
                endcase
            end
        endcase
    end

endmodule
