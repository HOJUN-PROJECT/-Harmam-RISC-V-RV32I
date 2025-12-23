module control (
    input  logic [31:0] Instr_rdata,
    output logic        register_write_en,
    output logic [ 3:0] alu_control_en,
    output logic        mem_write_en,
    output logic        imm_en,
    output logic [ 2:0] S_type_data,
    output logic [ 2:0] L_type_data,
    output logic [ 2:0] B_type_data,
    output logic [ 2:0] rd_mux_en,
    output logic        jump_en
);

    typedef enum logic [3:0] {
        ADD  = 4'b0000,
        SUB  = 4'b1000,
        AND  = 4'b0111,
        OR   = 4'b0110,
        XOR  = 4'b0100,
        SLL  = 4'b0001,
        SRL  = 4'b0101,
        SRA  = 4'b1101,
        SLT  = 4'b0010,
        SLTU = 4'b0011
    } R_type_opt;

    typedef enum logic [3:0] {
        ADDI  = 4'b0000,
        SLTI  = 4'b0010,
        SLTIU = 4'b0011,
        XORI  = 4'b0100,
        ORI   = 4'b0110,
        ANDI  = 4'b0111,
        SLLI  = 4'b0001,
        SRLI  = 4'b0101,
        SRAI  = 4'b1101
    } I_type_opt;

    typedef enum logic [2:0] {
        SB = 3'b000,
        SH = 3'b001,
        SW = 3'b010
    } S_type_opt;

    typedef enum logic [2:0] {
        LB  = 3'b000,
        LH  = 3'b001,
        LW  = 3'b010,
        LBU = 3'b100,
        LHU = 3'b101
    } L_type_opt;

    typedef enum logic [2:0] {
        BEQ  = 3'b000,
        BNE  = 3'b001,
        BLT  = 3'b100,
        BGE  = 3'b101,
        BLTU = 3'b110,
        BGEU = 3'b111
    } B_type_opt;

    logic [6:0] opcode;
    logic [2:0] func3;
    logic       func7_5;

    logic [3:0] R_type_data;
    logic [3:0] I_type_data;

    assign opcode  = Instr_rdata[6:0];
    assign func3   = Instr_rdata[14:12];
    assign func7_5 = Instr_rdata[30];

    always_comb begin
        register_write_en = 1'b0;
        imm_en            = 1'b0;
        alu_control_en    = 4'b0000;
        mem_write_en      = 1'b0;
        rd_mux_en         = 3'b000;
        S_type_data       = 3'bxxx;
        L_type_data       = 3'bxxx;
        jump_en           = 1'b0;

        case (opcode)
            7'b0110011: begin
                register_write_en = 1'b1;
                rd_mux_en = 3'b000;

                case ({
                    func7_5, func3
                })
                    4'b0_000: R_type_data = ADD;
                    4'b1_000: R_type_data = SUB;
                    4'b0_111: R_type_data = AND;
                    4'b0_110: R_type_data = OR;
                    4'b0_100: R_type_data = XOR;
                    4'b0_001: R_type_data = SLL;
                    4'b0_101: R_type_data = SRL;
                    4'b1_101: R_type_data = SRA;
                    4'b0_010: R_type_data = SLT;
                    4'b0_011: R_type_data = SLTU;
                    default:  R_type_data = ADD;
                endcase
                alu_control_en = R_type_data;
            end

            7'b0010011: begin
                register_write_en = 1'b1;
                imm_en            = 1'b1;
                rd_mux_en         = 3'b000;

                case ({
                    func7_5, func3
                })
                    4'b0_000: I_type_data = ADDI;
                    4'b0_010: I_type_data = SLTI;
                    4'b0_011: I_type_data = SLTIU;
                    4'b0_100: I_type_data = XORI;
                    4'b0_110: I_type_data = ORI;
                    4'b0_111: I_type_data = ANDI;
                    4'b0_001: I_type_data = SLLI;
                    4'b0_101: I_type_data = SRLI;
                    4'b1_101: I_type_data = SRAI;
                    default:  I_type_data = ADDI;
                endcase
                alu_control_en = I_type_data;
            end

            7'b0100011: begin
                mem_write_en      = 1'b1;
                imm_en            = 1'b1;
                register_write_en = 1'b0;
                rd_mux_en         = 3'bx;
                case (func3)
                    3'b000:  S_type_data = SB;
                    3'b001:  S_type_data = SH;
                    3'b010:  S_type_data = SW;
                    default: S_type_data = 3'bx;
                endcase
                alu_control_en = ADD;
            end

            7'b0000011: begin
                mem_write_en      = 1'b0;
                imm_en            = 1'b1;
                register_write_en = 1'b1;
                rd_mux_en         = 3'b001;
                case (func3)
                    3'b000:  L_type_data = LB;  // LB
                    3'b001:  L_type_data = LH;  // LH
                    3'b010:  L_type_data = LW;  // LW
                    3'b100:  L_type_data = LBU;  // LBU
                    3'b101:  L_type_data = LHU;  // LHU
                    default: L_type_data = 3'bx;
                endcase
                alu_control_en = ADD;
            end

            7'b1100011: begin
                imm_en            = 1'b0;
                register_write_en = 1'b0;
                rd_mux_en         = 2'bx;
                case (func3)
                    3'b000:  B_type_data = BEQ;  // BEQ
                    3'b001:  B_type_data = BNE;  // BNE
                    3'b100:  B_type_data = BLT;  // BLT
                    3'b101:  B_type_data = BGE;  // BGE
                    3'b110:  B_type_data = BLTU;  // BLTU
                    3'b111:  B_type_data = BGEU;  // BGEU
                    default: B_type_data = 3'bx;
                endcase
                alu_control_en = 4'bx;
            end

            7'b0110111: begin
                imm_en            = 1'b0;
                register_write_en = 1'b1;
                rd_mux_en         = 3'b010;
                alu_control_en    = 3'bx;
            end

            7'b0010111: begin
                imm_en            = 1'b0;
                register_write_en = 1'b1;
                rd_mux_en         = 3'b011;
                alu_control_en    = 3'bx;
            end

            7'b1101111: begin
                imm_en            = 1'b0;
                register_write_en = 1'b1;
                rd_mux_en         = 3'b100;
                jump_en           = 1'b1;
            end

            default: begin
                alu_control_en = 4'b0000;
            end
        endcase
    end

endmodule
