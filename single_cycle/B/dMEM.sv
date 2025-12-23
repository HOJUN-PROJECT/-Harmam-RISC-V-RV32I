module dMEM (
    input  logic        clk,
    input  logic        reset,
    input  logic        mem_write_en,
    input  logic [ 2:0] s_type,        // SB/SH/SW
    input  logic [ 2:0] l_type,        // LB/LH/LW/LBU/LHU
    input  logic [31:0] mem_addr,
    input  logic [31:0] mem_wdata,
    output logic [31:0] mem_rdata
);

    logic [31:0] data_memory[0:31];
    logic [31:0] word_w;
    logic [31:0] word_r;

    initial begin
        for (int i = 0; i < 32; i++) data_memory[i] = i;
    end

    always_ff @(posedge clk) begin
        // if (reset) begin
        //     for (int k = 0; k < 32; k++) data_memory[k] <= 32'b0;
        // end else
        if (mem_write_en) begin
            word_w <= data_memory[mem_addr[4:2]];

            case (s_type)
                3'b000: begin  // SB
                    case (mem_addr[1:0])
                        2'b00: word_w[7:0] <= mem_wdata[7:0];
                        2'b01: word_w[15:8] <= mem_wdata[7:0];
                        2'b10: word_w[23:16] <= mem_wdata[7:0];
                        2'b11: word_w[31:24] <= mem_wdata[7:0];
                    endcase
                end

                3'b001: begin  // SH
                    if (mem_addr[1] == 1'b0) word_w[15:0] <= mem_wdata[15:0];
                    else word_w[31:16] <= mem_wdata[15:0];
                end

                3'b010: begin  // SW
                    word_w <= mem_wdata;
                end


            endcase

            data_memory[mem_addr[4:2]] <= word_w;
        end
    end

    // ------------------------
    // READ (L-type)
    // ------------------------
    always_comb begin
        word_r = data_memory[mem_addr[4:2]];
        mem_rdata = word_r;  // 기본 LW

        case (l_type)
            3'b000: begin  // LB
                case (mem_addr[1:0])
                    2'b00: mem_rdata = {{24{word_r[7]}}, word_r[7:0]};
                    2'b01: mem_rdata = {{24{word_r[15]}}, word_r[15:8]};
                    2'b10: mem_rdata = {{24{word_r[23]}}, word_r[23:16]};
                    2'b11: mem_rdata = {{24{word_r[31]}}, word_r[31:24]};
                endcase
            end

            3'b100: begin  // LBU
                case (mem_addr[1:0])
                    2'b00: mem_rdata = {24'b0, word_r[7:0]};
                    2'b01: mem_rdata = {24'b0, word_r[15:8]};
                    2'b10: mem_rdata = {24'b0, word_r[23:16]};
                    2'b11: mem_rdata = {24'b0, word_r[31:24]};
                endcase
            end

            3'b001: begin  // LH
                if (mem_addr[1] == 1'b0)
                    mem_rdata = {{16{word_r[15]}}, word_r[15:0]};
                else mem_rdata = {{16{word_r[31]}}, word_r[31:16]};
            end

            3'b101: begin  // LHU
                if (mem_addr[1] == 1'b0) mem_rdata = {16'b0, word_r[15:0]};
                else mem_rdata = {16'b0, word_r[31:16]};
            end

            3'b010: begin  // LW
                mem_rdata = word_r;
            end
            default: mem_rdata = 32'b0;

        endcase
    end

endmodule
