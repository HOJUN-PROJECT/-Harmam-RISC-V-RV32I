module dMEM (
    input  logic        clk,
    input  logic        rst,
    input  logic        mem_write_en,
    input  logic [2:0]  s_type,       // func3
    input  logic [31:0] mem_addr,  
    input  logic [31:0] mem_wdata,   
    output logic [31:0] mem_rdata    
);

    logic [31:0] data_memory [0:7];
    logic [31:0] word;

    integer i;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 8; i = i + 1)
                data_memory[i] <= 32'b0;
            word <= 32'b0;
        end
        else if (mem_write_en) begin
            word = data_memory[mem_addr[4:2]];
            case (s_type)
                3'b000: begin
                    case (mem_addr[1:0])
                        2'b00: word[7:0]   = mem_wdata[7:0];
                        2'b01: word[15:8]  = mem_wdata[7:0];
                        2'b10: word[23:16] = mem_wdata[7:0];
                        2'b11: word[31:24] = mem_wdata[7:0];
                    endcase
                end
                3'b001: begin
                    if (mem_addr[1] == 1'b0)
                        word[15:0]  = mem_wdata[15:0];
                    else
                        word[31:16] = mem_wdata[15:0];
                end
                3'b010: begin
                    word = mem_wdata;
                end
            endcase
            data_memory[mem_addr[4:2]] <= word;
        end
    end

    assign mem_rdata = data_memory[mem_addr[4:2]];

endmodule
