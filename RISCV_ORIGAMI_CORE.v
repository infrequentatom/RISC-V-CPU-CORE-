module CLOCK_GATING_CELL (
    input wire clk_in,
    input wire enable,
    output wire clk_out
);
    reg latch_en;
    
    always @(clk_in or enable) begin
        if (!clk_in) 
            latch_en <= enable;
    end
    
    assign clk_out = clk_in & latch_en;
endmodule

module ALU_FOLDED (
    input [31:0] in1, in2,
    input [3:0] alu_control,
    output reg [31:0] result,
    output zero_flag
);
    wire [31:0] operand_b_inv;
    wire [31:0] adder_out;
    wire carry_in;
    
    assign carry_in = (alu_control == 4'b0100 || alu_control == 4'b1000) ? 1'b1 : 1'b0;
    assign operand_b_inv = (carry_in) ? ~in2 : in2;
    
    assign adder_out = in1 + operand_b_inv + carry_in;

    always @(*) begin
        case(alu_control)
            4'b0000: result = in1 & in2;
            4'b0001: result = in1 | in2;
            4'b0010: result = adder_out;
            4'b0100: result = adder_out;
            4'b1000: result = {31'b0, adder_out[31]};
            4'b0011: result = in1 << in2[4:0];
            4'b0101: result = in1 >> in2[4:0];
            4'b0110: result = in1 * in2;
            4'b0111: result = in1 ^ in2;
            default: result = 32'b0;
        endcase
    end

    assign zero_flag = (result == 32'b0);

endmodule

module REG_FILE_GATED(
    input [4:0] rs1, rs2, rd,
    input [31:0] write_data,
    output [31:0] rdata1, rdata2,
    input regwrite_en,
    input sys_clock,
    input reset
);
    reg [31:0] registers [31:0];
    wire gated_clock;
    integer i;

    CLOCK_GATING_CELL icg_inst (
        .clk_in(sys_clock),
        .enable(regwrite_en),
        .clk_out(gated_clock)
    );

    assign rdata1 = (rs1 == 0) ? 32'b0 : registers[rs1];
    assign rdata2 = (rs2 == 0) ? 32'b0 : registers[rs2];

    always @(posedge gated_clock or posedge reset) begin
        if (reset) begin
            for(i=0; i<32; i=i+1) registers[i] <= i; 
        end
        else begin
            if(rd != 0) 
                registers[rd] <= write_data;
        end
    end
endmodule

module CONTROL_UNIT(
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] alu_ctrl,
    output reg regwrite_en
);
    always @(*) begin
        regwrite_en = 0;
        alu_ctrl = 4'b0000;

        if(opcode == 7'b0110011) begin 
            regwrite_en = 1;
            case(funct3)
                3'b000: alu_ctrl = (funct7[5]) ? 4'b0100 : 4'b0010;
                3'b110: alu_ctrl = 4'b0001;
                3'b111: alu_ctrl = 4'b0000;
                3'b001: alu_ctrl = 4'b0011;
                3'b101: alu_ctrl = 4'b0101;
                3'b010: alu_ctrl = 4'b1000;
                3'b100: alu_ctrl = 4'b0111;
                default: alu_ctrl = 4'b0000;
            endcase
        end
    end
endmodule

module INST_MEM(
    input [31:0] pc,
    input reset,
    output [31:0] instr
);
    reg [7:0] mem [63:0];
    integer k;

    assign instr = {mem[pc+3], mem[pc+2], mem[pc+1], mem[pc]};

    always @(posedge reset) begin
        if(reset) begin
            for(k=0; k<64; k=k+1) mem[k] = 8'b0;
            {mem[3], mem[2], mem[1], mem[0]} = 32'h00940333; 
            {mem[7], mem[6], mem[5], mem[4]} = 32'h409303B3;
        end
    end
endmodule

module IFU(
    input clock, reset,
    output [31:0] instr_out
);
    reg [31:0] pc;
    INST_MEM imem(.pc(pc), .reset(reset), .instr(instr_out));

    always @(posedge clock or posedge reset) begin
        if(reset) pc <= 0;
        else pc <= pc + 4;
    end
endmodule

module RISCV_ORIGAMI_CORE(
    input clock,
    input reset,
    output [31:0] debug_alu_result,
    output debug_zero
);
    wire [31:0] instruction;
    wire [3:0] alu_control_sig;
    wire regwrite_sig;
    wire [31:0] rdata1, rdata2, alu_res;

    IFU fetch_unit (
        .clock(clock), 
        .reset(reset), 
        .instr_out(instruction)
    );

    CONTROL_UNIT ctrl_unit (
        .opcode(instruction[6:0]),
        .funct3(instruction[14:12]),
        .funct7(instruction[31:25]),
        .alu_ctrl(alu_control_sig),
        .regwrite_en(regwrite_sig)
    );

    REG_FILE_GATED rf_unit (
        .rs1(instruction[19:15]),
        .rs2(instruction[24:20]),
        .rd(instruction[11:7]),
        .write_data(alu_res),
        .rdata1(rdata1),
        .rdata2(rdata2),
        .regwrite_en(regwrite_sig), 
        .sys_clock(clock),
        .reset(reset)
    );

    ALU_FOLDED ex_unit (
        .in1(rdata1),
        .in2(rdata2),
        .alu_control(alu_control_sig),
        .result(alu_res),
        .zero_flag(debug_zero)
    );

    assign debug_alu_result = alu_res;

endmodule
