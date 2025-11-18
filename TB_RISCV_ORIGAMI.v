module TB_RISCV_ORIGAMI;
    reg clock;
    reg reset;
    wire [31:0] result;
    wire zero;

    RISCV_ORIGAMI_CORE soc (
        .clock(clock),
        .reset(reset),
        .debug_alu_result(result),
        .debug_zero(zero)
    );

    initial begin
        $dumpfile("origami_cpu.vcd");
        $dumpvars(0, TB_RISCV_ORIGAMI);
        
        clock = 0;
        reset = 1;
        
        #20 
        reset = 0; 
        
        #200 
        $finish;
    end

    always #10 clock = ~clock;

    initial begin
        $monitor("Time:%4t | PC:%8h | Instr:%8h | ALU_Out:%d | Zero:%b | Gated_W:%b", 
                 $time, 
                 soc.fetch_unit.pc, 
                 soc.fetch_unit.instr_out, 
                 result, 
                 zero,
                 soc.rf_unit.regwrite_en
                 );
    end
endmodule
