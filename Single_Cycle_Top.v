module PC_Module(clk,rst,PC,PC_Next);
    input clk,rst;
    input [31:0]PC_Next;
    output [31:0]PC;
    reg [31:0]PC;

    always @(posedge clk)
    begin
        if(~rst)
            PC <= {32{1'b0}};
        else
            PC <= PC_Next;
    end
endmodule

module Instruction_Memory(rst,A,RD);

  input rst;
  input [31:0]A;
  output [31:0]RD;

  reg [31:0] mem [1023:0];
  
  assign RD = (~rst) ? {32{1'b0}} : mem[A[31:2]];

  initial begin
    $readmemh("memfile.hex",mem);
  end


endmodule

module Register_File(clk,rst,WE3,WD3,A1,A2,A3,RD1,RD2);

    input clk,rst,WE3;
    input [4:0]A1,A2,A3;
    input [31:0]WD3;
    output [31:0]RD1,RD2;

    reg [31:0] Register [31:0];

    always @ (posedge clk)
    begin
        if(WE3)
            Register[A3] <= WD3;
    end

    assign RD1 = (~rst) ? 32'd0 : Register[A1];
    assign RD2 = (~rst) ? 32'd0 : Register[A2];

    initial begin
        Register[0] = 32'h00000000;         
        Register[2] = 32'h00000014;
    end

endmodule

module Sign_Extend (In,Imm_Ext,ImmSrc);

    input [31:0]In;
    input [1:0] ImmSrc;
    output [31:0]Imm_Ext;

    assign Imm_Ext = (ImmSrc == 2'b00) ? ({{20{In[31]}}, In[31:20]}):
                     (ImmSrc == 2'b01) ? ({{20{In[31]}}, In[31:25], In[11:7]} ):
                     (ImmSrc == 2'b10) ? ({{20{In[31]}}, In[7], In[30:25], In[11:8], 1'b0}):
                                         {{12{In[31]}}, In[19:12], In[20], In[30:21], 1'b0};
                                
endmodule

module ALU(A,B,Result,ALUControl,OverFlow,Carry,Zero,Negative);

    input [31:0]A,B;
    input [2:0]ALUControl;
    output Carry,OverFlow,Zero,Negative;
    output [31:0]Result;

    wire Cout;
    wire [31:0]Sum;

    assign {Cout,Sum} = (ALUControl[0] == 1'b0) ? A + B :
                                          (A + ((~B)+1)) ;
    assign Result = (ALUControl == 3'b000) ? Sum :
                    (ALUControl == 3'b001) ? Sum :
                    (ALUControl == 3'b010) ? A & B :
                    (ALUControl == 3'b011) ? A | B :
                    (ALUControl == 3'b101) ? {{31{1'b0}},(Sum[31])} : {32{1'b0}};
    
    assign OverFlow = ((Sum[31] ^ A[31]) & 
                      (~(ALUControl[0] ^ B[31] ^ A[31])) &
                      (~ALUControl[1]));
    assign Carry = ((~ALUControl[1]) & Cout);
    assign Zero = &(~Result);
    assign Negative = Result[31];

endmodule

module ALU_Decoder(ALUOp,funct3,funct7,op,ALUControl);

    input [1:0]ALUOp;
    input [2:0]funct3;
    input [6:0]funct7,op;
    output [2:0]ALUControl;
    
    assign ALUControl = (ALUOp == 2'b00) ? 3'b000 :
                        (ALUOp == 2'b01) ? 3'b001 :
                        ((ALUOp == 2'b10) & (funct3 == 3'b000) & ({op[5],funct7[5]} == 2'b11)) ? 3'b001 : 
                        ((ALUOp == 2'b10) & (funct3 == 3'b000) & ({op[5],funct7[5]} != 2'b11)) ? 3'b000 : 
                        ((ALUOp == 2'b10) & (funct3 == 3'b010)) ? 3'b101 : 
                        ((ALUOp == 2'b10) & (funct3 == 3'b110)) ? 3'b011 : 
                        ((ALUOp == 2'b10) & (funct3 == 3'b111)) ? 3'b010 : 
                                                                  3'b000 ;
endmodule

module Main_Decoder(Op,RegWrite,ImmSrc,ALUSrc,MemWrite,ResultSrc,Branch,ALUOp,jump);
    input [6:0]Op;
    output RegWrite,ALUSrc,MemWrite,Branch,jump;
    output [1:0]ImmSrc,ResultSrc,ALUOp;

    assign RegWrite = (Op == 7'b0000011 | Op == 7'b0110011 | Op == 7'b0010011 | Op == 7'b1101111 ) ? 1'b1 :
                                                              1'b0 ;
    assign ImmSrc = (Op == 7'b0100011) ? 2'b01 : 
                    (Op == 7'b1100011) ? 2'b10 : 
                    (Op == 7'b1101111) ? 2'b11 :   
                                         2'b00 ;
    assign ALUSrc = (Op == 7'b0110011 | Op == 7'b1100011) ? 1'b0 :
                                                            1'b1 ;
    assign MemWrite = (Op == 7'b0100011) ? 1'b1 :
                                           1'b0 ;
    assign ResultSrc = (Op == 7'b0000011) ? 2'b01 :
                         (Op == 7'b1101111) ? 2'b10 :
                                            2'b00 ;
    assign Branch = (Op == 7'b1100011) ? 1'b1 :
                                         1'b0 ;
    assign ALUOp = (Op == 7'b0110011 | Op == 7'b0010011) ? 2'b10 :
                   (Op == 7'b1100011) ? 2'b01 :
                                        2'b00 ;
    assign jump = (Op == 7'b1101111) ? 1'b1:
                                       1'b0;

endmodule

module Control_Unit_Top(Op,pcSRC,RegWrite,ImmSrc,ALUSrc,MemWrite,ResultSrc,funct3,funct7,zero,ALUControl);

    input [6:0]Op,funct7;
    input [2:0]funct3;
    input zero;
    wire Branch,jump;
    output RegWrite,ALUSrc,MemWrite,pcSRC;
    output [1:0]ImmSrc,ResultSrc;
    output [2:0]ALUControl;

    wire [1:0]ALUOp;

    Main_Decoder Main_Decoder(
                .Op(Op),
                .RegWrite(RegWrite),
                .ImmSrc(ImmSrc),
                .MemWrite(MemWrite),
                .ResultSrc(ResultSrc),
                .Branch(Branch),
                .ALUSrc(ALUSrc),
                .ALUOp(ALUOp),
                .jump(jump)
    );

    assign pcSRC= (Branch & zero) | jump;

    ALU_Decoder ALU_Decoder(
                            .ALUOp(ALUOp),
                            .funct3(funct3),
                            .funct7(funct7),
                            .op(Op),
                            .ALUControl(ALUControl)
    );


endmodule

module Data_Memory(clk,rst,WE,WD,A,RD);

    input clk,rst,WE;
    input [31:0]A,WD;
    output [31:0]RD;

    reg [31:0] mem [1023:0];

    always @ (posedge clk)
    begin
        if(WE)
            mem[A] <= WD;
    end

    assign RD = (~rst) ? 32'd0 : mem[A];

    initial begin
        mem[0] = 32'h00000003; //3
        mem[1] = 32'h00000007; //7
        mem[2] = 32'h00000002; //2
        mem[3] = 32'h00000006; //6
        mem[4] = 32'h00000005; //5
        mem[5] = 32'h00000004; //4
        mem[6] = 32'h00000001; //1
        mem[7] = 32'h000003e8; //1000
        mem[8] = 32'h000003e7; //999
        mem[9] = 32'h00000019; //25
        mem[10] = 32'h0000005a; //90
        mem[11] = 32'h00000064; //100   
        mem[12] = 32'h0000001e; //30
        mem[13] = 32'h00000014; //20
        mem[14] = 32'h0000000A; //10
        mem[15] = 32'h000000c8; //200
        mem[16] = 32'h00000ce4; //3300
        mem[17] = 32'h000000fa; //250
        mem[18] = 32'h0000000c; //12
        mem[19] = 32'h0000004b; //75
        /*---------------------------------------------------------------------------------------------------------------*/
        mem[20] = 32'h00000011; //17
        mem[21] = 32'h0000000d; //13
        mem[22] = 32'h00000012; //18
        mem[23] = 32'h0000000e; //14
        mem[24] = 32'h0000000f; //15
        mem[25] = 32'h00000010; //16
        mem[26] = 32'h00000013; //19
        mem[27] = 32'h00000001; //1
        mem[28] = 32'h00000002; //2
        mem[29] = 32'h00000009; //9
        mem[30] = 32'h00000006; //6
        mem[31] = 32'h00000005; //5
        mem[32] = 32'h00000008; //8
        mem[33] = 32'h0000000a; //10
        mem[34] = 32'h0000000c; //12
        mem[35] = 32'h00000004; //4
        mem[36] = 32'h00000000; //0
        mem[37] = 32'h00000003; //3
        mem[38] = 32'h0000000b; //11
        mem[39] = 32'h00000007; //7
    end


endmodule

module PC_Adder (a,b,c);

    input [31:0]a,b;
    output [31:0]c;

    assign c = a + b;
    
endmodule

module Mux (a,b,s,c);

    input [31:0]a,b;
    input s;
    output [31:0]c;

    assign c = (~s) ? a : b ;
    
endmodule

module Mux4 (a,b,d,f,s,c);

    input [31:0]a,b,d,f;
    input [1:0]s;
    output [31:0]c;
    
    assign c = (s==2'b00) ? a :
               (s==2'b01) ? b:
                (s==2'b10) ? d :
                f;
    
endmodule


module Single_Cycle_Top(clk,rst);

    input clk,rst;

    wire [31:0] PC_Top,RD_Instr,RD1_Top,Imm_Ext_Top,ALUResult,ReadData,PCPlus4,RD2_Top,SrcB,Result,pcTarget,PC_Next;
    wire RegWrite,MemWrite,ALUSrc,pcSRC;
    wire [1:0]ImmSrc , ResultSrc;
    wire [2:0]ALUControl_Top; 

    Mux Mux_pcSRC(
        .a(PCPlus4),
        .b(pcTarget),
        .s(pcSRC),
        .c(PC_Next)
    );

    PC_Module PC(
        .clk(clk),
        .rst(rst),
        .PC(PC_Top),
        .PC_Next(PC_Next)
    );

    PC_Adder PC_Adder(
                    .a(PC_Top),
                    .b(32'd4),
                    .c(PCPlus4)
    );

    PC_Adder adder_pcTarget(
        .a(PC_Top),
        .b(Imm_Ext_Top),
        .c(pcTarget)
    );
    
    Instruction_Memory Instruction_Memory(
                            .rst(rst),
                            .A(PC_Top),
                            .RD(RD_Instr)
    );

    Register_File Register_File(
                            .clk(clk),
                            .rst(rst),
                            .WE3(RegWrite),
                            .WD3(Result),
                            .A1(RD_Instr[19:15]),
                            .A2(RD_Instr[24:20]),
                            .A3(RD_Instr[11:7]),
                            .RD1(RD1_Top),
                            .RD2(RD2_Top)
    );

    Sign_Extend Sign_Extend(
                        .In(RD_Instr),
                        .ImmSrc(ImmSrc),
                        .Imm_Ext(Imm_Ext_Top)
    );

    Mux Mux_Register_to_ALU(
                            .a(RD2_Top),
                            .b(Imm_Ext_Top),
                            .s(ALUSrc),
                            .c(SrcB)
    );

    ALU ALU(
            .A(RD1_Top),
            .B(SrcB),
            .Result(ALUResult),
            .ALUControl(ALUControl_Top),
            .OverFlow(),
            .Carry(),
            .Zero(zero),
            .Negative()
    );

    Control_Unit_Top Control_Unit_Top(
                            .Op(RD_Instr[6:0]),
                            .RegWrite(RegWrite),
                            .ImmSrc(ImmSrc),
                            .ALUSrc(ALUSrc),
                            .MemWrite(MemWrite),
                            .pcSRC(pcSRC),
                            .ResultSrc(ResultSrc),
                            .zero(zero),
                            .funct3(RD_Instr[14:12]),
                            .funct7(RD_Instr[31:25]),
                            .ALUControl(ALUControl_Top)
    );

    Data_Memory Data_Memory(
                        .clk(clk),
                        .rst(rst),
                        .WE(MemWrite),
                        .WD(RD2_Top),
                        .A(ALUResult),
                        .RD(ReadData)
    );

    Mux4 Mux_DataMemory_to_Register(
                            .a(ALUResult),
                            .b(ReadData),
                            .d(PCPlus4),
                            .f(),
                            .s(ResultSrc),
                            .c(Result)
    );

endmodule

//iverilog -o single_cycle_top_tb.vvp single_cycle_top_tb.v
//vvp single_cycle_top_tb.vvp
//gtkwave single_cycle_top_tb.vcd