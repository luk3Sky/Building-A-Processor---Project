/*
	 ________________CPU________________
	         E/15/154 | E/15/142
      _____________________________
              CO224-Lab5
*/

//ALU Module
module alu(RESULT,DATA1,DATA2,SELECT);
    output reg [7:0] RESULT;
    input [7:0] DATA1,DATA2;
    input [2:0] SELECT;

    always @(DATA1,DATA2,SELECT)
    begin
        case (SELECT)
            //FORWARD INSTRUCTION
            3'b000:
                begin
                    RESULT = DATA1;
                end
            //ADD INSTRUCTION
            3'b001:
                begin
                    RESULT = DATA1 + DATA2;
                end
            //AND INSTRUCTION
            3'b010:
                begin
                    RESULT = DATA1 & DATA2;
                end
            //OR INSTRUCTION
            3'b011:
                begin
                    RESULT = DATA1 | DATA2;
                end
            //RESET INSTRUCTION
            default:
                    RESULT = 8'b00000000;
        endcase
    end

endmodule //ALU


// Register File
module regfile8x8a ( clk, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2);

	input [2:0] OUT1addr,OUT2addr,INaddr;
	input [7:0] IN;
	input clk;
	output [7:0] OUT1,OUT2;

	reg [63:0] regMemory = 0;
	reg [7:0] OUT1reg, OUT2reg;
	integer i;

	assign	OUT1 = OUT1reg[7:0];
	assign	OUT2 = OUT2reg[7:0];

	always @(posedge clk) begin
		for(i = 0 ; i < 8 ; i = i + 1) begin
			OUT1reg[i] = regMemory[ OUT1addr*8 + i ];
			OUT2reg[i] = regMemory[ OUT2addr*8 + i ];
		end
	end

	always @(negedge clk) begin
		for(i = 0 ; i < 8 ; i = i + 1 ) begin
			regMemory[INaddr*8 + i] = IN[i];
		end
	end

endmodule //regfile8x8a

//2's Complement Module
module two_s_complement (OUT,IN);
  input signed [7:0] IN;
  output signed [7:0] OUT;

  // input is inverted and added value 1
  assign OUT = ~IN + 8'b00000001;

endmodule //two_s_complement

//2:1 Multiplexer Module
module mux (OUT,IN1,IN2,SELECT);
  input [7:0] IN1,IN2;
  input SELECT;
  output reg [7:0] OUT;

  always @ (IN1,IN2,SELECT) begin
    case (SELECT)
      0: begin OUT <= IN1; end
      1: begin OUT <= IN2; end
      default: OUT <= 0;
    endcase
  end

endmodule //MUX

//Program Counter Module
module counter(clk, reset, Read_addr );
	input clk;
	input reset;
	output [31:0] Read_addr;
	reg Read_addr;

	always @(negedge clk)
	begin
		case(reset)
			1'b1 : begin Read_addr = 32'd0; end
			1'b0 : begin Read_addr = Read_addr + 3'b100; end
		endcase
	end

endmodule //Counter


//Control Unit Module
module CU (instruction, OUT1addr, OUT2addr, INaddr, Im_val, select, as_MUX, im_MUX);
    input [31:0] instruction;
    output [2:0] OUT1addr;
    output [2:0] OUT2addr;
    output [2:0] select;
    output [2:0] INaddr;
    output [7:0] Im_val;
    output as_MUX,im_MUX;

    reg [2:0] OUT1addr,OUT2addr,INaddr,select;
    reg [7:0] Im_val;
    reg as_MUX,im_MUX;

    always @(instruction)
      begin

        select = instruction[26:24];
        Im_val = instruction[7:0];
        OUT1addr = instruction[2:0];
        OUT2addr = instruction[10:8];
        INaddr = instruction[18:16];
        im_MUX = 1'b1;
        as_MUX = 1'b0;

        case(instruction[31:24])

            //loadi
            8'b00000000 : begin
              assign im_MUX = 1'b0;
              end

            //sub
            8'b00001001 : begin
              assign as_MUX = 1'b1;
              end

        endcase
      end

endmodule // CU

//IR - INSTRUCTION REGISTER
module Instruction_reg (clk,Read_Addr,instruction);
  input clk;
  input [31:0] Read_Addr;
  output [31:0] instruction;
  reg instruction;

  always @(negedge clk)
  begin
    instruction = Read_Addr;
  end

endmodule // Instruction_reg

module Processor( Read_Addr, Result, clk );

	input [31:0] Read_Addr;
	input clk;
	output [7:0] Result;
	wire [7:0] Result;

	wire [31:0] instruction;
	wire [2:0] OUT1addr,OUT2addr,INaddr,Select;
	wire  [7:0] Imm,OUT1,OUT2,OUTPUT,INPUT,cmp;
	wire [7:0] imValueMUXout, addSubMUXout;
	wire addSubMUX, imValueMUX;

    // ----- Set up modules -----
    //Instruction Regiter
	Instruction_reg ir1(clk, Read_Addr, instruction);
    //Control Unit
    CU cu1( instruction, OUT1addr, OUT2addr, INaddr, Imm, Select, addSubMUX, imValueMUX );
    //Register File
    regfile8x8a rf1( clk, INaddr, Result, OUT1addr, OUT1, OUT2addr, OUT2 );
    //2'sComplement
    two_s_complement tcomp( OUTPUT, OUT1 );
    //2's complement MUX
    mux addsubMUX( addSubMUXout, OUT1, OUTPUT, addSubMUX );
    //Imediate Value MUX
    mux immValMUX( imValueMUXout, Imm, addSubMUXout, imValueMUX );
	//ALU
    alu alu1( Result, imValueMUXout, OUT2, Select );

endmodule //Processor

// Testbench Module
module test;
	reg [31:0] Read_Addr;
	wire [7:0] Result;
	reg clk;

    // Processor module instance 
	Processor simpleP( Read_Addr, Result, clk );

	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end

	initial begin

	// ---- Operation set 1 ----
	$display("\nOperation      Binary   | Decimal");
	$display("---------------------------------");

        //loadi 4,X,0xFF
    	Read_Addr = 32'b0000000000000100xxxxxxxx11111111;
	#20
		$display("load r4        %b | %d",Result,Result);

        //loadi 6,X,0xAA
		Read_Addr = 32'b0000000000000110xxxxxxxx10101010;
	#20
		$display("load r6        %b | %d",Result,Result);

        //loadi 3,X,0xBB
		Read_Addr = 32'b0000000000000011xxxxxxxx10111011;
	#20
		$display("load r3        %b | %d",Result,Result);

        //add 5,6,3
		Read_Addr = 32'b00000001000001010000011000000011;
	#20
		$display("add r5 (r6+r3) %b | %d",Result,Result);

        //and 1,4,5
		Read_Addr = 32'b00000010000000010000010000000101;
	#20
		$display("and r1 (r4,r5) %b | %d",Result,Result);

        //or 2,1,6
		Read_Addr = 32'b00000011000000100000000100000110;
	#20
		$display("or r2 (r1,r6)  %b | %d",Result,Result);

        //mov 7,X,2
		Read_Addr = 32'b0000100000001111xxxxxxxx00000010;
	#20
		$display("copy r7 (r2)   %b | %d",Result,Result);

        //sub 4,7,3
		Read_Addr = 32'b00001001000001000000111100000011;
	#20
		$display("sub r4 (r7-r3) %b | %d",Result,Result);


	// ---- Operation set 2 ----
	$display("\nOperation      Binary   | Decimal");
		$display("---------------------------------");

        //loadi 4,X,0xFF
		Read_Addr = 32'b0000000000000100xxxxxxxx00001101;
	#20
		$display("load r4        %b | %d",Result,Result);

        //loadi 6,X,0xAA
		Read_Addr = 32'b0000000000000110xxxxxxxx00101101;
	#20
		$display("load r6        %b | %d",Result,Result);

        //loadi 3,X,0xBB
		Read_Addr = 32'b0000000000000011xxxxxxxx00100001;
	#20
	$display("load r3        %b | %d",Result,Result);

        //add 5,6,3
		Read_Addr = 32'b00000001000001010000011000000011;
	#20
		$display("add r5 (r3+r6) %b | %d",Result,Result);

        //and 1,4,5
		Read_Addr = 32'b00000010000000010000010000000101;
	#20
		$display("and r1 (r4,r5) %b | %d",Result,Result);

        //or 2,1,6
		Read_Addr = 32'b00000011000000100000000100000110;
	#20
		$display("or r2 (r1,r6)  %b | %d",Result,Result);

        //mov 7,X,2
		Read_Addr = 32'b0000100000001111xxxxxxxx00000010;
	#20
		$display("move r7 (r2)   %b | %d",Result,Result);

        //sub 4,7,3
		Read_Addr = 32'b00001001000001000000111100000011;
	#20
		$display("sub r4 (r7-r3) %b | %d",Result,Result);

		$finish;
	end
endmodule
