/*
	 ________________CPU________________
	         E/15/154 | E/15/142
      _____________________________
              CO224-Lab5
*/

//alu module
module alu(RESULT,DATA1,DATA2,SELECT);
    output reg [7:0] RESULT;
    input [7:0] DATA1,DATA2;
    input [2:0] SELECT;

    always @(DATA1,DATA2,SELECT)
    begin
        case (SELECT)
            3'b000:
                begin
                    RESULT = DATA1;         //FORWARD
                end
            3'b001:
                begin
                    RESULT = DATA1 + DATA2; //ADD
                end
            3'b010:
                begin
                    RESULT = DATA1 & DATA2; //AND
                end
            3'b011:
                begin
                    RESULT = DATA1 | DATA2; //OR
                end
            3'b100:
                begin
                  RESULT = DATA1;
                end
            3'b101:
              begin
                RESULT = DATA1;
              end
            default:
                    RESULT = 8'b00000000;
        endcase
    end

endmodule


// Register File
module regfile8x8a ( clk, busy_wait, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2);

  input busy_wait;
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
		for(i=0;i<8;i=i+1) begin
			OUT1reg[i] = regMemory[ OUT1addr*8 + i ];
			OUT2reg[i] = regMemory[ OUT2addr*8 + i ];
		end
	end


	always @(negedge clk) begin
		for(i=0;i<8;i=i+1)begin
			regMemory[INaddr*8 + i] = IN[i];
		end
	end

  always @(negedge clk) //new
  begin
	if (!busy_wait)
  begin
	case (INaddr)
		3'b000:
    begin
      regMemory[INaddr*8 + 0] = IN[0];
    end
		3'b001:
    begin
      regMemory[INaddr*8 + 1] = IN[1];
    end
		3'b010:
    begin
      regMemory[INaddr*8 + 2] = IN[2];
    end
		3'b011:
    begin
      regMemory[INaddr*8 + 3] = IN[3];
    end
		3'b100:
    begin
      regMemory[INaddr*8 + 4] = IN[4];
    end
		3'b101:
    begin
      regMemory[INaddr*8 + 5] = IN[5];
    end
		3'b110:
    begin
      regMemory[INaddr*8 + 6] = IN[6];
    end
		3'b111:
    begin
      regMemory[INaddr*8 + 7] = IN[7];
    end
	endcase
	end
end

endmodule

//2's complement
module two_s_complement (OUT,IN);
  input signed [7:0] IN;
  output signed [7:0] OUT;

  assign OUT = ~IN + 8'b00000001;

endmodule //two_s_complement

//2:1 Multiplexer
module mux (OUT,IN1,IN2,SELECT);
  input [7:0] IN1,IN2;
  input SELECT;
  output reg [7:0] OUT;

  always @ (IN1,IN2,SELECT) begin
    case (SELECT)
      0: begin OUT<=IN1; end
      1: begin OUT<=IN2; end
      default: OUT<=0;
    endcase
  end

endmodule //mux 2 to 1

//Program Counter
module counter(clk, reset, busy_wait, Read_addr );
	input clk;
	input reset;
  input busy_wait;
	output [3:0] Read_addr;

  reg [3:0] Read_addr = 4'b0000;

  always @(posedge clk)
  begin
  	if(~reset && !busy_wait)
  		begin

  			Read_addr <= Read_addr + 1'b1;
  		end

  	else if (busy_wait) begin
  		Read_addr <= Read_addr ;
  	end
  	else begin
  		Read_addr <= 4'b0000;
  	end

  end
endmodule


//Control Unit
module CU (busy_wait, instruction, OUT1addr, OUT2addr, INaddr, Im_val, select, as_MUX, im_MUX, memRead, memWrite, regWrite,address);
input busy_wait;
input [31:0] instruction;
output [2:0] OUT1addr;
output [2:0] OUT2addr;
output [2:0] select;
output [2:0] INaddr;
output [7:0] Im_val;
output memRead;
output memWrite;
output regWrite;
output as_MUX,im_MUX;
output address;

reg [2:0] OUT1addr,OUT2addr,INaddr,select;
reg [7:0] Im_val;
reg as_MUX,im_MUX;
reg memRead,regWrite,memWrite,address;


always @(instruction)
  begin
    memRead = 1'b0;
    memWrite = 1'b0;
    assign select = instruction[26:24];
    assign Im_val = instruction[7:0];
    assign OUT1addr = instruction[2:0];
    assign OUT2addr = instruction[10:8];
    assign INaddr = instruction[18:16];
    assign im_MUX = 1'b1;
    assign as_MUX = 1'b0;

  //   case(instruction[31:24])
  //
  //   8'b00000000 : begin  //loadi
  //     assign im_MUX = 1'b0;
  //     end
  //
  //   8'b00001001 : begin //sub
  //     assign as_MUX = 1'b1;
  //     end
  //
  //   endcase
  // end

  case (instruction[31:24])
		8'b00000100:
			// load	from memory
			begin
				memWrite = 1'b0;
				memRead = 1'b1;
				address = instruction[7:0];
				$display("oper = load");
			end

		8'b00000101:
			begin
				memRead = 1'b0;
				memWrite = 1'b1;
	 			address = instruction[23:16];
				$display("oper = store");
			end
			// store to memory
		8'b00001000:
			// loadi
			begin
				assign im_MUX = 1'b0;
				$display("oper = loadImmediate");
			end

		8'b00001001:
			// sub
			// use the 2's comp
			assign as_MUX = 1'b1;
			//$display("oper = SUB");
		default :
		begin
			memWrite = 1'b0;
			memRead = 1'b0;
		end



	endcase

  end

endmodule // CU

//IR
module Instruction_reg (clk,Read_Addr,instruction);
  input clk;
  input [31:0] Read_Addr;
  output [31:0] instruction;
  reg instruction;

  always @(negedge clk)
  begin
    instruction = Read_Addr;
  end

endmodule // Instruction Regiter

//Data Memory
module data_mem(
    clk,
    rst,
    read,
    write,
    address,
    write_data,
    read_data,
	busy_wait
);
input           clk;
input           rst;
input           read;
input           write;
input[7:0]      address;
input[7:0]      write_data;
output[7:0]     read_data;
output			busy_wait;

reg[7:0]     read_data;
reg busy_wait,clkMem=1'b0;

integer  i;

// Declare memory 256x8 bits
reg [7:0] memory_array [255:0];
//reg [7:0] memory_ram_q [255:0];



always @(posedge rst)
begin
    if (rst)
    begin
        for (i=0;i<256; i=i+1)
            memory_array[i] <= 0;
    end
end


always #1 clkMem = ~clkMem;

always @(posedge clkMem)
begin
    if (write && !read && !busy_wait)
	begin
		busy_wait <= 1;
		// artificially delay 100 cycles
		repeat(10)
		begin
		@(posedge clk);
        end
        $display("writing to memory");
        memory_array[address] = write_data;
		busy_wait <= 0;
	end
    if (!write && read && !busy_wait)
	begin
		busy_wait <= 1;
		// artificially delay 100 cycles
        repeat(10)
		begin
		@(posedge clk);
        end
        $display("reading from memory");
        read_data = memory_array[address];
		busy_wait <= 0;
	end
end

endmodule


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

	Instruction_reg ir1(clk, Read_Addr, instruction);	//Instruction Regiter
	CU cu1( busy_wait, instruction, OUT1addr, OUT2addr, INaddr, Im_val, select, as_MUX, im_MUX, memRead, memWrite, regWrite,address );	//Control Unit
	regfile8x8a rf1( clk, busy_wait, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2 );	//Register File
	two_s_complement tcomp( OUTPUT, OUT1 );		//2'sComplement
	mux addsubMUX( addSubMUXout, OUT1, OUTPUT, addSubMUX );		//2's complement MUX
	mux immValMUX( imValueMUXout, Imm, addSubMUXout, imValueMUX );	//Imediate Value MUX
	alu alu1( Result, imValueMUXout, OUT2, Select );	//ALU
  data_mem dm(clk,rst,read,write,address,write_data,read_data,busy_wait); //Data Memory

endmodule


module test;

	reg [31:0] Read_Addr;
	wire [7:0] Result;
	reg clk;
	Processor simpleP( Read_Addr, Result, clk );

	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end

	initial begin

	// Operation set 1
	$display("\nOperation      Binary   | Decimal");
	$display("---------------------------------");
		Read_Addr = 32'b0000000000000100xxxxxxxx11111111;//loadi 4,X,0xFF
	#40
		$display("load r4        %b | %d",Result,Result);

		Read_Addr = 32'b0000000000000110xxxxxxxx10101010;//loadi 6,X,0xAA
	#40
		$display("load r6        %b | %d",Result,Result);

		Read_Addr = 32'b0000000000000011xxxxxxxx10111011;//loadi 3,X,0xBB
	#40
		$display("load r3        %b | %d",Result,Result);

		Read_Addr = 32'b00000001000001010000011000000011;//add 5,6,3
	#40
		$display("add r5 (r6+r3) %b | %d  ****",Result,Result);

		Read_Addr = 32'b00000010000000010000010000000101;//and 1,4,5
	#40
		$display("and r1 (r4,r5) %b | %d",Result,Result);

		Read_Addr = 32'b00000011000000100000000100000110;//or 2,1,6
	#40
		$display("or r2 (r1,r6)  %b | %d",Result,Result);

		Read_Addr = 32'b0000100000001111xxxxxxxx00000010;//mov 7,X,2
	#40
		$display("copy r7 (r2)   %b | %d",Result,Result);

		Read_Addr = 32'b00001001000001000000111100000011;//sub 4,7,3
	#40
		$display("sub r4 (r7-r3) %b | %d",Result,Result);



  	Read_Addr = 32'b00001000000001000000000000010001;// loadi 4, X, 17
  #40
  	$display("loadi   %b | %d",Result,Result);

  	Read_Addr = 32'b00000101000000000000000000000100;// store 0, X, 4
  #40
  	$display("store %b | %d",Result,Result);

    Read_Addr = 32'b00000100000001010000000000000000;// load 5, X, 0
  #40
    $display("load %b | %d",Result,Result);

    Read_Addr = 32'b00000001000001100000010100000100;// add   5, 5, 4
  #40
    $display("add %b | %d",Result,Result);


	 // Operation set 2
  //
	// $display("\nOperation      Binary   | Decimal");
	// 	$display("---------------------------------");
  //
	// 	Read_Addr = 32'b0000000000000100xxxxxxxx00001101;//loadi 4,X,0xFF
	// #40
	// 	$display("load r4        %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b0000000000000110xxxxxxxx00101101;//loadi 6,X,0xAA
	// #40
	// 	$display("load r6        %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b0000000000000011xxxxxxxx00100001;//loadi 3,X,0xBB
	// #40
	// $display("load r3        %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b00000001000001010000011000000011;//add 5,6,3
	// #40
	// 	$display("add r5 (r3+r6) %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b00000010000000010000010000000101;//and 1,4,5
	// #40
	// 	$display("and r1 (r4,r5) %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b00000011000000100000000100000110;//or 2,1,6
	// #40
	// 	$display("or r2 (r1,r6)  %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b0000100000001111xxxxxxxx00000010;//mov 7,X,2
	// #40
	// 	$display("move r7 (r2)   %b | %d",Result,Result);
  //
	// 	Read_Addr = 32'b00001001000001000000111100000011;//sub 4,7,3
	// #40
	// 	$display("sub r4 (r7-r3) %b | %d",Result,Result);

		$finish;
	end
endmodule
