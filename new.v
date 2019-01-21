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
                    RESULT = DATA1; //load
                end
            3'b101:
                begin
                    RESULT = DATA1; //store
                end
            default:
                    RESULT = 8'b00000000;
        endcase
    end

endmodule

// // Register File
// module regfile8x8a (clk, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2, WAIT);
	
// 	input [2:0] OUT1addr,OUT2addr,INaddr;
// 	input [7:0] IN;
// 	input clk;
// 	input WAIT;
// 	output [7:0] OUT1,OUT2;

// 	reg [63:0] regMemory = 0;
// 	reg [7:0] OUT1reg, OUT2reg;
// 	integer i;
	
// 	assign OUT1 = OUT1reg[7:0];
// 	assign OUT2 = OUT2reg[7:0];

// 	always @(posedge clk) begin			//read @ positive edge
// 		for(i=0;i<8;i=i+1) begin
// 			OUT1reg[i] = regMemory[ OUT1addr*8 + i ];
// 			OUT2reg[i] = regMemory[ OUT2addr*8 + i ];
// 		end
// 	end	
	

// 	always @(negedge clk) begin			//write @ negative edge
// 		if ( !WAIT )begin			//Stall if DM access is happening
// 			for(i=0;i<8;i=i+1)begin
// 				regMemory[INaddr*8 + i] = IN[i];
// 			end
// 		end
// 	end

// endmodule

//REGISTER FILE
module regfile8x8a(CLK, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2, WAIT);
    input [7:0] IN;
    output reg [7:0] OUT1;
    output reg [7:0] OUT2;
    input [2:0] INaddr;
    input [2:0] OUT1addr;
    input [2:0] OUT2addr;
    input CLK;
    input WAIT;

    reg [7:0] 	 register0, register1, register2, register3, register4, register5, register6, register7;
      
    // Write functionality 
    always @(negedge CLK ) begin
      if (!WAIT)  begin
        case (INaddr)
            0: begin 
                register0 <= IN;
            end
            1: begin 
                register1 <= IN;
            end
            2: begin 
                register2 <= IN;
            end
            3: begin 
                register3 <= IN;
            end
            4: begin 
                register4 <= IN;
            end
            5: begin 
                register5 <= IN;
            end
            6: begin 
                register6 <= IN;
            end
            7: begin 
                register7 <= IN;
            end
        endcase // case (wrAddr)
        end
    end

    // always @ (posedge clk)
    // Read functionality
    always @(posedge CLK ) begin
        //OUT1
        case (OUT1addr)
          0: begin
            OUT1 <= register0;
          end
          1: begin
            OUT1 <= register1;
          end
          2: begin
            OUT1 <= register2;
          end
          3: begin
            OUT1 <= register3;
          end
          4: begin
            OUT1 <= register4;
          end
          5: begin
            OUT1 <= register5;
          end
          6: begin
            OUT1 <= register6;
          end
          7: begin
            OUT1 <= register7;
          end
          default:
            OUT1 <= 0;
        endcase
        //OUT2
        case (OUT2addr)
          0: begin
            OUT2 <= register0;
          end
          1: begin
            OUT2 <= register1;
          end
          2: begin
            OUT2 <= register2;
          end
          3: begin
            OUT2 <= register3;
          end
          4: begin
            OUT2 <= register4;
          end
          5: begin
            OUT2 <= register5;
          end
          6: begin
            OUT2 <= register6;
          end
          7: begin
            OUT2 <= register7;
          end
          default:
            OUT2 <= 0;
        endcase
    end
endmodule

//Program Counter
module counter(clk, reset, Read_addr, WAIT);
	input clk;
	input reset;
	input WAIT;
	output [31:0] Read_addr;
	reg Read_addr;

	always @(negedge clk)
	begin
		if ( !WAIT ) begin
			case(reset)
				1'b1 : begin 
                        Read_addr = 32'd0; 
                        end	//reset
				1'b0 : begin 
                        Read_addr = Read_addr + 3'b100; 
                        end	//otherwise
			endcase
		end
	end
endmodule

//2:1 Multiplexer
module mux(OUT, IN1, IN2, SELECT);
	input [7:0] IN1, IN2;
	output [7:0] OUT;
	input SELECT;
	reg [7:0] OUT;

	always @(IN1, IN2, SELECT)
	begin
		case( SELECT )
			0 : begin OUT <= IN1; end
			1 : begin OUT <= IN2; end
		endcase
	end
endmodule

//2's complement
module two_s_complement (OUT,IN);
  input signed [7:0] IN;
  output signed [7:0] OUT;

  assign OUT = ~IN + 8'b00000001;

endmodule //two_s_complement

//IR
module Instruction_reg ( clk, Read_Addr, instruction );
	input clk;
	input [31:0] Read_Addr;
	output [31:0] instruction;
	reg instruction;

	always @(negedge clk) 
	begin
	instruction = Read_Addr;
	end
endmodule

//Control Unit
module CU(instruction, WAIT, OUT1addr, OUT2addr, INaddr, Imm, Select, add_mux, im_mux, dm_mux, read, write, address);
	input [31:0] instruction;
	input WAIT;
	output [2:0] OUT1addr;
	output [2:0] OUT2addr;
	output [2:0] Select;
	output [2:0] INaddr;
	output [7:0] Imm, address;
	output add_mux, im_mux, dm_mux, read, write;

	reg [2:0] OUT1addr, OUT2addr, INaddr, Select;
	reg [7:0] Imm, address;
	reg add_mux,im_mux, dm_mux, read, write;
		
	always @(instruction) begin
		if ( !WAIT ) begin						//Stall if DM access is happening
			assign Select = instruction[26:24];		//Common Signals
			assign Imm = instruction[7:0];
			assign OUT1addr = instruction[2:0];
			assign OUT2addr = instruction[10:8];
			assign INaddr = instruction[18:16];
			assign im_mux = 1'b1;
			assign add_mux = 1'b0;
			assign write = 1'b0;
			assign read = 1'b0;
			assign dm_mux = 1'b1;
			
			case(instruction[31:24])
				
			8'b00000000 : begin			//loadi
				assign im_mux = 1'b0;
				end
			
			8'b00001001 : begin			//sub
				assign add_mux = 1'b1;
				end

			8'b00000100 : begin			//load
				assign read = 1'b1;
				assign dm_mux = 1'b0;
				assign address = instruction[7:0];	
			end
			
			8'b00000101: begin			//store
				assign write = 1'b1;
				assign address = instruction[23:16];
			end		

			endcase
		end
	end
endmodule

//Data Memory
module data_mem(clk, reset, read, write, address, write_data, read_data,WAIT);
	input clk;
	input reset;
	input read;
	input write;
	input[6:0] address;
	input[15:0] write_data;
	output[15:0] read_data;
	output WAIT;
	
	reg WAIT = 1'b0;
	reg[15:0] read_data;

	integer  i;
	
	// Declare memory 128x16 bits 
	reg [15:0] memory_array [127:0];

	always @(posedge reset)			//Reset Data memory
	begin
		if ( reset )
		begin
			for (i=0;i<128; i=i+1)
				memory_array[i] <= 0;
		end
	end
	
	always @( read, write, address, write_data ) begin
		if ( write && !read )			//Write to Data memory
		begin
			WAIT <= 1;
			//Artificial delay 98 cycles
			repeat(98)
			begin
				@(posedge clk);
			end
            $display("writing to memory [Data Memory Module]");
			memory_array[address] = write_data;
			WAIT <= 0;
		end
		if ( !write && read ) begin		//Read from Data memory
			WAIT <= 1;
			//Artificial delay 98 cycles
			repeat(98)
			begin
				@(posedge clk);
			end
            $display("reading from memory [Data Memory Module]");
			read_data = memory_array[address];
			WAIT <= 0;
		end
	end
	
endmodule

// Data Memory Cache
module data_cache(clk, reset, read, write, address, write_data, read_data, WAIT , dm_read, dm_write, dm_addr, dm_writeData, dm_readData, dm_WAIT);
	input clk;
    input reset;
	
    input read;					//Cache links with Control unit
    input write;
    input [7:0] address;
    input [7:0] write_data;
    output [7:0] read_data;
    output WAIT;

	output dm_read;				//Cache links with Data Memory
	output dm_write;
    output [6:0] dm_addr;
    output [15:0] dm_writeData;
    input [15:0] dm_readData;
	input dm_WAIT;

	reg [15:0] dm_writeData;
	reg [6:0] dm_addr;
	reg dm_read,dm_write;
	reg [7:0] read_data;
	wire WAIT;
	
	assign WAIT = dm_WAIT;

	integer  i;

	//Cache Memory 16x8 bits 
	//16 Bytes // 2Bytes/Block
	reg [7:0] cache_ram [15:0];
	
	wire cout;
	wire hit;
	reg valid [7:0];
	reg dirty [7:0];
	reg [3:0] cacheTAG [7:0];
	wire [3:0] tag;
	wire [2:0] index;
	wire offset;
	reg flag = 1'b0;
	

	assign offset = address[0];
	assign index = address[3:1];
	assign tag = address[7:4];

	always @(posedge reset)begin				//Cache Reset
		if(reset)begin
			for (i=0; i<8; i=i+1)begin
				valid [i] <= 0;
				dirty [i] <= 0;
			end	
			for (i=0; i<16; i=i+1) begin
				cache_ram[i] <= 0;
			end
		end
	end

	//Look for HIT
	Comparator cm1( cout, tag, cacheTAG[index] );
	and and1( hit, cout, valid[index] );
	
	always @( clk ) begin
	
		if ( write && !read )begin			//Write to Data memory
			if( hit && !dm_WAIT ) begin
				if( flag ) begin				//IF fetching from DM is finished store in Cache
					cache_ram[ 2*index ] = dm_readData[7:0];
					cache_ram[ 2*index+1 ] = dm_readData[15:8];
					flag = 1'b0;
				end
				cache_ram[ 2*index+offset ] = write_data;
				dirty[index] = 1'b1;
			end
			
			if( !hit ) begin
                //$display("MISS");
				begin
					@(posedge clk )begin
						if( dirty[index] && !dm_WAIT ) begin
                            $display("dirty - Writing Back [Cache Module]");
							dm_writeData[7:0] = cache_ram[ 2*index ];
							dm_writeData[15:8] = cache_ram[ 2*index +1 ];

							dm_read = 1'b0;
							dm_write = 1'b1;

							dm_addr[2:0] = address[3:1];
							dm_addr[6:3] = cacheTAG[ index ];
							dirty[index] = 1'b0;
						end  
					end
				end
				begin
					@(negedge clk)begin
						if( !dirty[index] && !dm_WAIT ) begin
                            $display("Not Dirty - Fetching from memory [Cache Module]");
							dm_addr = address[7:1];
							dm_read = 1'b1;
							dm_write = 1'b0;
							cacheTAG[ index ] = address[7:4];
							valid[index] = 1'b1;
							flag = 1'b1;
						end
					end
				end
			end
		end
		
		if ( !write && read ) begin		//Read from Data memory
			if( hit && !dm_WAIT ) begin
				if( flag ) begin				//IF fetching from DM is finished store in Cache
					cache_ram[ 2*index ] = dm_readData[7:0];
					cache_ram[ 2*index+1 ] = dm_readData[15:8];
					flag = 1'b0;
				end
				read_data = cache_ram[ 2*index+offset ];
			end

			if( !hit ) begin
                //$display("MISS");
				begin
					@(posedge clk)begin
						if( dirty[index] && !dm_WAIT ) begin
                            $display("Dirty - Writing Back [Cache Module]");
							dm_writeData[7:0] = cache_ram[ 2*index ];
							dm_writeData[15:8] = cache_ram[ 2*index +1 ];
							
							dm_read = 1'b0;
							dm_write = 1'b1;

							dm_addr[2:0] = address[3:1];
							dm_addr[6:3] = cacheTAG[ index ];

							dirty[index] = 1'b0;
						end  
					end
				end
				begin
					@(negedge clk)begin
						if( !dirty[index] && !dm_WAIT ) begin
                            $display("Not dirty - Fetching from Memory [Cache Module]");
							dm_addr = address[7:1];
							dm_read = 1'b1;
							dm_write = 1'b0;
							cacheTAG[ index ] = address[7:4];
							valid[index] = 1'b1;
							flag = 1'b1;
						end	
					end
				end
			end			
		end
	end

endmodule

//comparator 
module Comparator( OUT, IN1, IN2 );
	input [3:0] IN1;
	input [3:0] IN2;
	output OUT;

	wire out1,out2,out3,out4;

	xnor xnor1( out1, IN1[0], IN2[0] );
	xnor xnor2( out2, IN1[1], IN2[1] );
	xnor xnor3( out3, IN1[2], IN2[2] );
	xnor xnor4( out4, IN1[3], IN2[3] );
	and and1( OUT, out1, out2, out3, out4 );
	
endmodule

// Processor
module Processor( Read_Addr, DataMemMUXout , clk, reset );
	
	input [31:0] Read_Addr;
	input clk,reset;
	output [7:0] DataMemMUXout;
	
	wire [7:0] hit;
	wire [7:0] Result;
	wire [31:0] instruction;
	wire [2:0] OUT1addr,OUT2addr,INaddr,Select;
	wire  [7:0] Imm,OUT1,OUT2,OUTPUT,INPUT,cmp;
	wire [7:0] read_data,address;
	wire [7:0] imValueMUXout, addSubMUXout, DataMemMUXout;
	wire addSubMUX, imValueMUX, dmMUX;
	wire read, write, WAIT, reset;
	wire [6:0] dm_addr;
	wire [15:0] dm_writeData, dm_readData;
	wire dm_read, dm_write, dm_WAIT;
	
	Instruction_reg ir(clk, Read_Addr, instruction);				//Instruction Regiter
	CU cu( instruction, WAIT, OUT1addr, OUT2addr, INaddr, Imm, Select, addSubMUX, imValueMUX, dmMUX, read, write, address );	//Control Unit
	regfile8x8a rf( clk, INaddr, DataMemMUXout, OUT1addr, OUT1, OUT2addr, OUT2, WAIT );			//Register File
	two_s_complement tscomp( OUTPUT, OUT1 );							//2'sComplement
	mux addsubMUX( addSubMUXout, OUT1, OUTPUT, addSubMUX );			//2's complement MUX
	mux immValMUX( imValueMUXout, Imm, addSubMUXout, imValueMUX );	//Imediate Value MUX
	mux DataMemMUX( DataMemMUXout, read_data ,Result, dmMUX);		//Data Memory MUX 
	alu aluX( Result, imValueMUXout, OUT2, Select );				//alu
	
	data_cache cache( clk, reset, read, write, address, Result, read_data, WAIT ,
					dm_read, dm_write, dm_addr, dm_writeData, dm_readData, dm_WAIT );		//Data Memory Cache
	data_mem dataMem( clk, reset, dm_read, dm_write, dm_addr, dm_writeData, dm_readData, dm_WAIT);	//Data Memory

endmodule

module testbench;
	reg [31:0] Read_Addr;
	wire [7:0] Result;
	reg clk,reset;

	Processor pro( Read_Addr, Result, clk, reset);

	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end

	initial begin
		
		reset = 0;
		#20
		reset = 1;
		#20
		reset = 0;
		#20
		
		Read_Addr = 32'b00000000_00000100_xxxxxxxx_00000111;//loadi r4,X,7
		$display("loadi reg4,X,7");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result);
		Read_Addr = 32'b00000000_00000110_xxxxxxxx_00000001;//loadi r6,X,1
		$display("loadi reg6,X,1");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result);

		Read_Addr = 32'b00000101_00011001_xxxxxxxx_00000100;//store 25,X,r4
		$display("store mem[25],X,reg4");
		#2000
		$display("After 100 CC\nOUTPUT: %d\n",Result);
		Read_Addr = 32'b00000101_00011000_xxxxxxxx_00000110;//store 24,X,r6
		$display("store mem[16],X,reg6");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result );
		
		Read_Addr = 32'b00000100_00000001_xxxxxxxx_00011001;//load r1,X,25
		$display("load reg1,X,mem[25]");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result);
		Read_Addr = 32'b00000100_00001000_xxxxxxxx_00011000;//load r8,X,24
		$display("load reg8,X,mem[24]");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result);
		
		Read_Addr = 32'b00000000_00000011_xxxxxxxx_00110001;//loadi r6,X,49
		$display("loadi reg[3],X,49");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result);
		
		Read_Addr = 32'b00000101_00111000_xxxxxxxx_00000011;//store 56,X,r6
		$display("store mem[56],X,reg3");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d",Result);
		#1980
		$display("100 clk cycles elapsed:\nOUTPUT: %d",Result);
		#2000
		$display("200 clc cycles elapsed:\nOUTPUT: %d )\n",Result);
		
		Read_Addr = 32'b00000100_00001000_xxxxxxxx_00111000;//load r8,X,56
		$display("load reg8,X,mem[56]");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d\n",Result);
		
		Read_Addr = 32'b00000001_00000101_00000001_00001000;//add reg5,reg1,reg8
		$display("add reg5,reg1,reg8");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d (49+7)\n",Result);
		Read_Addr = 32'b00001001_00000101_00001000_00000001;//sub reg4,reg8,reg1
		$display("sub reg4,reg8,reg1");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d (49-7)\n",Result);

		Read_Addr = 32'b0000010000001000xxxxxxxx00011000;//load r8,X,24
		$display("load reg8,X,mem[24]");
		#20
		$display("1 clk cycle elapsed:\nOUTPUT: %d",Result);
		#1980
		$display("100 clk cycles elapsed:\nOUTPUT: %d",Result);
		#2000
		$display("200 clc cycles elapsed:\nOUTPUT: %d\n",Result);
		
		
		$finish;
	end

endmodule