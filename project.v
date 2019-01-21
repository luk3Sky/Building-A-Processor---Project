//alu module
module alu( RESULT, DATA1, DATA2, SELECT);
	input [7:0] DATA1,DATA2;	
	input [2:0] SELECT;
	output [7:0] RESULT;
	reg [7:0] res_out;
	
	assign RESULT= res_out;

	always @(DATA1,DATA2,SELECT)
    	begin
        case ( SELECT )
         0 : res_out = DATA1;			//Forward ( loadi, mov )
         1 : res_out = DATA1 + DATA2;	//Addition ( add, sub )
         2 : res_out = DATA1 & DATA2;	//Bitwise AND ( and )
         3 : res_out = DATA1 | DATA2;	//Bitwise OR ( or )
		 4 : res_out = DATA1;			//Forward ( load )
		 5 : res_out = DATA1;			//Forward ( store )
		default : res_out = 0;
        endcase 
    end

endmodule

// Register File
module regfile8x8a (clk, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2, busy_wait);
	
	input [2:0] OUT1addr,OUT2addr,INaddr;
	input [7:0] IN;
	input clk;
	input busy_wait;
	output [7:0] OUT1,OUT2;

	reg [63:0] regMemory = 0;
	reg [7:0] OUT1reg, OUT2reg;
	integer i;
	
	assign OUT1 = OUT1reg[7:0];
	assign OUT2 = OUT2reg[7:0];

	always @(posedge clk) begin			//read @ positive edge
		for(i=0;i<8;i=i+1) begin
			OUT1reg[i] = regMemory[ OUT1addr*8 + i ];
			OUT2reg[i] = regMemory[ OUT2addr*8 + i ];
		end
	end	
	

	always @(negedge clk) begin			//write @ negative edge
		if ( !busy_wait )begin			//Stall if DM access is happening
			for(i=0;i<8;i=i+1)begin
				regMemory[INaddr*8 + i] = IN[i];
			end
		end
	end

endmodule

//Program Counter
module counter(clk, reset, Read_addr, busy_wait);
	input clk;
	input reset;
	input busy_wait;
	output [31:0] Read_addr;
	reg Read_addr;

	always @(negedge clk)
	begin
		if ( !busy_wait ) begin			//Stall if DM access is happening
			case(reset)
				1'b1 : begin Read_addr = 32'd0; end					//Reset if reset = 1
				1'b0 : begin Read_addr = Read_addr + 3'b100; end	//pc = pc + 4 if reset = 0
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
module two_s_complement( OUT, IN );
	input [7:0] IN;
	output [7:0] OUT;

	assign OUT[7:0] = -IN[7:0];

endmodule

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
module CU(instruction, busy_wait, OUT1addr, OUT2addr, INaddr, Imm, Select, add_mux, im_mux, dm_mux, read, write, address);
	input [31:0] instruction;
	input busy_wait;
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
		if ( !busy_wait ) begin						//Stall if DM access is happening
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
module data_mem(clk, rst, read, write, address, write_data, read_data,	busy_wait);
	input clk;
	input rst;
	input read;
	input write;
	input[6:0] address;
	input[15:0] write_data;
	output[15:0] read_data;
	output busy_wait;
	
	reg busy_wait = 1'b0;
	reg[15:0] read_data;

	integer  i;
	
	// Declare memory 128x16 bits 
	reg [15:0] memory_array [127:0];

	always @(posedge rst)			//Reset Data memory
	begin
		if ( rst )
		begin
			for (i=0;i<128; i=i+1)
				memory_array[i] <= 0;
		end
	end
	
	always @( read, write, address, write_data ) begin
		if ( write && !read )			//Write to Data memory
		begin
			busy_wait <= 1;
			//Artificial delay 98 cycles
			repeat(98)
			begin
				@(posedge clk);
			end
            $display("writing to memory");
			memory_array[address] = write_data;
			busy_wait <= 0;
		end
		if ( !write && read ) begin		//Read from Data memory
			busy_wait <= 1;
			//Artificial delay 98 cycles
			repeat(98)
			begin
				@(posedge clk);
			end
            $display("reading from memory");
			read_data = memory_array[address];
			busy_wait <= 0;
		end
	end
	
endmodule

// Data Memory Cache
module data_cache(clk, rst, read, write, address, write_data, read_data, busy_wait ,
					dm_read, dm_write, dm_addr, dm_writeData, dm_readData, dm_busyWait);
	input clk;
    input rst;
	
    input read;					//Cache links with Control unit
    input write;
    input [7:0] address;
    input [7:0] write_data;
    output [7:0] read_data;
    output busy_wait;

	output dm_read;				//Cache links with Data Memory
	output dm_write;
    output [6:0] dm_addr;
    output [15:0] dm_writeData;
    input [15:0] dm_readData;
	input dm_busyWait;

	reg [15:0] dm_writeData;
	reg [6:0] dm_addr;
	reg dm_read,dm_write;
	reg [7:0] read_data;
	wire busy_wait;
	
	assign busy_wait = dm_busyWait;

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

	always @(posedge rst)begin				//Cache Reset
		if(rst)begin
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
			if( hit && !dm_busyWait ) begin
				if( flag ) begin				//IF fetching from DM is finished store in Cache
					cache_ram[ 2*index ] = dm_readData[7:0];
					cache_ram[ 2*index+1 ] = dm_readData[15:8];
					flag = 1'b0;
				end
				cache_ram[ 2*index+offset ] = write_data;
				dirty[index] = 1'b1;
			end
			
			if( !hit ) begin	//If not a hit
				begin
					@(posedge clk )begin
						if( dirty[index] && !dm_busyWait ) begin			//If dirty Write back
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
						if( !dirty[index] && !dm_busyWait ) begin			//If not dirty fetch from Data Memory
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
			if( hit && !dm_busyWait ) begin
				if( flag ) begin				//IF fetching from DM is finished store in Cache
					cache_ram[ 2*index ] = dm_readData[7:0];
					cache_ram[ 2*index+1 ] = dm_readData[15:8];
					flag = 1'b0;
				end
				read_data = cache_ram[ 2*index+offset ];
			end

			if( !hit ) begin		//If not a hit	
				begin
					@(posedge clk)begin
						if( dirty[index] && !dm_busyWait ) begin			//If dirty Write back
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
						if( !dirty[index] && !dm_busyWait ) begin			//If not dirty fetch from Data Memory
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
module Processor( Read_Addr, DataMemMUXout , clk, rst );
	
	input [31:0] Read_Addr;
	input clk,rst;
	output [7:0] DataMemMUXout;
	
	wire [7:0] hit;
	wire [7:0] Result;
	wire [31:0] instruction;
	wire [2:0] OUT1addr,OUT2addr,INaddr,Select;
	wire  [7:0] Imm,OUT1,OUT2,OUTPUT,INPUT,cmp;
	wire [7:0] read_data,address;
	wire [7:0] imValueMUXout, addSubMUXout, DataMemMUXout;
	wire addSubMUX, imValueMUX, dmMUX;
	wire read, write, busy_wait, rst;
	wire [6:0] dm_addr;
	wire [15:0] dm_writeData, dm_readData;
	wire dm_read, dm_write, dm_busyWait;
	
	Instruction_reg ir(clk, Read_Addr, instruction);				//Instruction Regiter
	CU cu( instruction, busy_wait, OUT1addr, OUT2addr, INaddr, Imm, Select, addSubMUX, imValueMUX, dmMUX, read, write, address );	//Control Unit
	regfile8x8a rf( clk, INaddr, DataMemMUXout, OUT1addr, OUT1, OUT2addr, OUT2, busy_wait );			//Register File
	two_s_complement tscomp( OUTPUT, OUT1 );							//2'sComplement
	mux addsubMUX( addSubMUXout, OUT1, OUTPUT, addSubMUX );			//2's complement MUX
	mux immValMUX( imValueMUXout, Imm, addSubMUXout, imValueMUX );	//Imediate Value MUX
	mux DataMemMUX( DataMemMUXout, read_data ,Result, dmMUX);		//Data Memory MUX 
	alu aluX( Result, imValueMUXout, OUT2, Select );				//alu
	
	data_cache cache( clk, rst, read, write, address, Result, read_data, busy_wait ,
					dm_read, dm_write, dm_addr, dm_writeData, dm_readData, dm_busyWait );		//Data Memory Cache
	data_mem dataMem( clk, rst, dm_read, dm_write, dm_addr, dm_writeData, dm_readData, dm_busyWait);	//Data Memory

endmodule

module testbench;
	reg [31:0] Read_Addr;
	wire [7:0] Result;
	reg clk,rst;

	Processor pro( Read_Addr, Result, clk, rst);

	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end

	initial begin
		
		$display("\nPrinting The results of MUX that is before register file( output from alu OR DM )\n");
		rst = 0;
		#20
		rst = 1;
		#20
		rst = 0;
		#20
		
		Read_Addr = 32'b0000000000000110xxxxxxxx00000111;//loadi r6,X,7
		$display("loadi reg6,X,7");
		#20
		$display("After 1 CC	%b | %d\n",Result,Result);
		Read_Addr = 32'b0000000000000011xxxxxxxx00000001;//loadi r3,X,1
		$display("loadi reg3,X,1");
		#20
		$display("After 1 CC	%b | %d\n",Result,Result);

		Read_Addr = 32'b0000010100011001xxxxxxxx00000110;//store 25,X,r6
		$display("store mem[25],X,reg6");
		#2000
		$display("After 100 CC	%b | %d\n",Result,Result);
		Read_Addr = 32'b0000010100011000xxxxxxxx00000011;//store 24,X,r3
		$display("store mem[16],X,reg3");
		#20
		$display("After 1 CC	%b | %d\n",Result,Result );
		
		Read_Addr = 32'b0000010000000111xxxxxxxx00011001;//load r7,X,25
		$display("load reg7,X,mem[25]");
		#20
		$display("After 1 CC	%b | %d (Earliar it took 100CC )\n",Result,Result);
		Read_Addr = 32'b0000010000001000xxxxxxxx00011000;//load r8,X,24
		$display("load reg8,X,mem[24]");
		#20
		$display("After 1 CC	%b | %d (Earliar it took 100CC )\n",Result,Result);
		
		Read_Addr = 32'b0000000000000011xxxxxxxx00110001;//loadi r3,X,49
		$display("loadi reg[3],X,mem[49]");
		#20
		$display("After 1 CC	%b | %d\n",Result,Result);
		
		Read_Addr = 32'b0000010100111000xxxxxxxx00000011;//store 56,X,r3
		$display("store mem[56],X,reg3");
		#20
		$display("After 1 CC	%b | %d (It will be a (Write) miss & Dirty)",Result,Result);
		#1980
		$display("After 100 CC	%b | %d (takes 100CC to Write-Back)",Result,Result);
		#2000
		$display("After 200 CC	%b | %d (takes 100CC to Fetch from DM)\n",Result,Result);
		
		Read_Addr = 32'b0000010000001000xxxxxxxx00111000;//load r8,X,56
		$display("load reg8,X,mem[56]");
		#20
		$display("After 1 CC	%b | %d (Its a hit takes 1CC)\n",Result,Result);
		
		Read_Addr = 32'b00000001000001010000011100001000;//add 5,7,8
		$display("add reg5,reg7,reg8");
		#20
		$display("After 1 CC	%b | %d (49+7)\n",Result,Result);
		Read_Addr = 32'b00001001000001010000100000000111;//sub 4,8,7
		$display("sub reg4,reg8,reg7");
		#20
		$display("After 1 CC	%b | %d (49-7)\n",Result,Result);

		Read_Addr = 32'b0000010000001000xxxxxxxx00011000;//load r8,X,24
		$display("load reg8,X,mem[24]");
		#20
		$display("After 1 CC	%b | %d (It should be 1. It is a (Read) miss & Dirty)",Result,Result);
		#1980
		$display("After 100 CC	%b | %d (It should be 1. Takes 100CC to Write Back)",Result,Result);
		#2000
		$display("After 200 CC	%b | %d (It should be 1. Takes 100CC to Fetch from DM)\n",Result,Result);
		
		// #20
		// Read_Addr = 32'b00000001000001010000011100001000;//add 5,7,8
		// $display("add 5,7,8");
		// #20
		// $display("After 1 CC	%b | %d\n",Result,Result);
		// Read_Addr = 32'b00001001000001010000100000000111;//sub 4,8,7
		// $display("sub 4,8,7");
		// #20
		// $display("After 1 CC	%b | %d\n",Result,Result);

		// Read_Addr = 32'b0000010000001000xxxxxxxx00111000;//load r8,X,56
		// $display("load 8,X,56");
		// #20
		// $display("After 1 CC	%b | %d (It should be 95. It is a (Read) miss & Not Dirty. WB not needed. Just Fetching from DM)",Result,Result);
		// #2000
		// $display("After 100 CC	%b | %d (It should be 95. Takes 100CC to to Fetching)",Result,Result);
		
		$finish;
	end

endmodule