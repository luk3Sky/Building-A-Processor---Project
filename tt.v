// -------- Program Counter ----------

module counter(clk, reset, Read_addr, busy_wait );
	input clk;
	input reset;
	input busy_wait;
	output [7:0] Read_addr;
	reg Read_addr;

	always @(*)
	begin
		if ( !busy_wait ) begin			//Stall if DM access is happening
			case(reset)
				1'b1 : begin Read_addr = 8'd0; end					//Reset if reset = 1
				1'b0 : begin Read_addr = Read_addr + 8'd1; end	//PC = PC + 1, if reset = 0
			endcase
		end
	end
endmodule

// -------- Instruction Register ------------

module Instruction_reg ( clk, Read_Addr, instruction ,reset);
	input clk,reset;
	input [7:0] Read_Addr;
	output [31:0] instruction;
    wire instruction;
endmodule

// ----------Instruction Memory --------------

module Instruction_mem( clk, read ,Read_Addr, instruction, IMbusy_wait,reset);
       input clk,reset,read;
       input [5:0] Read_Addr;
       output reg [127:0] instruction;
       output reg IMbusy_wait;
       //reg instruction;

       // Declare memory 64x128 bits 
	   reg [127:0] memory_array [63:0];
	   
	   always @(reset)
	      begin
	      memory_array[0] = 128'b00000101001110000000000000000011000001010011100100000000000000100000000000000011000000000000001100000000000000100000000000011101;
          memory_array[1] = 128'b00000101000110000000000000000011000000000000001100000000010000110000010000001000000000000011100000000100000001110000000000111001;
          memory_array[2] = 128'b00000100000010000000000000111000000010010000010100001000000001110000000100000101000001110000100000000100000010000000000000011000;
          memory_array[3] = 128'b00000101000110010000000000000010000001000000100000000000000110000000010100111001000000000000001100000100000010000000000000111001;
          memory_array[4] = 128'b00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001000001010000011100000010;
          memory_array[5] = 128'b00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001000001010000011100000010;
          end
          
       always @(read,Read_Addr)
       begin
          IMbusy_wait <= 1;
          repeat(98)
		  begin
			@(negedge clk);
		  end
          instruction = memory_array[Read_Addr];
          IMbusy_wait <= 0;
       end
endmodule

// --------- Instruction Memory Cache ----------

module cache_mem1( clk, rst, read, address, read_data, busy_wait ,
					IMread, IMaddress, IMread_data, IMbusy_wait );
	input clk;
    input rst;
	
    input read;					//Cache links with Control unit
    input [7:0] address;
    output [31:0] read_data;
    output busy_wait;

	output IMread;				//Cache links with Data Memory
    output [5:0] IMaddress;
    input [127:0] IMread_data;
	input IMbusy_wait;

	reg [5:0] IMaddress;
	reg IMread;
	reg [31:0] read_data;
	wire busy_wait;

	
	assign busy_wait = IMbusy_wait;

	integer  i;
	reg [31:0] cache_ram [15:0];
	
	wire cout;
	wire hit;
	reg valid [3:0];
	reg [3:0] cacheTAG [3:0];
	wire [3:0] tag;
	wire [1:0] index;
	wire [1:0] offset;
	reg flag = 1'b0;
	

	assign offset = address[1:0];
	assign index = address[3:2];
	assign tag = address[7:4];

	always @(posedge rst)begin				//Cache Reset
		if(rst)begin
			for (i=0; i<4; i=i+1)begin
				valid [i] <= 0;
			end	
			for (i=0; i<16; i=i+1) begin
				cache_ram[i] <= 0;
			end
		end
	end

	//Look for HIT
	Comparator cm1( cout, tag, cacheTAG[index] );
	and and1( hit, cout, valid[index] );
	
	always @(negedge clk ) begin
		
		//Read from Instruction memory

			if( hit && !IMbusy_wait ) begin
				if( flag ) begin
					cache_ram[ 4*index ] = IMread_data[31:0];
					cache_ram[ 4*index+1 ] = IMread_data[63:32];
					cache_ram[ 4*index+2 ] = IMread_data[95:64];
					cache_ram[ 4*index+3 ] = IMread_data[127:96];
					flag = 1'b0;
				end

				read_data = cache_ram[ 4*index+offset ];
			end

			if( !hit ) begin		//If not a hit	
				begin
					@(negedge clk)begin
							IMaddress = address[7:2];
							IMread = 1'b1;
							cacheTAG[ index ] = address[7:4];
							valid[index] = 1'b1;
							flag = 1'b1;
					end
				end
			end
		
	end

endmodule

// --------- Control Unit ------------

module CU( instruction, busy_wait, OUT1addr, OUT2addr, INaddr, Imm, Select, addSubMUX, imValueMUX, dmMUX, read, write, address );
	input [31:0] instruction;
	input busy_wait;
	output [2:0] OUT1addr;
	output [2:0] OUT2addr;
	output [2:0] Select;
	output [2:0] INaddr;
	output [7:0] Imm, address;
	output addSubMUX, imValueMUX, dmMUX, read, write;

	reg [2:0] OUT1addr, OUT2addr, INaddr, Select;
	reg [7:0] Imm, address;
	reg addSubMUX,imValueMUX, dmMUX, read, write;
		
	always @(instruction) begin
		if ( !busy_wait ) begin						//Stall if DM access is happening
			assign Select = instruction[26:24];		//Common Signals
			assign Imm = instruction[7:0];
			assign OUT1addr = instruction[2:0];
			assign OUT2addr = instruction[10:8];
			assign INaddr = instruction[18:16];
			assign imValueMUX = 1'b1;
			assign addSubMUX = 1'b0;
			assign write = 1'b0;
			assign read = 1'b0;
			assign dmMUX = 1'b1;
			
			case(instruction[31:24])
				
			8'b00000000 : begin			//loadi
				assign imValueMUX = 1'b0;
				end
			
			8'b00001001 : begin			//sub
				assign addSubMUX = 1'b1;
				end

			8'b00000100 : begin			//load
				assign read = 1'b1;
				assign dmMUX = 1'b0;
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

// --------- Register File -------------

module regfile8x8a ( clk, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2, busy_wait );
	
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

	always @(posedge clk) begin			//Read at postive edge of Clock
		for(i=0;i<8;i=i+1) begin
			OUT1reg[i] = regMemory[ OUT1addr*8 + i ];
			OUT2reg[i] = regMemory[ OUT2addr*8 + i ];
		end
	end	
	

	always @(negedge clk) begin			//Write at negative edge of Clock
		if ( !busy_wait )begin			//Stall if DM access is happening
			for(i=0;i<8;i=i+1)begin
				regMemory[INaddr*8 + i] = IN[i];
			end
		end
	end

endmodule

// --------- 2's Complement ------------

module TwosComplement( OUTPUT, INPUT );
	input [7:0] INPUT;
	output [7:0] OUTPUT;

	assign OUTPUT[7:0] = -INPUT[7:0];

endmodule

// ---------- Multiplexer 2x1 -----------

module MUX( OUTPUT, INPUT1, INPUT2, CTRL );
	input [7:0] INPUT1, INPUT2;
	output [7:0] OUTPUT;
	input CTRL;
	reg [7:0] OUTPUT;

	always @( INPUT1, INPUT2, CTRL )
	begin
		case( CTRL )
			1'b0 : begin OUTPUT <= INPUT1; end
			1'b1 : begin OUTPUT <= INPUT2; end
		endcase
	end
endmodule

// ---------- ALU --------------

module ALU( RESULT, DATA1, DATA2, SELECT );
	input [7:0] DATA1,DATA2;	//Source 1 & 2	
	input [2:0] SELECT;
	output [7:0] RESULT;
	reg [7:0] Res;
	
	assign RESULT= Res;

	always @(DATA1,DATA2,SELECT)
    	begin
        case ( SELECT )
         0 : Res = DATA1;			//Forward ( loadi, mov )
         1 : Res = DATA1 + DATA2;	//Addition ( add, sub )
         2 : Res = DATA1 & DATA2;	//Bitwise AND ( and )
         3 : Res = DATA1 | DATA2;	//Bitwise OR ( or )
		 4 : Res = DATA1;			//Forward ( load )
		 5 : Res = DATA1;			//Forward ( store )
		default : Res = 0;
        endcase 
    end

endmodule

// --------- Data Memory -----------

module data_mem( clk, rst, read, write, address, write_data, read_data,	busy_wait );
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

			read_data = memory_array[address];
			busy_wait <= 0;
		end
	end
	
endmodule

// --------- Data Memory Cache ----------

module cache_mem( clk, rst, read, write, address, write_data, read_data, busy_wait ,
					DMread, DMwrite, DMaddress, DMwrite_data, DMread_data, DMbusy_wait );
	input clk;
    input rst;
	
    input read;					//Cache links with Control unit
    input write;
    input [7:0] address;
    input [7:0] write_data;
    output [7:0] read_data;
    output busy_wait;

	output DMread;				//Cache links with Data Memory
	output DMwrite;
    output [6:0] DMaddress;
    output [15:0] DMwrite_data;
    input [15:0] DMread_data;
	input DMbusy_wait;

	reg [15:0] DMwrite_data;
	reg [6:0] DMaddress;
	reg DMread,DMwrite;
	reg [7:0] read_data;
	wire busy_wait;

	
	assign busy_wait = DMbusy_wait;

	integer  i;
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
			if( hit && !DMbusy_wait ) begin
				if( flag ) begin
					cache_ram[ 2*index ] = DMread_data[7:0];
					cache_ram[ 2*index+1 ] = DMread_data[15:8];
					flag = 1'b0;
				end

				cache_ram[ 2*index+offset ] = write_data;
				dirty[index] = 1'b1;

			end
			
			if( !hit ) begin	//If not a hit
				
				begin
					@(posedge clk )begin
						if( dirty[index] && !DMbusy_wait ) begin			//If dirty Write back
							DMwrite_data[7:0] = cache_ram[ 2*index ];
							DMwrite_data[15:8] = cache_ram[ 2*index +1 ];

							DMread = 1'b0;
							DMwrite = 1'b1;

							DMaddress[2:0] = address[3:1];
							DMaddress[6:3] = cacheTAG[ index ];
							dirty[index] = 1'b0;

						end  
					end
				end

				begin
					@(negedge clk)begin
						if( !dirty[index] && !DMbusy_wait ) begin			//If not dirty fetch from Data Memory
							DMaddress = address[7:1];
							DMread = 1'b1;
							DMwrite = 1'b0;
							cacheTAG[ index ] = address[7:4];
							valid[index] = 1'b1;
							flag = 1'b1;
						end
					end
				end
			end

		end
		
		if ( !write && read ) begin		//Read from Data memory

			if( hit && !DMbusy_wait ) begin
				if( flag ) begin
					cache_ram[ 2*index ] = DMread_data[7:0];
					cache_ram[ 2*index+1 ] = DMread_data[15:8];
					flag = 1'b0;
				end

				read_data = cache_ram[ 2*index+offset ];

			end

			if( !hit ) begin		//If not a hit	

				begin
					@(posedge clk)begin
						if( dirty[index] && !DMbusy_wait ) begin			//If dirty Write back
							DMwrite_data[7:0] = cache_ram[ 2*index ];
							DMwrite_data[15:8] = cache_ram[ 2*index +1 ];
							
							DMread = 1'b0;
							DMwrite = 1'b1;

							DMaddress[2:0] = address[3:1];
							DMaddress[6:3] = cacheTAG[ index ];

							dirty[index] = 1'b0;
						end  
					end
				end
				
				begin
					@(negedge clk)begin
						if( !dirty[index] && !DMbusy_wait ) begin			//If not dirty fetch from Data Memory
							DMaddress = address[7:1];
							DMread = 1'b1;
							DMwrite = 1'b0;
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

// ----------- Comparator ----------

module Comparator( Out, Input1, Input2 );
	input [3:0] Input1;
	input [3:0] Input2;
	output Out;

	wire out1,out2,out3,out4;

	xnor xnor1( out1, Input1[0], Input2[0] );
	xnor xnor2( out2, Input1[1], Input2[1] );
	xnor xnor3( out3, Input1[2], Input2[2] );
	xnor xnor4( out4, Input1[3], Input2[3] );
	and and1( Out, out1, out2, out3, out4 );
	
endmodule

//---------- Processor ----------

module Processor( Read_Addr, DataMemMUXout , clk, rst );
	
	input [7:0] Read_Addr;
	input clk,rst;
	output [7:0] DataMemMUXout;
	
	wire [7:0] hit;
	wire [7:0] Result;
	wire [31:0] instruction,read_inst;
	wire [2:0] OUT1addr,OUT2addr,INaddr,Select;
	wire  [7:0] Imm,OUT1,OUT2,OUTPUT,INPUT,cmp;
	wire [7:0] read_data,address;
	wire [7:0] imValueMUXout, addSubMUXout, DataMemMUXout;
	wire addSubMUX, imValueMUX, dmMUX;
	wire read, write, busy_wait, rst;
	wire [6:0] DMaddress;
	wire [5:0] IMaddress;
	wire [15:0] DMwrite_data, DMread_data;
	wire [127:0] IMread_data;
	wire DMread, DMwrite, DMbusy_wait,IMbusy_wait,IMread;
	
	Instruction_reg ir1(clk, Read_Addr, instruction,rst);				//Instruction Regiter
	cache_mem1 cm1( clk, rst, read, Read_Addr, read_inst, busy_wait ,IMread, IMaddress, IMread_data, IMbusy_wait );
	Instruction_mem im1( clk, IMread,IMaddress, IMread_data, IMbusy_wait,rst);
	CU cu1( read_inst, busy_wait, OUT1addr, OUT2addr, INaddr, Imm, Select, addSubMUX, imValueMUX, dmMUX, read, write, address );	//Control Unit
	regfile8x8a rf1( clk, INaddr, DataMemMUXout, OUT1addr, OUT1, OUT2addr, OUT2, busy_wait );			//Register File
	TwosComplement tcomp( OUTPUT, OUT1 );							//2'sComplement
	MUX addsubMUX( addSubMUXout, OUT1, OUTPUT, addSubMUX );			//2's complement MUX
	MUX immValMUX( imValueMUXout, Imm, addSubMUXout, imValueMUX );	//Imediate Value MUX
	MUX DataMemMUX( DataMemMUXout, read_data ,Result, dmMUX);		//Data Memory MUX 
	ALU alu1( Result, imValueMUXout, OUT2, Select );				//ALU
	
	cache_mem cm( clk, rst, read, write, address, Result, read_data, busy_wait ,
					DMread, DMwrite, DMaddress, DMwrite_data, DMread_data, DMbusy_wait );		//Data Memory Cache
	data_mem dm( clk, rst, DMread, DMwrite, DMaddress, DMwrite_data, DMread_data, DMbusy_wait);	//Data Memory

endmodule
//////////////////////
module testbench;
	reg [7:0] Read_Addr;
	wire [7:0] Result;
	reg clk,rst;

    initial
    begin
      $dumpfile("testbench.vcd");
      $dumpvars(0,testbench);
    end
    
	Processor simpleP( Read_Addr, Result, clk, rst);

	initial begin
		clk = 0;
		forever #10 clk = ~clk;
	end

	initial begin

		rst = 0;
		#20
		rst = 1;
		#20
		rst = 0;
		#20
		Read_Addr = 8'd0;//loadi r2,X,29
		$display("\nloadi 2,X,29");
		#20
		$display("After 1 CC	%b | %d (It will be a Instruction Read miss)",Result,Result);
		#2000
		#20
		$display("After 100 CC	%b | %d\n",Result,Result);
		Read_Addr = 8'd1;//loadi r3,X,3
		$display("loadi 3,X,3");
		#20
		$display("After 1 CC	%b | %d \n",Result,Result);

		Read_Addr = 8'd2;//store 57,X,r2
		$display("store 57,X,2");
		#20
		#2000
		$display("After 100 CC	%b | %d (takes 100CC to Fetch from DM )\n",Result,Result);
		
		Read_Addr = 8'd3;//store 56,X,r3
		$display("store 56,X,3");
		#20
		$display("After 1 CC	%b | %d \n",Result,Result );
		
		Read_Addr = 8'd4;//load r7,X,57
		$display("load 7,X,57");
		#20
		$display("After 1 CC	%b | %d (It will be a Instruction Read miss.)",Result,Result);
		#2000
		#20
		$display("After 100 CC	%b | %d (Earliar it took 100CC  for Read from DM now it is 1CC)\n",Result,Result);
		Read_Addr = 8'd5;//load r8,X,56
		$display("load 8,X,56");
		#20
		$display("After 1 CC	%b | %d (Earliar it took 100CC for Read from DM now it is 1CC)\n",Result,Result);
		
		Read_Addr = 8'd6;//loadi r3,X,67
		$display("loadi 3,X,67");
		#20
		$display("After 1 CC	%b | %d \n",Result,Result);
		
		Read_Addr = 8'd7;//store 24,X,r3
		$display("store 24,X,3");
		#20
		$display("After 1 CC	%b | %d (It will be a Write miss)",Result,Result);
		#2000
		$display("After 100 CC	%b | %d (takes 100CC to Write-Back to DM )",Result,Result);
		#2000
		$display("After 200 CC	%b | %d (takes 100CC to Fetch from DM)\n",Result,Result);
		
		Read_Addr = 8'd8;//load r8,X,24
		$display("load 8,X,24");
		#20
		$display("After 1 CC	%b | %d (It will be a Instruction Read miss.)",Result,Result);
		#2000
		#20
		$display("After 100 CC	%b | %d (Its a Read hit takes 1CC)\n",Result,Result);
		
		Read_Addr = 8'd9;//add 5,7,8
		$display("add 5,7,8");
		#20
		$display("After 1 CC	%b | %d \n",Result,Result);
		Read_Addr = 8'd10;//sub 4,8,7
		$display("sub 4,8,7");
		#20
		$display("After 1 CC	%b | %d (It will be a Instruction Read hit takes 1CC.)\n",Result,Result);

		Read_Addr = 8'd11;//load r8,X,56
		$display("load 8,X,56");
		#20
		$display("After 1 CC	%b | %d (It should be 67. It is a Read miss)",Result,Result);
		#2000
		$display("After 100 CC	%b | %d (It should be 67. Takes 100CC to Write Back to DM because Dirty bit is set)",Result,Result);
		#2000
		$display("After 200 CC	%b | %d (It should be 67. Takes 100CC to Fetch from DM)\n",Result,Result);
		
		Read_Addr = 8'd12;//load r8,X,57
		$display("load 8,X,57");
		#20
		$display("After 1 CC	%b | %d (It will be a Instruction Read miss.)",Result,Result);
		#2000
		#20
		$display("After 100 CC	%b | %d (Its a Read hit takes 1CC)\n",Result,Result);
		
		Read_Addr = 8'd13;//store 57,X,r3
		$display("store 57,X,3");
		#20
		$display("After 1 CC	%b | %d (It will be a Write hit take 1CC)\n",Result,Result );
		
		Read_Addr = 8'd14;//load r8,X,24
		$display("load 8,X,24");
		#20
		$display("After 1 CC	%b | %d (It should be 97. It is a Read miss )",Result,Result);
		#2000
		$display("After 100 CC	%b | %d (It should be 97. Takes 100CC to Write Back to DM )",Result,Result);
		#2000
		$display("After 200 CC	%b | %d (It should be 97. Takes 100CC to Fetch from DM)\n",Result,Result);
		
		Read_Addr = 8'd15;//store 25,X,r2
		$display("store 25,X,2");
		#20
		$display("After 1 CC	%b | %d (It will be a  Write hit)\n",Result,Result);
		
		Read_Addr = 8'd16;//add 5,r7,r2
		$display("add 5,7,2");
		#20
		$display("After 1 CC	%b | %d (It will be a Instruction Read miss.)",Result,Result);
		#2000
		#20
		$display("After 100 CC	%b | %d \n",Result,Result);
		
		$finish;
	end

endmodule
