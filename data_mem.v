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