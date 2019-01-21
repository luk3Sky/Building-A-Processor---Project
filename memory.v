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


always @(*)
begin
    if (write && !read)
	begin
		busy_wait <= 1;
		// artificially delay 100 cycles
        memory_ram_d[address] = write_data;
		busy_wait <= 0;
	end
    if (!write && read)
	begin
		busy_wait <= 1;
		// artificially delay 100 cycles
        read_data = memory_ram_q[address];
		busy_wait <= 0;
	end
end

endmodule
