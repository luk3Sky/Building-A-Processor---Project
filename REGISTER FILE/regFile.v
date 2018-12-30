/*
	 ____________Register File_____________
	         E/15/154 | E/15/142
      _____________________________
              CO224-Lab5
*/

module testbench;

	reg [2:0] INaddr,OUT1addr,OUT2addr;
	reg clk;
	reg [7:0] IN;
	wire [7:0] OUT1,OUT2;
	wire [7:0] RESULT;

	regfile8x8a regf ( clk, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2);
	initial begin
	clk = 1'b0; end
	always #10 clk = ~clk;

	initial begin
	#5//T=5
		IN = 12;
		INaddr = 5;
		OUT1addr = 5;
		OUT2addr = 3;
    $display("OUT1addr = %d OUT2addr = %d INaddr = %d IN = %d",OUT1addr,OUT2addr,INaddr,IN);
	#10//T=15
		$display("OUT1 = %d OUT2 = %d",OUT1,OUT2);
	#20//T=35
		$display("OUT1 = %d OUT2 = %d",OUT1,OUT2);
		IN = 10;
		INaddr = 3;
	#10//T=45
		$display("OUT1 = %d OUT2 = %d",OUT1,OUT2);
	#10//T=55
		$display("OUT1 = %d OUT2 = %d",OUT1,OUT2);
	$finish;
	end
endmodule

// ******** Register File ********
module regfile8x8a ( clk, INaddr, IN, OUT1addr, OUT1, OUT2addr, OUT2);

	input [2:0] OUT1addr,OUT2addr,INaddr;
	input [7:0] IN;
	input clk;
	output reg [7:0] OUT1,OUT2;

	reg [63:0] regMemory = 0;
	reg [7:0] OUT1reg, OUT2reg;
	integer i;

	always @ ( * ) begin
		OUT1 = OUT1reg[7:0];
		OUT2 = OUT2reg[7:0];
	end


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

endmodule