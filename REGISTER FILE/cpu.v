/*
	 ________________CPU________________
	         E/15/154 | E/15/142
      _____________________________
              CO224-Lab5
*/

//2's complement
module two_s_complement (IN,OUT);
  input signed [7:0] IN;
  output signed [7:0] OUT;

  assign OUT = ~IN + 8'b00000001;

endmodule //two_s_complement

//2:1 Multiplexer
module mux (IN1,IN2,SELECT,OUT,CLK);
  input [7:0] IN1,IN2;
  input SELECT,CLK;
  output reg [7:0] OUT;

  always @ (negedge CLK) begin
    case (SELECT)
      0: OUT<=IN1;
      1: OUT<=IN2;
      default: OUT<=0;
    endcase
  end

endmodule //mux 2 to 1

//Program Counter
module PC (RESET,CLK,COUNT);
  input RESET,CLK;
  output reg [7:0] COUNT;

  always @ (negedge CLK) begin
    case (RESET)
      0: COUNT=COUNT+1;
      1: COUNT=0;
      default: COUNT<=COUNT;
    endcase
  end

endmodule // PC

/*
  ____TEST-BENCHES_____
*/
module testbench_for_two_s;
  reg [7:0] IN;
  wire [7:0] OUT;

  two_s_complement ts(IN,OUT);

  initial begin
    IN=8'b00000010;
    #1 $display ("2s complement of %b is %b.",IN,OUT);
  end
endmodule // testbench_for_two_s

module testbench_for_mux;
  reg [7:0] IN1,IN2;
  wire [7:0] OUT;
  reg SELECT,CLK;

  initial begin
  //initial values
    CLK=0;
    SELECT=0;
    IN1=5;
    IN2=10;

  //time=3
    #3 $display ("OUT = %d",OUT);
  //time=6
    #6 $display ("OUT = %d",OUT);
  //time=9
    #9 SELECT=1;
    #9 $display ("OUT = %d",OUT);
  //time=12
    #12 $display ("OUT = %d",OUT);
    $finish;
  end

  // Clock generator
  always begin
    #5 CLK = ~CLK; // Toggle clock every 5 ticks
  end

  mux two_to_one_mux(IN1,IN2,SELECT,OUT,CLK);

endmodule // testbench_for_mux

module testbench_for_PC;
  reg RESET,CLK;
  reg COUNT;

  initial begin
  //initial values
    RESET=0;
    COUNT=0;
    CLK=0;
  //time=3
    #3 $display("Counter = %d",COUNT);
  //time=6
    #6 $display("Counter = %d",COUNT);
  //time=9
    #9 RESET=1;
    #9 $display ("Counter = %d",COUNT);
  //time=12
    #12 $display ("Counter = %d",COUNT);
    $finish;
  end

  // Clock generator
  always begin
    #5 CLK = ~CLK; // Toggle clock every 5 ticks
  end

endmodule // testbench_for_PC