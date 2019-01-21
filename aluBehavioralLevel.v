/*  op-code = 8 bit
    dest    = 8 bit
    src1    = 8 bit
    src2   = 8 bit
    control = 3 bit
*/


//testbench
module testbench;
	reg [7:0] DATA1,DATA2;	//Inputs
	reg [2:0] SELECT;		//Control Input
	wire [7:0] RESULT;		//Output

	alu ALU(RESULT,DATA1,DATA2,SELECT);

	initial
	begin
		DATA1=8'b00000101;
		DATA2=8'b00000011;

		SELECT=3'b000;
		#1 $display("FORWARD : Input value = %d , FORWARD value = %d",DATA1,RESULT);
		SELECT=3'b001;
		#1 $display("ADD : %d + %d = %d",DATA1,DATA2,RESULT);
		SELECT=3'b010;
		#1 $display("AND : %b and %b = %b",DATA1,DATA2,RESULT);
		SELECT=3'b011;
		#1 $display("OR : %b or %b = %b",DATA1,DATA2,RESULT);
    SELECT=3'b111;
    #1 $display("DEFAULT CASE: %d",RESULT);
	end
endmodule

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
