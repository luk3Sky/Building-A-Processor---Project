
/*
	   __________4-bit Combinational ALU__________
	               E/15/154 | E/15/142
             _____________________________
		             CO224-Lab5
	

*/



module testbench;
	reg [7:0] A,B;		//Inputs 
	reg [2:0] C;		//Control Inputs
	wire [7:0] F;		//Outputs 	
    wire cout;

	ALU alu(A,B,C,F,cout);	

	initial 
	begin 
		A=4'b0101;
		B=4'b0011;

		// C = 3'b000;						//--------------------------2s complement of A
		// #1 $display("\t2's complement of A (%b) is %b \n",A,F);
		// C=3'b001;						//--------------------------2s complement of B
		// #1 $display("\t2's complement of B (%b) is %b \n",B,F);
		C=3'b010;						//--------------------------A+B
		#1 $display("\tAddition : %d + %d = %d , Carry out = %d\n",A,B,F,cout);
		C=3'b011;						//--------------------------A-B
		#1 $display("\tSubstraction : %d - %d = %d \n",A,B,F);
		C=3'b100;						//--------------------------A AND B
		#1 $display("\tAND : %b and %b = %b \n",A,B,F);
		C=3'b101;						//--------------------------A OR B
		#1 $display("\tOR : %b or %b = %b \n",A,B,F);
		// C=3'b110;						//--------------------------A * B
		//#1 $display("\tMultiplication : %d * %d = %d \n",A,B,F);
		// C=3'b111;						//--------------------------A xor B
		// #1 $display("\tXOR : %b xor %b = %b \n",A,B,F);
	end
endmodule
//--------------------------------------------------------------------end of test bench

//--------------------------------------------------------------------ALU module
module ALU(A,B,C,out,cout);

	input [7:0] A,B;
	input [2:0] C;

	output [7:0] out;
    output cout;


	wire [7:0] a_2s,b_2s;
	wire [7:0] AplusB,AminB;
	wire [7:0] AandB,AorB;
	wire [7:0] AmultiB;
	wire [7:0] X_OR;
	

	two_sComp tc1(A,a_2s);  //2s complement of A
	
	two_sComp tc2(B,b_2s);  //2s complement of B

	adder8bit fba1(A,B,1'b0,AplusB,cout);   //+
	
	substractor s1(A,B,AminB);  //-

	bitWise_and bwa1(A,B,AandB);    //and

	bitWise_or bwo1(A,B,AorB);  //or

	//multiplier m1(A,B,AmultiB); //*
	
	xor_ing x_or1(A,B,X_OR);  //xor_ing Checker

    /*
        get output using 8:1 MUX    
    */

	MUX8to1 mux1(a_2s[0],b_2s[0],AplusB[0],AminB[0],AandB[0],AorB[0],AmultiB[0],X_OR[0],C,out[0]);
	MUX8to1 mux2(a_2s[1],b_2s[1],AplusB[1],AminB[1],AandB[1],AorB[1],AmultiB[1],X_OR[1],C,out[1]);
	MUX8to1 mux3(a_2s[2],b_2s[2],AplusB[2],AminB[2],AandB[2],AorB[2],AmultiB[2],X_OR[2],C,out[2]);
	MUX8to1 mux4(a_2s[3],b_2s[3],AplusB[3],AminB[3],AandB[3],AorB[3],AmultiB[3],X_OR[3],C,out[3]);
	MUX8to1 mux5(a_2s[4],b_2s[4],AplusB[4],AminB[4],AandB[4],AorB[4],AmultiB[4],X_OR[4],C,out[4]);
	MUX8to1 mux6(a_2s[5],b_2s[5],AplusB[5],AminB[5],AandB[5],AorB[5],AmultiB[5],X_OR[5],C,out[5]);
	MUX8to1 mux7(a_2s[6],b_2s[6],AplusB[6],AminB[6],AandB[6],AorB[6],AmultiB[6],X_OR[6],C,out[6]);
	MUX8to1 mux8(a_2s[7],b_2s[7],AplusB[7],AminB[7],AandB[7],AorB[7],AmultiB[7],X_OR[7],C,out[7]);

endmodule
//----------------------------------------------------------------------------------------------

//-----------------------------not module
module inverter(x, notx);

	input [7:0] x;
	output [7:0] notx;
	
	not (notx[0], x[0]);
	not (notx[1], x[1]);
	not (notx[2], x[2]);
	not (notx[3], x[3]);
	not (notx[4], x[4]);
	not (notx[5], x[5]);
	not (notx[6], x[6]);
	not (notx[7], x[7]);

endmodule
//---------------------------------------------------------------------------------------------

//-----------------------------2s complement
module two_sComp(a,out);			

	input [7:0] a;
	output [7:0] out;
	wire [7:0] na;
	wire cout;

	inverter in1(a,na);
	adder8bit fba2(na,1'b1,1'b0,out,cout);

endmodule
//---------------------------------------------------------------------------------------------

//---------------------------------------4-bit adder
module adder8bit(a,b,cin,s,cout);

	input  [7:0] a, b; input cin;
	output [7:0] s; output cout;

	wire cout1, cout2, cout3, cout4, cout5, cout6, cout7;



	FullAdder fa0(a[0],b[0],cin,s[0],cout1);
	FullAdder fa1(a[1],b[1],cout1,s[1],cout2);
	FullAdder fa2(a[2],b[2],cout2,s[2],cout3);
	FullAdder fa3(a[3],b[3],cout3,s[3],cout4);
	FullAdder fa4(a[4],b[4],cout4,s[4],cout5);
	FullAdder fa5(a[5],b[5],cout5,s[5],cout6);
	FullAdder fa6(a[6],b[6],cout6,s[6],cout7);
	FullAdder fa7(a[7],b[7],cout7,s[7],cout);


endmodule
//---------------------------------------------------------------------------------------------

//-------------------------------1-bit adder
module FullAdder(a,b,cin,s,cout);
	
	//---------------i/p o/p declaration
	input a,b,cin; 
	output s,cout;
	
	//---------------internal wires
	wire s1,c1,s2;

	//---------------gate level implementation for 1-bit adder
	xor x1(s1, a, b);
	and a1(c1, a, b);
	xor x2(s, s1, cin);
	and a2(s2, s1, cin);
	xor x3(cout, s2, c1);
	

endmodule
//---------------------------------------------------------------------------------------------


//-----------------------------substractor
module substractor(a,b,out);			
	
	input [3:0] a,b;
	output [3:0] out;
	wire [3:0] tcb;
	wire cout;

	two_sComp tc3(b,tcb);
	adder8bit fba3(a,tcb,1'b0,out,cout);
	

endmodule
//---------------------------------------------------------------------------------------------

//------------------------------and
module bitWise_and(a,b,out);			

        input [7:0] a,b;
        output [7:0] out;

        and(out[0],a[0],b[0]);
        and(out[1],a[1],b[1]);
        and(out[2],a[2],b[2]);
        and(out[3],a[3],b[3]);
		and(out[4],a[4],b[4]);
		and(out[5],a[5],b[5]);
		and(out[6],a[6],b[6]);
		and(out[7],a[7],b[7]);

endmodule
//----------------------------------------------------------------------------------------------

//-------------------------------or
module bitWise_or(a,b,out);			

        input [7:0] a,b;
        output [7:0] out;

        or(out[0],a[0],b[0]);
        or(out[1],a[1],b[1]);
        or(out[2],a[2],b[2]);
        or(out[3],a[3],b[3]);
		or(out[4],a[4],b[4]);
		or(out[5],a[5],b[5]);
		or(out[6],a[6],b[6]);
		or(out[7],a[7],b[7]);

endmodule
//----------------------------------------------------------------------------------------------

//-------------------------------and_4bit
module AND4bit(a,b,out);
	
	input [7:0] a;
	input b;
	output [7:0] out;

	and(out[0],a[0],b);
	and(out[1],a[1],b);
	and(out[2],a[2],b);
	and(out[3],a[3],b);
	and(out[4],a[4],b);
	and(out[5],a[5],b);
	and(out[6],a[6],b);
	and(out[7],a[7],b);

endmodule
//----------------------------------------------------------------------------------------------

// //-------------------------------*
// module multiplier(a,b,out);			
	
// 	input [3:0] a,b;
// 	output [3:0] out;
// 	wire [3:0] and1,and2,and3,and4;
// 	wire s1,s2,s3,cout1,cout2,cout3,cout4,cout5,cout6;

// 	AND4bit ab1(a,b[0],and1);
// 	AND4bit ab2(a,b[1],and2);
// 	AND4bit ab3(a,b[2],and3);
// 	AND4bit ab4(a,b[3],and4);
	
// 	assign out[0]=and1[0];

// 	FullAdder fa5(and1[1],and2[0],1'b0,out[1],cout1);

// 	FullAdder fa6(and1[2],and2[1],cout1,s1,cout2);
// 	FullAdder fa7(and3[0],s1,cout2,out[2],cout3);

// 	FullAdder fa8(and1[3],and2[2],cout3,s2,cout4);
// 	FullAdder fa9(and3[1],s2,cout4,s3,cout5);
// 	FullAdder fa10(and4[0],s3,cout5,out[3],cout6);

// endmodule
// //----------------------------------------------------------------------------------------------

//--------------------------------
module xor_ing(a,b,out);		

	input [7:0] a,b;
	output [7:0] out;
	//wire xor1,xor2,xor3,xor4,xor5,xor6;

	xor(out[0],a[0],b[0]);
	xor(out[1],a[1],b[1]);
	xor(out[2],a[2],b[2]);
	xor(out[3],a[3],b[3]);
	xor(out[4],a[4],b[4]);
	xor(out[5],a[5],b[5]);
	xor(out[6],a[6],b[6]);
	xor(out[7],a[7],b[7]);

endmodule
//----------------------------------------------------------------------------------------------

//---------------------------------MUX 8:1
module MUX8to1(i1,i2,i3,i4,i5,i6,i7,i8,c,out);		

	input i1,i2,i3,i4,i5,i6,i7,i8;
	input [2:0] c;
	output out;
	wire [2:0] nc;
	wire w1,w2,w3,w4,w5,w6,w7,w8;

	not(nc[0],c[0]);
	not(nc[1],c[1]);
	not(nc[2],c[2]);

	and(w1,i1,nc[2],nc[1],nc[0]);
    and(w2,i2,nc[2],nc[1],c[0]);
    and(w3,i3,nc[2],c[1],nc[0]);
    and(w4,i4,nc[2],c[1],c[0]);
	and(w5,i5,c[2],nc[1],nc[0]);
    and(w6,i6,c[2],nc[1],c[0]);
    and(w7,i7,c[2],c[1],nc[0]);
    and(w8,i8,c[2],c[1],c[0]);

	or(out,w1,w2,w3,w4,w5,w6,w7,w8);

endmodule
//----------------------------------------------------------------------------------------------
