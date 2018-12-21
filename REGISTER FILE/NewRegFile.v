module testbench();
    reg clk,ctrl,reset;
    wire [7:0] OUT1,OUT2;
    reg [7:0] IN;
    reg [2:0] INaddr,OUT1addr,OUT2addr;

    // Initialize all variables
    initial begin        
        $display ("time\t clk reset ctrl in out1 out2");	
        $monitor ("%g\t  %d    %d   %d   %d  %d  %d", $time, clk, reset, ctrl, IN, OUT1, OUT2);

        clk = 0;       // initial value of clock
        reset = 0;       // initial value of reset
        ctrl = 1;
        
        //write
        #5 ctrl = 0;
        #5 @(negedge clk) INaddr = 2;
        #5 @(negedge clk) IN = 200;

        #5 @(negedge clk) INaddr = 4;
        #5 @(negedge clk) IN = 100;


        $dumpfile("R.vcd");
	      $dumpvars(0, regF);

        //read 
        #5 ctrl=1;
        #5 @(posedge clk) OUT1addr = 2;
        #5 @(posedge clk) OUT2addr = 4;
        
        //reset
        #5 reset=1;

        #5 @(posedge clk) OUT1addr = 2;
        #5 @(posedge clk) OUT2addr = 4;

        // off reset
        #5 reset=0;

        //write
        #5 ctrl=0;
        #5 @(negedge clk) INaddr = 2;
        #5 @(negedge clk) IN = 33;
        
        //read
        #5 ctrl=1;
        #5 @(posedge clk) OUT1addr = 2;
        #5 @(posedge clk) OUT2addr = 4;

        #5 $finish;      // Terminate simulation
    end

    // Clock generator
    always begin
        #5 clk = ~clk; // Toggle clock every 5 ticks
    end

    //connect reg-file with testbench
    regfile8x8a regF(IN, OUT1, OUT2, INaddr, OUT1addr, OUT2addr, clk, reset, ctrl);

endmodule

module regfile8x8a(IN, OUT1, OUT2, INaddr, OUT1addr, OUT2addr, CLK, RESET, CTRL);
    input [7:0] IN;
    output reg [7:0] OUT1;
    output reg [7:0] OUT2;
    input [2:0] INaddr;
    input [2:0] OUT1addr;
    input [2:0] OUT2addr;
    input CLK;
    input RESET;
    input CTRL;

    reg [7:0] 	 register0, register1, register2, register3, register4, register5, register6, register7;
    
    //RESET
    always @(*)
    if (RESET==1)
      begin
        register0<=0;
        register1<=0;
        register2<=0;
        register3<=0;
        register4<=0;
        register5<=0;
        register6<=0;
        register7<=0;
      end
        
      
    // Write functionality 
    always @(negedge CLK ) begin
      if (CTRL==0)  begin
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
      if (CTRL==1) begin
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
    end
endmodule