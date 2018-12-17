module regfile8x8a(IN, OUT1, OUT2, INaddr, OUT1addr, OUT2addr, CLK, RESET, CTRL);
    input reg [7:0] IN;
    output reg [7:0] OUT1;
    output reg [7:0] OUT2;
    input reg [2:0] INaddr;
    input reg [2:0] OUT1addr;
    input reg [2:0] OUT2addr;
    input CLK;
    input RESET;
    input CTRL;

    reg [7:0] 	 register0, register1, register2, register3, register4, register5, register6, register7;
    
    // Write functionality 
    always @(negedge clk ) begin
      if (CTRL)
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
    end // always @ (posedge clk)

    // Read functionality
    always @(posedge clk ) begin
      
    end
endmodule