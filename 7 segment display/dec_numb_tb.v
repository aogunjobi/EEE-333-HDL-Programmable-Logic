`timescale 1ns / 1ps

// Display numbers from binary format
module dec_numb_tb();
    
    reg  [7:0] Decimal;
    wire [6:0] HexSeg;

    Dec27Seg AI (Decimal, HexSeg);

    initial begin
		Decimal = 8'd0; #10; //0
		Decimal = 8'd1; #10; //1
		Decimal = 8'd2; #10; //2
		Decimal = 8'd3; #10; //3
		Decimal = 8'd4; #10; //4
		Decimal = 8'd5; #10; //5
		Decimal = 8'd6; #10; //6
		Decimal = 8'd7; #10; //7
		Decimal = 8'd8; #10; //8
		Decimal = 8'd9; #10; //9
    end

endmodule

