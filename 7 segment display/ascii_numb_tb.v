`timescale 1ns / 1ps

module ascii_numb_tb();
    
    reg  [7:0] AsciiCode;
    wire [6:0] HexSeg;

    ASCII7Seg AI (AsciiCode, HexSeg); // initial block sets up all values of regs to create the cases for the truth tables

    initial begin // a 10 unit time delay is created by #10
        AsciiCode = 8'h30; #10;  // '0'
        AsciiCode = 8'h31; #10;  // '1'
        AsciiCode = 8'h32; #10;  // '2'
        AsciiCode = 8'h33; #10;  // '3'
        AsciiCode = 8'h34; #10;  // '4'
        AsciiCode = 8'h35; #10;  // '5'
        AsciiCode = 8'h36; #10;  // '6'
        AsciiCode = 8'h37; #10;  // '7'
        AsciiCode = 8'h38; #10;  // '8'
        AsciiCode = 8'h39; #10;  // '9'
    end

endmodule

