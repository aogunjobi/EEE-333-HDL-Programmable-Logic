module testFA();
reg A, B, Cin;
FA fa1 (A, B, Cin, S, Cout);
initial begin
    A=0; B=0; Cin=0; #5;
    A=1; B=0; #5;
    A=0; B=1; #5;
    A=1; B=1; #5;
    A=0; B=0; Cin=1; #5;
    A=1; B=0; #5;
    A=0; B=1; #5;
    A=1; B=1; #5;
end
endmodule

module testALU();
reg [3:0] aluin_a, aluin_b, OPCODE;
wire [3:0] alu_out;
reg Cin;
wire Cout, OF;
initial begin
    // add
    aluin_a=4'b0011; aluin_b=4'b0011; Cin=0; OPCODE=4'b0010; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // add with Cin
    aluin_a=4'b0110; aluin_b=4'b0101; Cin=1; OPCODE=4'b0001; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // sub b from a
    aluin_a=4'b0111; aluin_b=4'b0110; Cin=0; OPCODE=4'b0011; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // Bitwise NAND
    aluin_a=4'b0111; aluin_b=4'b1010; Cin=0; OPCODE=4'b0100; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // Bitwise OR
    aluin_a=4'b0111; aluin_b=4'b0011; Cin=0; OPCODE=4'b0101; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // Bitwise XOR
    aluin_a=4'b0101; aluin_b=4'b1110; Cin=0; OPCODE=4'b0110; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // Bitwise NOT
    aluin_a=4'b1011; aluin_b=4'b0000; Cin=0; OPCODE=4'b0111; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // Logical right shift
    aluin_a=4'b0101; aluin_b=4'b0000; Cin=0; OPCODE=4'b1000; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
    // Function given
    aluin_a=4'b0100; aluin_b=4'b0011; Cin=0; OPCODE=4'b1111; #5;
    $display("A= %b, B=%b, C= %b, Sum= %b, OPCODE= %b, Cout= %b, OF=%b", aluin_a, aluin_b, Cin, alu_out, OPCODE, Cout, OF);
end
ALU alu1 (aluin_a, aluin_b, OPCODE, Cin, alu_out, Cout, OF);
endmodule





