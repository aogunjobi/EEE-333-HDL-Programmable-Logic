// 4-bit ALU
module ALU (input [3:0] aluin_a, aluin_b, OPCODE, input Cin, output reg [3:0] alu_out, output reg Cout, output OF);
    reg [3:0] Ain, Bin;
    reg Ci;
    wire [3:0] Bn, S;
    wire Co, OF_w;

    // 2's complement of aluin_b
    com2s C1 (aluin_b, Bn);

    // 4-bit ripple-carry adder
    FA4 FA1 (Ain, Bin, Ci, S, Co, OF_w);

    // Pass the ripple adder overflow out
    assign OF = OF_w; // Overflow only non-zero in arithmetic ops

    always @ (*) begin
        // Default values
        Ain     = 4'b0000;
        Bin     = 4'b0000;
        alu_out = 4'b0000;
        Ci      = 1'b0;
        Cout    = 'b0;

        case (OPCODE)
        // A + B with Cin
        4'b0001 : begin
            Ain     = aluin_a;
            Bin     = aluin_b;
            Ci      = Cin;
            alu_out = S;
            Cout    = Co;
        end

        // A + B without Cin
        4'b0010 : begin
            Ain     = aluin_a;
            Bin     = aluin_b;
            alu_out = S;
            Cout    = Co;
        end

        // A - B
        4'b0011 : begin
            Ain     = aluin_a;
            Bin     = Bn;    // 2's complement of B
            Ci      = 1'b1;  // Add 1 for subtraction
            alu_out = S;
            Cout    = Co;
        end

        // NAND ~(aluin_a & aluin_b)
        4'b0100 : begin
            alu_out = ~(aluin_a & aluin_b);
            Cout    = 0;     // Logic => no carry
            // OF remains 0 because the adder sees zero inputs
        end

        // OR (aluin_a | aluin_b)
        4'b0101 : begin
            alu_out = (aluin_a | aluin_b);
            Cout    = 0;     // Logic => no carry
        end

        // XOR (aluin_a ^ aluin_b)
        4'b0110 : begin
            alu_out = aluin_a ^ aluin_b;
            Cout    = 0;     // Logic => no carry
        end

        // NOT ~aluin_a
        4'b0111 : begin
            alu_out = ~aluin_a;
            Cout    = 0;     // Logic => no carry
        end

        // Right Shift aluin_a >> 1
        4'b1000 : begin
            alu_out = aluin_a >> 1;
            Cout    = 0;     // Logic => no carry
        end

        // Default (NOP)
        default : begin
            alu_out = 4'b0000;
            Cout    = 0;
        end
        endcase
    end
endmodule

// 1-bit adder
module FA (a, b, cin, sum, cout);
    input a;
    input b;
    input cin;
    output cout;
    output sum;

    assign sum  = (a ^ b) ^ cin;
    assign cout = (a & b) | (cin & a) | (cin & b);
endmodule

// 2's complement
module com2s (input [3:0] B, output [3:0] Bn);
    wire [3:0] Bn1;
    wire OF;

    assign Bn1 = ~B;
    // Add 1 to ~B
    FA4 fa1 (Bn1, 4'b0000, 1'b1, Bn, Cout, OF);
endmodule

// 4-bit ripple-carry adder
module FA4 (input [3:0] A, B, input Cin, output [3:0] Sum, output Cout, output OF);
    wire Cout1, Cout2, Cout3;

    FA fa1 (A[0], B[0], Cin,   Sum[0], Cout1);
    FA fa2 (A[1], B[1], Cout1, Sum[1], Cout2);
    FA fa3 (A[2], B[2], Cout2, Sum[2], Cout3);
    FA fa4 (A[3], B[3], Cout3, Sum[3], Cout);

    assign OF = (A[3] & B[3] & ~Sum[3]) | (~A[3] & ~B[3] & Sum[3]);
endmodule

// Half adder
module HA (input A, B, output Sum, Cout);
    assign Sum  = A ^ B;
    assign Cout = A & B;
endmodule

// ALU for physical validation (hardcodes B=4'b0011)

module ALU_pv (input [3:0] aluin_a, OPCODE, input Cin, output [3:0] alu_out, output reg Cout, output OF);
    wire [3:0] aluin_b;
    assign aluin_b = 4'b0011;
    
    // Explicitly define wires for ALU outputs
    wire [3:0] alu_out_w;
    wire Cout_w;
    wire OF_w;
    
    ALU alu2 (aluin_a, aluin_b, OPCODE, Cin, alu_out_w, Cout_w, OF_w);
    
    // Assign internal wires to module outputs
    assign alu_out = alu_out_w;
    assign Cout = Cout_w;
    assign OF = OF_w;
endmodule


