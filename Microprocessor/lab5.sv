module lab5_pv(
    input  logic        clk,
    input  logic        SW0,
    input  logic        SW1,
    input  logic        KEY0,
    input  logic        SW2,
    input  logic        SW3,
    input  logic        SW4,
    output logic [6:0]  HEX5,
    output logic [6:0]  HEX4,
    output logic [6:0]  HEX3,
    output logic [6:0]  HEX2,
    output logic [6:0]  HEX1,
    output logic [6:0]  HEX0,
    output logic        LED0,
    output logic        LED1,
    output logic        LED2,
    output logic        LED3,
    output logic        LED4,
    output logic        LED5,
    output logic        LED6,
    output logic        LED7
);
  logic        clkdiv;
  logic [24:0] count;
  logic [3:0]  OPCODE;
  logic [7:0]  PC, alu_out, W_REG;
  logic [7:0]  disp0, disp1, disp2, disp3, disp4, disp5;

  pmcntr#(25) u_cnt(clk, SW0, 25'd50000, count, clkdiv);
  lab5 u_cpu(clkdiv, SW0, SW1, KEY0, SW2, SW3, SW4, OPCODE, PC, alu_out, W_REG);

  always_ff @(posedge clk or posedge SW0) begin
    if (SW0) begin
      disp5 <= "_"; disp4 <= "_"; disp3 <= "_";
      disp2 <= "_"; disp1 <= "_"; disp0 <= "_";
    end else begin
      case ({SW4,SW3,SW2})
        3'b000: begin
          // Display "OGUNJO" left-justified
          disp5 <= "O";
          disp4 <= "G";
          disp3 <= "U";
          disp2 <= "N";
          disp1 <= "J";
          disp0 <= "O";
        end
        3'b110: begin
          // Display PC (right-justified)
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_"; disp2 <= "_";
          disp1 <= {4'd0, PC[7:4]};
          disp0 <= {4'd0, PC[3:0]};
        end
        3'b101: begin
          // Display W_REG
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_"; disp2 <= "_";
          disp1 <= {4'd0, W_REG[7:4]};
          disp0 <= {4'd0, W_REG[3:0]};
        end
        3'b011: begin
          // Display ALU_OUT
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_"; disp2 <= "_";
          disp1 <= {4'd0, alu_out[7:4]};
          disp0 <= {4'd0, alu_out[3:0]};
        end
        3'b111: begin
          // Display OPCODE (bitwise)
          disp5 <= "_"; disp4 <= "_";
          disp3 <= {4'd0, OPCODE[3]};
          disp2 <= {4'd0, OPCODE[2]};
          disp1 <= {4'd0, OPCODE[1]};
          disp0 <= {4'd0, OPCODE[0]};
        end
        default: begin
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_";
          disp2 <= "_"; disp1 <= "_"; disp0 <= "_";
        end
      endcase
    end
  end

  assign {LED7,LED6,LED5,LED4,LED3,LED2,LED1,LED0} = PC;

  ASCII7Seg u0(disp0, HEX0);
  ASCII7Seg u1(disp1, HEX1);
  ASCII7Seg u2(disp2, HEX2);
  ASCII7Seg u3(disp3, HEX3);
  ASCII7Seg u4(disp4, HEX4);
  ASCII7Seg u5(disp5, HEX5);
endmodule



/*//------------------------------------------------------------------------------
// Top-level physical validation 
//------------------------------------------------------------------------------
module lab5_pv(
    input        clk,
    input        SW0,
    input        SW1,
    input        KEY0,
    input        SW2,
    input        SW3,
    input        SW4,
    output logic [6:0] HEX5, HEX4, HEX3, HEX2, HEX1, HEX0,
    output logic      LED0, LED1, LED2, LED3, LED4, LED5, LED6, LED7
);
  logic        clkdiv;
  logic [24:0] count;
  logic [3:0]  OPCODE;
  logic [7:0]  PC, alu_out, W_REG;
  logic [7:0]  disp0, disp1, disp2, disp3, disp4, disp5;

  // 50MHz -> ~1kHz divider
  pmcntr #(25) u_cnt(clk, SW0, 25'd50000, count, clkdiv);

  // CPU core
  lab5 u_cpu(
    clkdiv, SW0, SW1, KEY0, SW2, SW3, SW4,
    OPCODE, PC, alu_out, W_REG
  );

  // 7-segment drivers
  ASCII27Seg u0(disp0, HEX0);
  ASCII27Seg u1(disp1, HEX1);
  ASCII27Seg u2(disp2, HEX2);
  ASCII27Seg u3(disp3, HEX3);
  ASCII27Seg u4(disp4, HEX4);
  ASCII27Seg u5(disp5, HEX5);

  // display selection
  always_ff @(posedge clk or posedge SW0) begin
    if (SW0) begin
      // on reset, blank display
      disp5 <= "_"; disp4 <= "_"; disp3 <= "_";
      disp2 <= "_"; disp1 <= "_"; disp0 <= "_";
    end
    else begin
      case ({SW4, SW3, SW2})
        3'b000: begin
          disp5 <= "O";
          disp4 <= "G";
          disp3 <= "U";
          disp2 <= "N";
          disp1 <= "J";
          disp0 <= "O";
        end

        3'b110: begin
          // display PC (right-justified)
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_"; disp2 <= "_";
          disp1 <= {4'd0, PC[7:4]};
          disp0 <= {4'd0, PC[3:0]};
        end

        3'b101: begin
          // display W_REG
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_"; disp2 <= "_";
          disp1 <= {4'd0, W_REG[7:4]};
          disp0 <= {4'd0, W_REG[3:0]};
        end

        3'b011: begin
          // display alu_out
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_"; disp2 <= "_";
          disp1 <= {4'd0, alu_out[7:4]};
          disp0 <= {4'd0, alu_out[3:0]};
        end

        3'b111: begin
          // display OPCODE (4 bits)
          disp5 <= "_"; disp4 <= "_";
          disp3 <= {4'd0, OPCODE[3]};
          disp2 <= {4'd0, OPCODE[2]};
          disp1 <= {4'd0, OPCODE[1]};
          disp0 <= {4'd0, OPCODE[0]};
        end

        default: begin
          // all other combos: blank
          disp5 <= "_"; disp4 <= "_"; disp3 <= "_";
          disp2 <= "_"; disp1 <= "_"; disp0 <= "_";
        end
      endcase
    end
  end

  // LEDs = PC
  assign {LED7, LED6, LED5, LED4, LED3, LED2, LED1, LED0} = PC;
endmodule*/

//------------------------------------------------------------------------------
// Testbench
//------------------------------------------------------------------------------
module lab5_tb();
  logic clk, SW0, SW1, KEY0, SW2, SW3, SW4;
  logic [3:0]  OPCODE;
  logic [7:0]  PC, alu_out, W_REG;

  lab5 u_cpu(clk, SW0, SW1, KEY0, SW2, SW3, SW4,
             OPCODE, PC, alu_out, W_REG);

  initial begin
    clk = 0; SW0 = 1; SW1 = 0; SW2=0; SW3=0; SW4=0; KEY0=1;
    #20 SW0 = 0;
    repeat (500) #5 clk = ~clk;
    $finish;
  end
endmodule

/*//---------------------------------------------------------------------------
// Testbench with CSV output
//---------------------------------------------------------------------------
module lab5_tb();
  logic        clk, SW0, SW1, KEY0, SW2, SW3, SW4;
  logic [3:0]  OPCODE;
  logic [7:0]  PC, alu_out, W_REG;

  lab5 u_cpu(clk, SW0, SW1, KEY0, SW2, SW3, SW4,
             OPCODE, PC, alu_out, W_REG);

  int fd;
  initial begin
    // open CSV and write header
    fd = $fopen("output.csv", "w");
    $fwrite(fd, "PC, IR, OPCODE, RA, RB, RD, W_Reg, Cout, OF\n");

    // reset & run
    clk = 0; SW0=1; SW1=0; SW2=SW3=SW4=0; KEY0=1;
    #20 SW0 = 0;
    repeat (500) #5 clk = ~clk;
    $fclose(fd);
    $finish;
  end

  // log at end of EX (state==2'd2)
  always @(posedge clk) begin
    if (u_cpu.u_ctrl.state == 2'd2) begin
      logic [15:0] IR = u_cpu.u_ctrl.IR;
      logic [3:0]  RA = IR[12:8], RB = IR[7:4], RD = IR[3:0];
      $fwrite(fd,
        "%02h, %04h, %01h, %01h, %01h, %01h, %02h, 0, 0\n",
        u_cpu.u_ctrl.PC,
        IR,
        IR[15:12],
        RA, RB, RD,
        u_cpu.u_ctrl.W_REG
      );
    end
  end
endmodule */


//------------------------------------------------------------------------------
// Lab5 wrapper for single-step
//------------------------------------------------------------------------------
module lab5(
  input        clk,
  input        SW0,
  input        SW1,
  input        KEY0,
  input        SW2, SW3, SW4,
  output logic [3:0] OPCODE,
  output logic [7:0] PC,
  output logic [7:0] alu_out,
  output logic [7:0] W_REG
);
  logic gated_clk;
  always_comb gated_clk = SW1 ? KEY0 : clk;

  control u_ctrl(gated_clk, SW0, OPCODE, PC, alu_out, W_REG);
endmodule


//------------------------------------------------------------------------------
// CONTROL FSM 
//------------------------------------------------------------------------------
module control(
  input  logic        clk,
  input  logic        reset,
  output logic [3:0]  OPCODE,
  output logic [7:0]  PC,
  output logic [7:0]  alu_out,
  output logic [7:0]  W_REG
);
  typedef enum logic [1:0] {IF=2'd0, FD=2'd1, EX=2'd2, RWB=2'd3} state_t;
  state_t state, next_state;

  logic [15:0] IR;
  logic [3:0]  RA, RB, RD;
  logic [7:0]  A, B;
  logic        write_en;

  assign write_en = (state == RWB) && (IR[15:12] < 4'hD);

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      state <= IF;
      PC    <= 8'd0;
      W_REG <= 8'd0;
    end else begin
      state <= next_state;
      if (write_en)
        W_REG <= alu_out;
      if (state == RWB) begin
        case (IR[15:12])
          4'hE: PC <= {RA,RB};
          4'hD: PC <= (A >= B) ? PC + RD : PC + 1;
          4'hF: PC <= PC;
          default: PC <= (PC == 8'h14) ? PC : PC + 1;
        endcase
      end
    end
  end

  always_comb begin
    next_state = IF;
    case (state)
      IF:  next_state = FD;
      FD:  next_state = EX;
      EX:  next_state = RWB;
      RWB: next_state = (IR[15:12] == 4'hF) ? RWB : IF;
    endcase
  end

  // fetch & decode
  ROM     u_rom(PC, IR);
  assign OPCODE = IR[15:12];
  assign RA     = IR[12:8];
  assign RB     = IR[7:4];
  assign RD     = IR[3:0];

  // datapath
  RegFile u_rf(clk, reset, RA, RB, RD, OPCODE, state, W_REG, A, B);
  ALU     u_alu(OPCODE, RA, RB, A, B, alu_out);
endmodule


//------------------------------------------------------------------------------
// ALU with 16:1 MUX
//------------------------------------------------------------------------------
module ALU(
  input  logic [3:0] OPCODE,
  input  logic [3:0] RA,
  input  logic [3:0] RB,
  input  logic [7:0] A,
  input  logic [7:0] B,
  output logic [7:0] alu_out
);
  logic [7:0] case_vec[0:15];
  assign case_vec[1]  = A + B;
  assign case_vec[2]  = {RA,RB};
  assign case_vec[3]  = A - B;
  assign case_vec[4]  = A + RB;
  assign case_vec[5]  = A * B;
  //assign case_vec[6]  = A / B;
  assign case_vec[6]  = (B != 0) ? (A / B) : 8'd0;
  assign case_vec[7]  = B - 1;
  assign case_vec[8]  = B + 1;
  assign case_vec[9]  = ~(A | B);
  assign case_vec[10] = ~(A & B);
  assign case_vec[11] = A ^ B;
  assign case_vec[12] = ~B;
  assign case_vec[13] = 8'd0;
  assign case_vec[14] = 8'd0;
  assign case_vec[15] = 8'd0;

  Mux16to1 u_mux(OPCODE, case_vec, alu_out);
endmodule


//------------------------------------------------------------------------------
// 4:1 MUX
//------------------------------------------------------------------------------
module Mux4to1(
  input  logic [1:0] sel,
  input  logic [7:0] in0, in1, in2, in3,
  output logic [7:0] out
);
  always_comb begin
    case (sel)
      2'b00: out = in0;
      2'b01: out = in1;
      2'b10: out = in2;
      2'b11: out = in3;
      default: out = '0;
    endcase
  end
endmodule

module Mux16to1(
  input  logic [3:0] sel,
  input  logic [7:0] data_in[0:15],
  output logic [7:0] out
);
  logic [7:0] m0, m1, m2, m3;
  Mux4to1 u0(sel[1:0], data_in[0],data_in[1],data_in[2],data_in[3],m0);
  Mux4to1 u1(sel[1:0], data_in[4],data_in[5],data_in[6],data_in[7],m1);
  Mux4to1 u2(sel[1:0], data_in[8],data_in[9],data_in[10],data_in[11],m2);
  Mux4to1 u3(sel[1:0], data_in[12],data_in[13],data_in[14],data_in[15],m3);
  Mux4to1 u4(sel[3:2],m0,m1,m2,m3,out);
endmodule


//------------------------------------------------------------------------------
// W-Register (DReg)
//------------------------------------------------------------------------------
module wreg(
  input  logic clk,
  input  logic reset,
  input  logic en,
  input  logic [7:0] data_in,
  output logic [7:0] data_out
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset)      data_out <= '0;
    else if (en)    data_out <= data_in;
  end
endmodule


//------------------------------------------------------------------------------
// Counter/divider
//------------------------------------------------------------------------------
module pmcntr#(parameter size=25)(
  input  logic clk,
  input  logic reset,
  input  logic [size-1:0] max_count,
  output logic [size-1:0] cnt,
  output logic clkout
);
  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin cnt<=0; clkout<=0; end
    else if (cnt<max_count) cnt<=cnt+1;
    else begin cnt<=0; clkout<=~clkout; end
  end
endmodule


//------------------------------------------------------------------------------
// ASCII 7-Segment 
//------------------------------------------------------------------------------
module ASCII7Seg(input logic [7:0] AsciiCode, output logic [6:0] segments);
  always_comb begin
    segments = 7'b1111111;  // default: all segments off (active-low)
    case (AsciiCode)
      "0", "O", "o": segments = 7'b1000000;
      "1":           segments = 7'b1111001;
      "2":           segments = 7'b0100100;
      "3":           segments = 7'b0110000;
      "4":           segments = 7'b0011001;
      "5":           segments = 7'b0010010;
      "6":           segments = 7'b0000010;
      "7":           segments = 7'b1111000;
      "8":           segments = 7'b0000000;
      "9":           segments = 7'b0010000;
      "A", "a":      segments = 7'b0001000;
      "b":           segments = 7'b0000011;
      "C", "c":      segments = 7'b1000110;
      "d":           segments = 7'b0100001;
      "E", "e":      segments = 7'b0000110;
      "F", "f":      segments = 7'b0001110;
      "G", "g":      segments = 7'b0001100;
      "U", "u":      segments = 7'b1110001;
      "J", "j":      segments = 7'b1111001;
      "N", "n":      segments = 7'b1001000;
      "_":           segments = 7'b1111110;
      default:       segments = 7'b1111111;
    endcase
  end
endmodule

//------------------------------------------------------------------------------
// Register File (unchanged)
//------------------------------------------------------------------------------
module RegFile(
  input  logic        clk,
  input  logic        reset,
  input  logic [3:0]  RA,
  input  logic [3:0]  RB,
  input  logic [3:0]  RD,
  input  logic [3:0]  OPCODE,
  input  logic [1:0]  current_state,
  input  logic [7:0]  RF_data_in,
  output logic [7:0]  RF_data_out0,
  output logic [7:0]  RF_data_out1
);
  logic [7:0] RF [0:15];
  parameter IF=2'd0, FD=2'd1, EX=2'd2, RWB=2'd3;

  always_ff @(posedge clk or posedge reset) begin
    if (reset) begin
      for (int i=0;i<16;i++) RF[i]<=0;
    end else if ((current_state==RWB) && (OPCODE<4'hD)) begin
      RF[RD] <= RF_data_in;
    end
  end
  assign RF_data_out0 = RF[RA];
  assign RF_data_out1 = RF[RB];
endmodule


//------------------------------------------------------------------------------
// ROM with mux21a
//------------------------------------------------------------------------------
module ROM(
  input  logic [7:0] PC,
  output logic [15:0] data
);
  logic [15:0] mem[0:20];
  assign mem[ 0]=16'h2000;
  assign mem[ 1]=16'h2011;
  assign mem[ 2]=16'h2002;
  assign mem[ 3]=16'h20A3;
  assign mem[ 4]=16'hD236;
  assign mem[ 5]=16'h1014;
  assign mem[ 6]=16'h4100;
  assign mem[ 7]=16'h4401;
  assign mem[ 8]=16'h8022;
  assign mem[ 9]=16'hE040;
  assign mem[10]=16'h4405;
  assign mem[11]=16'h6536;
  assign mem[12]=16'h5637;
  assign mem[13]=16'h3538;
  assign mem[14]=16'h4329;
  assign mem[15]=16'h709A;
  assign mem[16]=16'h70AB;
  assign mem[17]=16'hBB8C;
  assign mem[18]=16'h9D8E;
  assign mem[19]=16'hC0EF;
  assign mem[20]=16'hF000;
  mux21a u21(mem, PC, data);
endmodule

//------------------------------------------------------------------------------
// 21:1 mux for ROM
//------------------------------------------------------------------------------
module mux21a(
  input  logic [15:0] a[0:20],
  input  logic [7:0]  s,
  output logic [15:0] z
);
  assign z = a[s];
endmodule


