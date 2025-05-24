// FSM module 
module FSM (input  logic clk, reset, SW1, SW2, SW3, SW4,
            output logic [2:0] state,
            output logic [1:0] Z);

   // a. local-state encodings
   localparam Strt  = 3'b000,
              Seen1 = 3'b001,
              Seen2 = 3'b010,
              Seen3 = 3'b011,
              Seen4 = 3'b100;

   logic [2:0] nextState;

   // b. synchronous state register
   always_ff @(posedge clk or posedge reset)
      if (reset)
         state <= Strt;
      else
         state <= nextState;

   // c. combinational next-state logic & Moore outputs
   always_comb begin
      nextState = state;   // stay unless a legal single-switch press occurs
      Z         = 2'b01;   // default output for Strt

      unique case (state)
         Strt:  begin
                    Z = 2'b01;
                    if (SW2 & ~SW1 & ~SW3 & ~SW4) nextState = Seen1;
                    else if (SW3 & ~SW1 & ~SW2 & ~SW4) nextState = Seen3;
                end

         Seen1: begin
                    Z = 2'b10;
                    if (SW1 & ~SW2 & ~SW3 & ~SW4) nextState = Seen2;
                end

         Seen2: begin
                    Z = 2'b00;
                    if (SW1 & ~SW2 & ~SW3 & ~SW4) nextState = Seen1;
                    else if (SW4 & ~SW1 & ~SW2 & ~SW3) nextState = Seen3;
                end

         Seen3: begin
                    Z = 2'b00;
                    if (SW1 & ~SW2 & ~SW3 & ~SW4) nextState = Seen1;
                    else if (SW3 & ~SW1 & ~SW2 & ~SW4) nextState = Seen4;
                end

         Seen4: begin
                    Z = 2'b11;
                    if (SW1 & ~SW2 & ~SW3 & ~SW4) nextState = Seen1;
                end
      endcase
   end
endmodule

//FSM module test bench 
module FSM_tb();
	logic KEY0, SW0, SW1, SW2, SW3, SW4;
	logic [2:0] state;
	logic [1:0] Z;
	FSM fsm1 (KEY0, SW0, SW1, SW2, SW3, SW4, state, Z);

	initial
		begin
			// ----------  Sequence #1 ----------
			KEY0=0; SW0=1; SW1=0; SW2=0; SW3=0; SW4=0; #10;   // SW0=1  (reset)
			SW0=0; SW3=1;                                   #10;   // SW3=1
			SW3=0; SW1=1;                                   #40;   // SW1=1 x4 (held high 4 clocks)
			SW1=0; SW4=1;                                   #10;   // SW4=1
			SW4=0; SW2=1; SW3=1;                            #10;   // SW2=1 & SW3=1
			SW2=0; SW3=0; SW3=1;                            #10;   // SW3=1
			SW3=0; SW1=1;                                   #10;   // SW1=1
			SW1=0; SW0=1;                                   #10;   // SW0=1 (second reset)
			SW0=0; SW3=1;                                   #20;   // SW3=1, SW3=1 (two clocks)
			SW3=0;                                          #10;

			// ----------  Sequence #2 ----------
			SW0=1;                                          #10;   // SW0=1 (reset)
			SW0=0; SW2=1;                                   #10;   // SW2=1
			SW2=0; SW1=1;                                   #10;   // SW1=1
			SW1=0; SW4=1;                                   #10;   // SW4=1
			SW4=0; SW3=1;                                   #10;   // SW3=1
			SW3=0; SW1=1;                                   #10;   // SW1=1
			SW1=0;                                          #10;
		end

	always begin
		#5 KEY0 = ~KEY0;
	end
endmodule


/*//FSM physical validation
module FSM_pv (input  KEY0, SW0, SW1, SW2, SW3, SW4,
               output logic [6:0] SEG0, SEG1, SEG2, SEG3,
               output logic [6:0] LED_SW);

   // a. internal wires
   wire  [2:0] state;
   logic [1:0] Z;

   // b. DUT and 7-segment decoder
   FSM        fsm1  (KEY0, SW0, SW1, SW2, SW3, SW4, state, Z);
   ASCIICodes disp   (state, SEG3, SEG2, SEG1, SEG0);

   // c. mirror switches & Z on the LED header
   always begin
      LED_SW[6:5] = Z;   // current FSM output
      LED_SW[4]   = SW4;
      LED_SW[3]   = SW3;
      LED_SW[2]   = SW2;
      LED_SW[1]   = SW1;
      LED_SW[0]   = SW0;
   end
endmodule*/


// 
module FSM_pv (input  KEY0,              // push-button clock
               input  SW0, SW1, SW2, SW3, SW4,   // slide switches
               output logic [6:0] SEG0, SEG1, SEG2, SEG3,
               output logic [6:0] LEDR);         // on-board red LEDs

   // a. internal nets
   wire  [2:0] state;
   logic [1:0] Z;

   // b. DUT and 7-segment decoder
   FSM         dut  (KEY0, SW0, SW1, SW2, SW3, SW4, state, Z);
   ASCIICodes  disp (state, SEG3, SEG2, SEG1, SEG0);   // show S0-S4

   // c. LED assignments  (REQUIRED in lab write-up)
   always_comb begin
      // switches straight to LEDs
      LEDR[0]   = SW0;
      LEDR[1]   = SW1;
      LEDR[2]   = SW2;
      LEDR[3]   = SW3;
      LEDR[4]   = SW4;

      // FSM output Z on LEDR6:5
      {LEDR[6], LEDR[5]} = Z;   // MSB on LEDR6, LSB on LEDR5
   end

   /*// c. LED assignments  ? ONLY the active switch + Z bits light up
   always_comb begin
      LEDR      = '0;                          // everything OFF by default
      LEDR[4:0] = {SW4, SW3, SW2, SW1, SW0};   // mirror the five switches
      LEDR[6:5] = Z;                           // FSM output bits
   end*/
endmodule


// 7-segment decoder for lab-3 FSM    (g f e d c b a, active-LOW)
module ASCIICodes (input  logic [2:0] state,
                   output logic [6:0] SEG3, SEG2, SEG1, SEG0);

   // a. character segment patterns (common-anode: 0 = ON)
   localparam SEG_S  = 7'b0010010,   // upper-case ?S?
              SEG_U  = 7'b1000001,   // upper-case ?U?
              SEG_O  = 7'b1000000,   // upper-case ?O?
              SEG_G  = 7'b0010000,   // upper-case ?G?
              SEG_N  = 7'b1001000,   // upper-case ?N?
              SEG_UND= 7'b1110111,   // underscore  (?_?)
              SEG_0  = 7'b1000000,
              SEG_1  = 7'b1111001,
              SEG_2  = 7'b0100100,
              SEG_3  = 7'b0110000,
              SEG_4  = 7'b0011001;

   // four characters of last anem 
   localparam SEG_L0 = SEG_O,
              SEG_L1 = SEG_G,
              SEG_L2 = SEG_U,
              SEG_L3 = SEG_N;

   // b. combinational character selection
   always_comb
      unique case (state)
         3'b000 : begin                     // S0 last-name banner
                     SEG3 = SEG_L0;         // left-most of the four
                     SEG2 = SEG_L1;
                     SEG1 = SEG_L2;
                     SEG0 = SEG_L3;         // right-most
                  end
         3'b001 : begin SEG3=SEG_S; SEG2=SEG_UND; SEG1=SEG_0; SEG0=SEG_1; end // S_01
         3'b010 : begin SEG3=SEG_S; SEG2=SEG_UND; SEG1=SEG_0; SEG0=SEG_2; end // S_02
         3'b011 : begin SEG3=SEG_S; SEG2=SEG_UND; SEG1=SEG_0; SEG0=SEG_3; end // S_03
         3'b100 : begin SEG3=SEG_S; SEG2=SEG_UND; SEG1=SEG_0; SEG0=SEG_4; end // S_04
         default: begin SEG3='1; SEG2='1; SEG1='1; SEG0='1; end              // all off
      endcase
endmodule


