// clocktime
module clocktime #(parameter size=8) (input clk, freerun, reset, input [size-1:0] Maxval, output logic [size-1:0] Count, output logic clkout);
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			Count  <= {size{1'b0}};
			clkout <= 1'b0;
		end
		else if (freerun) begin
			if (Count < Maxval)
				Count <= Count + 1;
			else begin
				Count  <= {size{1'b0}};
				clkout <= ~clkout;
			end
		end
	end
endmodule

// fdivby2_
module fdivby2_ #(parameter size=25) (input clk, reset, output logic clkout);
	always_ff @(posedge clk or posedge reset) begin
		if (reset)      clkout <= 1'b0;
		else            clkout <= ~clkout;
	end
endmodule

// pmcntr
module pmcntr #(parameter siz=25) (input clk, reset, input [siz-1:0] count_max, output logic [siz-1:0] count, output logic clkout);
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			count  <= {siz{1'b0}};
			clkout <= 1'b0;
		end
		else if (count < count_max)
			count <= count + 1;
		else begin
			count  <= {siz{1'b0}};
			clkout <= ~clkout;
		end
	end
endmodule

// alarm_clock (sim core)
module alarm_clock(input CLK_2Hz, reset, time_set, alarm_set, sethrs1min0, run, activatealarm, alarmreset,
                   output logic [7:0] sec, min, hrs, sec_alrm, min_alrm, hrs_alrm, output logic alrm);
	localparam fiftynine   = 8'd59, twentythree = 8'd23;
	logic sec_div;
	fdivby2_ d1 (CLK_2Hz, reset, sec_div);
	logic sec_clk, min_clk, hr_clk, runsec, runmin, runhrs, sel_min, sel_hr;
	clocktime #(8) SecClock (sec_div,    runsec,   reset, fiftynine,   sec,      sec_clk);
	clocktime #(8) MinClock (sel_min,    runmin,   reset, fiftynine,   min,      min_clk);
	clocktime #(8) HrClock  (sel_hr,     runhrs,   reset, twentythree, hrs,      hr_clk);
	logic asec, amin, ahr, arunmin, arunhrs, sel_amin, sel_ahr;
	clocktime #(8) AlarmSec (sec_div,        1'b0,     alarmreset, fiftynine, sec_alrm,  asec);
	clocktime #(8) AlarmMin (sel_amin,       arunmin,  alarmreset, fiftynine, min_alrm,  amin);
	clocktime #(8) AlarmHr  (sel_ahr,        arunhrs,  alarmreset, twentythree, hrs_alrm, ahr);

	always_comb begin
		runsec  = 0; runmin  = 0; runhrs  = 0; arunmin = 0; arunhrs = 0;
		sel_min = sec_clk; sel_hr = min_clk; sel_amin = 1'b0; sel_ahr = 1'b0;
		if (run) begin
			runsec = 1; runmin = 1; runhrs = 1;
		end
		else if (time_set) begin
			if (sethrs1min0) begin sel_hr  = CLK_2Hz; runhrs  = 1; end
			else             begin sel_min = CLK_2Hz; runmin  = 1; end
		end
		else if (alarm_set) begin
			if (sethrs1min0) begin sel_ahr  = CLK_2Hz; arunhrs = 1; end
			else             begin sel_amin = CLK_2Hz; arunmin = 1; end
		end
	end

	assign alrm = activatealarm && (hrs == hrs_alrm) && (min == min_alrm) /*&& (sec == 8'd0)*/;
endmodule

// alarm_clock_tb
module alarm_clock_tb();
	logic clk, reset, time_set, alarm_set, sethrs1min0, run, activatealarm, alarmreset;
	logic [7:0] sec, min, hrs, sec_alrm, min_alrm, hrs_alrm;
	logic alrm;

	alarm_clock c1 (clk, reset, time_set, alarm_set, sethrs1min0, run, activatealarm, alarmreset,
	                sec, min, hrs, sec_alrm, min_alrm, hrs_alrm, alrm);

	initial clk = 0;
	always #5 clk = ~clk;

	initial begin
		// 1. Reset both clock and alarm
		reset = 1; alarmreset = 1;
		time_set = 0; alarm_set = 0;
		sethrs1min0 = 0; run = 0; activatealarm = 0;
		#20 reset = 0; alarmreset = 0;

		// 2. Set alarm to 7:22
		alarm_set = 1; sethrs1min0 = 1;
		repeat(7)  @(posedge clk);
		@(negedge clk); sethrs1min0 = 0;
		repeat(22) @(posedge clk);
		@(negedge clk); alarm_set    = 0;

		// 3. Set time to 7:21
		time_set = 1; sethrs1min0 = 1;
		repeat(7)  @(posedge clk);
		@(negedge clk); sethrs1min0 = 0;
		repeat(21) @(posedge clk);
		@(negedge clk); time_set     = 0;

		// 4. Activate alarm and run time
		activatealarm = 1; run = 1;
		wait(alrm);

		// 5. Display and reset alarm
		$display("Alarm at %0d:%0d:%0d", hrs, min, sec);
		alarmreset = 1; @(posedge clk); alarmreset = 0;

		#20;
	end
endmodule

//------------------------------------------------------------------------------
// ASCII7Seg: ASCII ‘0’…‘9’ → 7-segment
//------------------------------------------------------------------------------
module ASCII7Seg (
    input  logic [7:0] AsciiCode,
    output reg [6:0]   HexSeg
);
    always_comb begin
        case (AsciiCode)
            8'h30: HexSeg = 7'b1000000; // '0'
            8'h31: HexSeg = 7'b1111001; // '1'
            8'h32: HexSeg = 7'b0100100; // '2'
            8'h33: HexSeg = 7'b0110000; // '3'
            8'h34: HexSeg = 7'b0011001; // '4'
            8'h35: HexSeg = 7'b0010010; // '5'
            8'h36: HexSeg = 7'b0000010; // '6'
            8'h37: HexSeg = 7'b1111000; // '7'
            8'h38: HexSeg = 7'b0000000; // '8'
            8'h39: HexSeg = 7'b0010000; // '9'
            default: HexSeg = 7'b1000000; // blank or treat as '0'
        endcase
    end
endmodule


// alarm_clock_pv: Physical-validation 
module alarm_clock_pv (
    input  logic        CLK,          // 50 MHz board clock (PIN_P11)
    input  logic        SW5, SW4, SW3, SW2, SW1, SW0,  // slide switches
    input  logic        KEY1, KEY0,   // push-buttons (PIN_A7, PIN_B8)
    output logic [6:0]  SEC_LSB, SEC_MSB,
    output logic [6:0]  MIN_LSB, MIN_MSB,
    output logic [6:0]  HR_LSB,  HR_MSB,
    output logic        LED7, LED5, LED4, LED3, LED2, LED1, LED0
);

    // 1) divide 50 MHz → 2 Hz
    logic        clk2Hz;
    logic [24:0] cnt25;
    // uses pmcntr #(25) (clk, reset, count_max, count, clkout)
    pmcntr #(25) p1 (CLK, SW0, 25'd12_500_000, cnt25, clk2Hz);

    // 2) switch/button functions exactly per spec
    wire reset_core    = SW0;                       // SW0 resets clock & alarm
    wire alarmreset    = SW0 | ~KEY0;               // SW0 or KEY0(clear) resets blinking
    wire alarm_set     = SW1 & ~SW0;                // SW1 enters alarm-set mode
    wire time_set      = SW2 & ~SW0;                // SW2 enters time-set mode
    wire sethrs1min0   = SW3;                       // SW3 chooses hours(1)/minutes(0)
    wire activatealarm = SW4 & ~SW0;                // SW4 arms the alarm when running
    wire run           = SW5 & ~alarm_set & ~time_set & ~SW0;
    wire step_enable   = ~KEY1 & (alarm_set | time_set); // KEY1=0 steps selection

    // 3) instantiate sim core (positional)
    logic [7:0] sec, min, hrs, sa, ma, ha;
    logic       alrm;
    alarm_clock c1 (
        clk2Hz,        // CLK_2Hz
        reset_core,    // reset
        time_set,      // time_set
        alarm_set,     // alarm_set
        sethrs1min0,   // sethrs1min0
        run,           // run
        activatealarm, // activatealarm
        alarmreset,    // alarmreset
        sec, min, hrs, // outputs
        sa, ma, ha,    // alarm outputs
        alrm           // alarm flag
    );

    // 4) decimal-digit split
    logic [7:0] d0,d1,d2,d3,d4,d5;
    always_comb begin
        if (alarm_set) begin
            // force seconds to 00 while setting
            d0 = 0;          d1 = 0;
            d2 = ma % 10;    d3 = ma / 10;  // minutes
            d4 = ha % 10;    d5 = ha / 10;  // hours
        end else begin
            d0 = sec % 10;   d1 = sec / 10;
            d2 = min % 10;   d3 = min / 10;
            d4 = hrs % 10;   d5 = hrs / 10;
        end
        // mirror SW0–SW5 onto LED0–LED5
        {LED5,LED4,LED3,LED2,LED1,LED0} = {SW5,SW4,SW3,SW2,SW1,SW0};
    end

    // 5) convert to ASCII codes by adding '0' (0x30)
    wire [7:0] a0 = d0 + 8'h30;
    wire [7:0] a1 = d1 + 8'h30;
    wire [7:0] a2 = d2 + 8'h30;
    wire [7:0] a3 = d3 + 8'h30;
    wire [7:0] a4 = d4 + 8'h30;
    wire [7:0] a5 = d5 + 8'h30;

    // 6) drive the six 7-segment displays
    ASCII7Seg s0 (a0, SEC_LSB);
    ASCII7Seg s1 (a1, SEC_MSB);
    ASCII7Seg s2 (a2, MIN_LSB);
    ASCII7Seg s3 (a3, MIN_MSB);
    ASCII7Seg s4 (a4, HR_LSB);
    ASCII7Seg s5 (a5, HR_MSB);

    // 7) blink LED7 at 2 Hz when alarm trips
    always_ff @(posedge clk2Hz or posedge alarmreset) begin
        if      (alarmreset) LED7 <= 1'b0;
        else if (alrm)       LED7 <= ~LED7;
        else                 LED7 <= 1'b0;
    end

endmodule





