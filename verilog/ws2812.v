// Status pixel policy:
// - On activity_pulse: flash green for FLASH_MS
// - If no activity for IDLE_MS: slowly blink orange
// - Else: off
// If override_en==1, show override_color (after brightness scaling).
module status_ws2812 #(
  parameter integer CLK_HZ   = 27_000_000,
  parameter integer FLASH_MS = 80,
  parameter integer IDLE_MS  = 1000,
  parameter real BLINK_HZ = 0.5
)(
  input  wire        clk_g,
  input  wire        activity_pulse,

  // --- optional override from SPI/host ---
  input  wire        override_en,         // 1 = bypass policy
  input  wire [23:0] override_color_grb,  // {G,R,B}, 8b each
  input  wire [7:0]  override_brightness, // 0..255, 255 = full

  output wire        ws2812_din
);
  // === timers (as before) ===
  localparam integer FLASH_CYC = (CLK_HZ/1000)*FLASH_MS;
  localparam integer IDLE_CYC  = (CLK_HZ/1000)*IDLE_MS;
  localparam integer BLINK_CYC = CLK_HZ/(2.0*BLINK_HZ);

  reg [31:0] flash_cnt = 0, idle_cnt = 0;
  always @(posedge clk_g) begin
    if (activity_pulse) flash_cnt <= FLASH_CYC;
    else if (flash_cnt != 0) flash_cnt <= flash_cnt - 1;
    if (activity_pulse) idle_cnt <= 0;
    else if (idle_cnt < IDLE_CYC) idle_cnt <= idle_cnt + 1;
  end

  wire flash_on = (flash_cnt != 0);
  wire idle_on  = (idle_cnt  >= IDLE_CYC);

  // blink osc
  reg [31:0] blink_cnt = 0; reg blink = 0;
  always @(posedge clk_g) begin
    if (blink_cnt >= BLINK_CYC) begin blink_cnt <= 0; blink <= ~blink; end
    else blink_cnt <= blink_cnt + 1;
  end

  // palette (GRB)
  wire [23:0] COL_GREEN  = {8'd10, 8'd0,  8'd0};
  wire [23:0] COL_ORANGE = {8'd6,  8'd8,  8'd0};
  wire [23:0] COL_OFF    = 24'h000000;

  // default policy color
  wire [23:0] policy_color =
      flash_on            ? COL_GREEN  :
      (idle_on & blink)   ? COL_ORANGE :
                            COL_OFF;

  // --- brightness scaler (8-bit) ---
  // y = (x * bri) >> 8, applied to selected color channel-wise.
  function [23:0] scale_grb(input [23:0] c, input [7:0] bri);
    reg [15:0] g, r, b;
    begin
      g = (c[23:16] * bri);
      r = (c[15:8]  * bri);
      b = (c[7:0]   * bri);
      scale_grb = { g[15:8], r[15:8], b[15:8] };
    end
  endfunction

  wire [23:0] chosen = override_en ? override_color_grb : policy_color;
  wire [23:0] color  = scale_grb(chosen, override_en ? override_brightness : 8'd255);

  // refresh whenever driver is idle
  reg start = 1'b0; wire busy;
  always @(posedge clk_g) start <= ~busy;

  ws2812_oneled #(.CLK_HZ(CLK_HZ)) u_drv (
    .clk_g     (clk_g),
    .color_grb (color),
    .start     (start),
    .busy      (busy),
    .dout      (ws2812_din)
  );
endmodule


// WS2812 (NeoPixel) single-LED driver @800 kHz, GRB order.
// Uses precise cycle math from CLK_HZ. Longer reset (300 us) for robustness.
module ws2812_oneled #(
  parameter integer CLK_HZ = 27_000_000
)(
  input  wire        clk_g,
  input  wire [23:0] color_grb,  // {G,R,B}
  input  wire        start,      // 1-clk pulse to start transmit
  output reg         busy = 1'b0,       // 1 while sending/resetting
  output reg         dout = 1'b0
);
  // cycles per microsecond from clk_g
  localparam integer CYC_PER_US = CLK_HZ / 1_000_000;

  // Timings (us): T0H=0.35, T1H=0.70, TBIT=1.25, TRESET=300
  localparam integer T0H = ((CYC_PER_US*350)  + 999)/1000;  // ~0.35 us
  localparam integer T1H = ((CYC_PER_US*700)  + 999)/1000;  // ~0.70 us
  localparam integer TBT = ((CYC_PER_US*1250) + 999)/1000;  // ~1.25 us
  localparam integer TRES=  CYC_PER_US*300;                 // 300 us

  // safety floors
  localparam integer T0H_C = (T0H  < 2) ? 2 : T0H;
  localparam integer T1H_C = (T1H  < 3) ? 3 : T1H;
  localparam integer TBT_C = (TBT  < 6) ? 6 : TBT;

  reg [23:0] sh = 24'h0;
  reg [5:0]  bit_idx = 6'd0;
  reg [15:0] cyc = 16'd0;
  reg [19:0] res_cnt = 20'd0;
  reg        sending = 1'b0;

  wire cur_bit = sh[23];
  wire [15:0] hi_lim = cur_bit ? T1H_C : T0H_C;

  always @(posedge clk_g) begin
    if (!sending) begin
      dout <= 1'b0;
      if (!busy && start) begin
        sh      <= color_grb;
        bit_idx <= 6'd0;
        cyc     <= 16'd0;
        sending <= 1'b1;
        busy    <= 1'b1;
      end else if (busy) begin
        // reset latch low
        dout <= 1'b0;
        if (res_cnt != 0) res_cnt <= res_cnt - 1'b1;
        else              busy    <= 1'b0;
      end
    end else begin
      // bit slot
      dout <= (cyc < hi_lim);
      cyc  <= cyc + 1'b1;

      if (cyc == TBT_C-1) begin
        cyc <= 16'd0;
        if (bit_idx == 6'd23) begin
          sending <= 1'b0;
          dout    <= 1'b0;
          res_cnt <= TRES;
        end else begin
          sh      <= {sh[22:0], 1'b0};
          bit_idx <= bit_idx + 1'b1;
        end
      end
    end
  end
endmodule


// Status + strip WS2812 driver. LED0 is status/override, LED1..N-1 are user controlled.
module status_ws2812_strip #(
  parameter integer CLK_HZ = 27_000_000,
  parameter integer FLASH_MS = 80,
  parameter integer IDLE_MS  = 1000,
  parameter real BLINK_HZ = 0.5,
  parameter integer LED_COUNT = 150
)(
  input  wire        clk_g,
  input  wire        activity_pulse,

  // LED0 override (same as status_ws2812)
  input  wire        override_en,
  input  wire [23:0] override_color_grb,
  input  wire [7:0]  override_brightness,

  // Strip write port (LED1..N-1)
  input  wire        strip_write,
  input  wire [15:0] strip_index,      // 0 = LED1, 1 = LED2, ...
  input  wire [23:0] strip_color_grb,

  output reg         ws2812_din = 1'b0
);
  localparam integer STRIP_COUNT = (LED_COUNT > 1) ? (LED_COUNT - 1) : 0;

  // === timers ===
  localparam integer FLASH_CYC = (CLK_HZ/1000)*FLASH_MS;
  localparam integer IDLE_CYC  = (CLK_HZ/1000)*IDLE_MS;
  localparam integer BLINK_CYC = CLK_HZ/(2.0*BLINK_HZ);

  reg [31:0] flash_cnt = 0, idle_cnt = 0;
  always @(posedge clk_g) begin
    if (activity_pulse) flash_cnt <= FLASH_CYC;
    else if (flash_cnt != 0) flash_cnt <= flash_cnt - 1;
    if (activity_pulse) idle_cnt <= 0;
    else if (idle_cnt < IDLE_CYC) idle_cnt <= idle_cnt + 1;
  end

  wire flash_on = (flash_cnt != 0);
  wire idle_on  = (idle_cnt  >= IDLE_CYC);

  // blink osc
  reg [31:0] blink_cnt = 0; reg blink = 0;
  always @(posedge clk_g) begin
    if (blink_cnt >= BLINK_CYC) begin blink_cnt <= 0; blink <= ~blink; end
    else blink_cnt <= blink_cnt + 1;
  end

  // palette (GRB)
  wire [23:0] COL_GREEN  = {8'd10, 8'd0,  8'd0};
  wire [23:0] COL_ORANGE = {8'd6,  8'd8,  8'd0};
  wire [23:0] COL_OFF    = 24'h000000;

  wire [23:0] policy_color =
      flash_on            ? COL_GREEN  :
      (idle_on & blink)   ? COL_ORANGE :
                            COL_OFF;

  function [23:0] scale_grb(input [23:0] c, input [7:0] bri);
    reg [15:0] g, r, b;
    begin
      g = (c[23:16] * bri);
      r = (c[15:8]  * bri);
      b = (c[7:0]   * bri);
      scale_grb = { g[15:8], r[15:8], b[15:8] };
    end
  endfunction

  wire [23:0] chosen0 = override_en ? override_color_grb : policy_color;
  wire [23:0] led0_color = scale_grb(chosen0, override_en ? override_brightness : 8'd255);

  // Strip memory (LED1..N-1)
  reg [23:0] strip_mem [0:STRIP_COUNT-1];
  integer i;
  initial begin
    for (i = 0; i < STRIP_COUNT; i = i + 1) begin
      strip_mem[i] = 24'h000000;
    end
  end

  always @(posedge clk_g) begin
    if (strip_write && (strip_index < STRIP_COUNT)) begin
      strip_mem[strip_index] <= strip_color_grb;
    end
  end

  // === WS2812 timing (shared) ===
  localparam integer CYC_PER_US = CLK_HZ / 1_000_000;
  localparam integer T0H = ((CYC_PER_US*350)  + 999)/1000;
  localparam integer T1H = ((CYC_PER_US*700)  + 999)/1000;
  localparam integer TBT = ((CYC_PER_US*1250) + 999)/1000;
  localparam integer TRES=  CYC_PER_US*300;
  localparam integer T0H_C = (T0H  < 2) ? 2 : T0H;
  localparam integer T1H_C = (T1H  < 3) ? 3 : T1H;
  localparam integer TBT_C = (TBT  < 6) ? 6 : TBT;

  reg [23:0] sh = 24'h0;
  reg [15:0] cyc = 16'd0;
  reg [19:0] res_cnt = 20'd0;
  reg        sending = 1'b0;
  reg [5:0]  bit_idx = 6'd0;
  localparam integer LED_BITS = (LED_COUNT <= 2) ? 1 : $clog2(LED_COUNT);
  reg [LED_BITS-1:0] led_idx = {LED_BITS{1'b0}};

  wire cur_bit = sh[23];
  wire [15:0] hi_lim = cur_bit ? T1H_C : T0H_C;

  always @(posedge clk_g) begin
    if (!sending) begin
      ws2812_din <= 1'b0;
      if (res_cnt != 0) begin
        res_cnt <= res_cnt - 1'b1;
      end else begin
        // start a new frame
        led_idx <= 8'd0;
        bit_idx <= 6'd0;
        cyc <= 16'd0;
        sh <= led0_color;
        sending <= 1'b1;
      end
    end else begin
      ws2812_din <= (cyc < hi_lim);
      cyc <= cyc + 1'b1;

      if (cyc == TBT_C-1) begin
        cyc <= 16'd0;
        if (bit_idx == 6'd23) begin
          bit_idx <= 6'd0;
          if (led_idx == (LED_COUNT - 1)) begin
            sending <= 1'b0;
            ws2812_din <= 1'b0;
            res_cnt <= TRES;
          end else begin
            led_idx <= led_idx + 1'b1;
            if (led_idx == {LED_BITS{1'b0}}) begin
              sh <= strip_mem[0];
            end else begin
              sh <= strip_mem[led_idx];
            end
          end
        end else begin
          sh <= {sh[22:0], 1'b0};
          bit_idx <= bit_idx + 1'b1;
        end
      end
    end
  end
endmodule
