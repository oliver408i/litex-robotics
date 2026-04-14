module status_ws2812_strip #(
  parameter integer CLK_HZ = 27_000_000,
  parameter integer FLASH_MS = 80,
  parameter integer IDLE_MS  = 1000,
  parameter real BLINK_HZ = 0.5,
  parameter integer LED_COUNT = 150,
  parameter integer STATUS_LED = 1
)(
  input  wire        clk_g,
  input  wire        activity_pulse,
  input  wire        override_en,
  input  wire [23:0] override_color_grb,
  input  wire [7:0]  override_brightness,
  input  wire        strip_write,
  input  wire [15:0] strip_index,
  input  wire [23:0] strip_color_grb,
  output reg         ws2812_din = 1'b0
);
  localparam integer STRIP_COUNT = (STATUS_LED != 0) ?
                                   ((LED_COUNT > 1) ? (LED_COUNT - 1) : 0) :
                                   LED_COUNT;

  localparam integer FLASH_CYC = (CLK_HZ/1000)*FLASH_MS;
  localparam integer IDLE_CYC  = (CLK_HZ/1000)*IDLE_MS;
  localparam integer BLINK_CYC = CLK_HZ/(2.0*BLINK_HZ);

  reg [31:0] flash_cnt = 0;
  reg [31:0] idle_cnt = 0;
  reg [31:0] blink_cnt = 0;
  reg blink = 0;

  always @(posedge clk_g) begin
    if (activity_pulse) flash_cnt <= FLASH_CYC;
    else if (flash_cnt != 0) flash_cnt <= flash_cnt - 1;

    if (activity_pulse) idle_cnt <= 0;
    else if (idle_cnt < IDLE_CYC) idle_cnt <= idle_cnt + 1;

    if (blink_cnt >= BLINK_CYC) begin
      blink_cnt <= 0;
      blink <= ~blink;
    end else begin
      blink_cnt <= blink_cnt + 1;
    end
  end

  function [23:0] scale_grb(input [23:0] color, input [7:0] brightness);
    reg [15:0] g;
    reg [15:0] r;
    reg [15:0] b;
    begin
      g = color[23:16] * brightness;
      r = color[15:8]  * brightness;
      b = color[7:0]   * brightness;
      scale_grb = {g[15:8], r[15:8], b[15:8]};
    end
  endfunction

  wire [23:0] policy_color =
      (flash_cnt != 0)              ? {8'd10, 8'd0, 8'd0} :
      ((idle_cnt >= IDLE_CYC) && blink) ? {8'd6, 8'd8, 8'd0} :
                                          24'h000000;

  wire [23:0] led0_color = scale_grb(
      override_en ? override_color_grb : policy_color,
      override_en ? override_brightness : 8'hFF
  );

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

  localparam integer CYC_PER_US = CLK_HZ / 1_000_000;
  localparam integer T0H = ((CYC_PER_US * 350) + 999) / 1000;
  localparam integer T1H = ((CYC_PER_US * 700) + 999) / 1000;
  localparam integer TBT = ((CYC_PER_US * 1250) + 999) / 1000;
  localparam integer TRES = CYC_PER_US * 300;
  localparam integer T0H_C = (T0H < 2) ? 2 : T0H;
  localparam integer T1H_C = (T1H < 3) ? 3 : T1H;
  localparam integer TBT_C = (TBT < 6) ? 6 : TBT;
  localparam integer LED_BITS = (LED_COUNT <= 2) ? 1 : $clog2(LED_COUNT);

  reg [23:0] shift_reg = 24'h0;
  reg [15:0] bit_cycle = 16'd0;
  reg [19:0] reset_count = 20'd0;
  reg [5:0] bit_index = 6'd0;
  reg [LED_BITS-1:0] led_index = {LED_BITS{1'b0}};
  reg sending = 1'b0;

  wire cur_bit = shift_reg[23];
  wire [15:0] hi_lim = cur_bit ? T1H_C : T0H_C;
  wire [23:0] first_color = (STATUS_LED != 0) ? led0_color : strip_mem[0];

  always @(posedge clk_g) begin
    if (!sending) begin
      ws2812_din <= 1'b0;
      if (reset_count != 0) begin
        reset_count <= reset_count - 1'b1;
      end else begin
        led_index <= {LED_BITS{1'b0}};
        bit_index <= 6'd0;
        bit_cycle <= 16'd0;
        shift_reg <= first_color;
        sending <= 1'b1;
      end
    end else begin
      ws2812_din <= (bit_cycle < hi_lim);
      bit_cycle <= bit_cycle + 1'b1;

      if (bit_cycle == TBT_C - 1) begin
        bit_cycle <= 16'd0;
        if (bit_index == 6'd23) begin
          bit_index <= 6'd0;
          if (led_index == (LED_COUNT - 1)) begin
            sending <= 1'b0;
            ws2812_din <= 1'b0;
            reset_count <= TRES;
          end else begin
            led_index <= led_index + 1'b1;
            if (STATUS_LED != 0) begin
              if (led_index == {LED_BITS{1'b0}}) shift_reg <= strip_mem[0];
              else shift_reg <= strip_mem[led_index];
            end else begin
              shift_reg <= strip_mem[led_index + 1'b1];
            end
          end
        end else begin
          shift_reg <= {shift_reg[22:0], 1'b0};
          bit_index <= bit_index + 1'b1;
        end
      end
    end
  end
endmodule
