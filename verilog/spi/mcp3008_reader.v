// MCP3008 SPI reader
// Generates SPI transactions to sample up to eight channels and exposes
// the latest 10-bit results along with status tracking.
module mcp3008_reader #(
  parameter integer SCLK_DIV = 16
)(
  input  wire        clk,
  input  wire        rstn,
  input  wire        enable,
  input  wire [7:0]  channel_mask,
  input  wire [31:0] sample_interval_ticks,
  input  wire [7:0]  clear_update_mask,
  output reg         busy = 1'b0,
  output reg [7:0]   update_mask = 8'd0,
  output reg [2:0]   last_channel = 3'd0,
  output reg [31:0]  sample_count = 32'd0,
  output reg         adc_sclk = 1'b0,
  output reg         adc_cs_n = 1'b1,
  output reg         adc_mosi = 1'b0,
  input  wire        adc_miso,
  output reg [9:0]   sample_ch0 = 10'd0,
  output reg [9:0]   sample_ch1 = 10'd0,
  output reg [9:0]   sample_ch2 = 10'd0,
  output reg [9:0]   sample_ch3 = 10'd0,
  output reg [9:0]   sample_ch4 = 10'd0,
  output reg [9:0]   sample_ch5 = 10'd0,
  output reg [9:0]   sample_ch6 = 10'd0,
  output reg [9:0]   sample_ch7 = 10'd0
);

  localparam integer LP_CLKS_PER_HALF_BIT = (SCLK_DIV < 2) ? 2 : SCLK_DIV;

  reg        tx_dv = 1'b0;
  reg [7:0]  tx_byte = 8'h00;
  wire       tx_ready;
  wire       rx_dv;
  wire [7:0] rx_byte;
  wire       spi_clk;
  wire       spi_mosi;

  SPI_Master #(
    .SPI_MODE(0),
    .CLKS_PER_HALF_BIT(LP_CLKS_PER_HALF_BIT)
  ) u_spi (
    .i_Rst_L   (rstn),
    .i_Clk     (clk),
    .i_TX_Byte (tx_byte),
    .i_TX_DV   (tx_dv),
    .o_TX_Ready(tx_ready),
    .o_RX_DV   (rx_dv),
    .o_RX_Byte (rx_byte),
    .o_SPI_Clk (spi_clk),
    .i_SPI_MISO(adc_miso),
    .o_SPI_MOSI(spi_mosi)
  );

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      adc_sclk <= 1'b0;
      adc_mosi <= 1'b0;
    end else begin
      adc_sclk <= spi_clk;
      adc_mosi <= spi_mosi;
    end
  end

  reg [31:0] tick_cnt = 32'd0;
  wire interval_elapsed = (sample_interval_ticks == 0) ? 1'b1 :
                          (tick_cnt >= sample_interval_ticks - 1);

  always @(posedge clk or negedge rstn) begin
    if (!rstn)
      tick_cnt <= 32'd0;
    else if (!enable)
      tick_cnt <= 32'd0;
    else if (busy)
      tick_cnt <= 32'd0;
    else if (!interval_elapsed)
      tick_cnt <= tick_cnt + 1'b1;
  end

  reg [2:0] next_ch;

  function [2:0] find_next_channel;
    input [2:0] cur;
    input [7:0] mask;
    integer i;
    reg [2:0] cand;
    begin
      find_next_channel = cur;
      for (i = 1; i <= 8; i = i + 1) begin
        cand = (cur + i) & 3'd7;
        if (mask[cand] && (find_next_channel == cur))
          find_next_channel = cand;
      end
    end
  endfunction

  localparam [2:0]
    ST_IDLE   = 3'd0,
    ST_ASSERT = 3'd1,
    ST_SEND1  = 3'd2,
    ST_SEND2  = 3'd3,
    ST_SEND3  = 3'd4,
    ST_DONE   = 3'd5;

  reg [2:0] state = ST_IDLE;
  reg [7:0] rx_b0 = 8'h00;
  reg [7:0] rx_b1 = 8'h00;

  always @(posedge clk or negedge rstn) begin
    if (!rstn)
      adc_cs_n <= 1'b1;
    else begin
      case (state)
        ST_ASSERT, ST_SEND1, ST_SEND2, ST_SEND3: adc_cs_n <= 1'b0;
        default: adc_cs_n <= 1'b1;
      endcase
    end
  end

  always @(posedge clk or negedge rstn) begin
    if (!rstn)
      busy <= 1'b0;
    else
      busy <= (state != ST_IDLE);
  end

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state <= ST_IDLE;
      tx_dv <= 1'b0;
      tx_byte <= 8'h00;
      last_channel <= 3'd0;
      rx_b0 <= 8'h00;
      rx_b1 <= 8'h00;
      update_mask <= 8'd0;
      sample_count <= 32'd0;
    end else begin
      tx_dv <= 1'b0;
      case (state)
        ST_IDLE: begin
          if (enable && (channel_mask != 8'd0) && interval_elapsed) begin
            next_ch <= find_next_channel(last_channel, channel_mask);
            state <= ST_ASSERT;
          end
          if (clear_update_mask != 0)
            update_mask <= update_mask & ~clear_update_mask;
        end
        ST_ASSERT: begin
          if (tx_ready) begin
            tx_byte <= 8'h01;
            tx_dv <= 1'b1;
            state <= ST_SEND1;
          end
        end
        ST_SEND1: begin
          if (tx_ready && !tx_dv) begin
            tx_byte <= {1'b1, next_ch[2], next_ch[1], next_ch[0], 4'b0000};
            tx_dv <= 1'b1;
            state <= ST_SEND2;
          end
        end
        ST_SEND2: begin
          if (rx_dv)
            rx_b0 <= rx_byte;
          if (tx_ready && !tx_dv) begin
            tx_byte <= 8'h00;
            tx_dv <= 1'b1;
            state <= ST_SEND3;
          end
        end
        ST_SEND3: begin
          if (rx_dv)
            rx_b1 <= rx_byte;
          if (tx_ready && !tx_dv)
            state <= ST_DONE;
        end
        ST_DONE: begin
          case (next_ch)
            3'd0: sample_ch0 <= {rx_b0[1:0], rx_b1};
            3'd1: sample_ch1 <= {rx_b0[1:0], rx_b1};
            3'd2: sample_ch2 <= {rx_b0[1:0], rx_b1};
            3'd3: sample_ch3 <= {rx_b0[1:0], rx_b1};
            3'd4: sample_ch4 <= {rx_b0[1:0], rx_b1};
            3'd5: sample_ch5 <= {rx_b0[1:0], rx_b1};
            3'd6: sample_ch6 <= {rx_b0[1:0], rx_b1};
            3'd7: sample_ch7 <= {rx_b0[1:0], rx_b1};
          endcase
          update_mask[next_ch] <= 1'b1;
          last_channel <= next_ch;
          sample_count <= sample_count + 1'b1;
          state <= ST_IDLE;
        end
        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule
