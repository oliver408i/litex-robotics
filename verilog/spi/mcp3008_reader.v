// MCP3008 SPI reader
// Generates SPI transactions to sample up to eight channels and exposes
// the latest 10-bit results along with status tracking.
module mcp3008_reader #(
  parameter integer SCLK_DIV = 16  // system clocks per half SCLK period (>= 1)
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
  , output reg [15:0] debug_miso_trace = 16'd0
  , output reg  [4:0] debug_last_count = 5'd0
);
  
  // --------------------------------------------------------------------------
  // MCP3008 SPI Reader Implementation
  // Uses provided SPI_Master (Mode 0) to read enabled channels at a programmable
  // interval. Returns 10-bit results and sets update_mask bits when a channel
  // gets a new sample. clear_update_mask clears those bits. Busy is high during
  // an active SPI transaction (3 bytes per conversion as per datasheet Fig 6-1).
  // --------------------------------------------------------------------------

  // ----------------------
  // SPI master connection
  // ----------------------
  localparam integer LP_CLKS_PER_HALF_BIT = (SCLK_DIV < 2) ? 2 : SCLK_DIV;

  // SPI_Master signals
  reg        tx_dv = 1'b0;
  reg [7:0]  tx_byte = 8'h00;
  wire       tx_ready;
  wire       rx_dv;
  wire [7:0] rx_byte;
  wire       spi_clk;
  wire       spi_mosi;

  SPI_Master #(
    .SPI_MODE(0),                // MCP3008 supports modes 0,0 and 1,1
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

  // Register pass-through to align with reg outputs
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      adc_sclk <= 1'b0;
      adc_mosi <= 1'b0;
    end else begin
      adc_sclk <= spi_clk;
      adc_mosi <= spi_mosi;
    end
  end

  // ----------------------
  // Sampling scheduler
  // ----------------------
  reg [31:0] tick_cnt = 32'd0;
  wire       interval_elapsed = (sample_interval_ticks == 0) ? 1'b1
                                : (tick_cnt >= sample_interval_ticks - 1);

  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      tick_cnt <= 32'd0;
    end else if (!enable) begin
      tick_cnt <= 32'd0;
    end else if (busy) begin
      tick_cnt <= 32'd0; // reset timer while a conversion is in flight
    end else if (!interval_elapsed) begin
      tick_cnt <= tick_cnt + 1'b1;
    end
  end

  // ----------------------
  // Channel iteration helper
  // ----------------------
  reg [2:0] next_ch;

  function [2:0] find_next_channel;
    input [2:0] cur;
    input [7:0] mask;
    integer i;
    reg [2:0] cand;
    begin
      find_next_channel = cur; // default
      // search forward (cur+1 .. cur+8)
      for (i = 1; i <= 8; i = i + 1) begin
        cand = (cur + i) & 3'd7;
        if (mask[cand] && (find_next_channel == cur)) begin
          // capture the first eligible channel only
          find_next_channel = cand;
        end
      end
    end
  endfunction

  // ----------------------
  // FSM to transact a single conversion (3 bytes)
  // ----------------------
  localparam [2:0]
    ST_IDLE   = 3'd0,
    ST_ASSERT = 3'd1,
    ST_SEND1  = 3'd2,
    ST_SEND2  = 3'd3,
    ST_SEND3  = 3'd4,
    ST_DONE   = 3'd5;

  reg [2:0] state = ST_IDLE;
  reg [1:0] bytes_sent = 2'd0;
  reg [7:0] rx_b0 = 8'h00; // first received byte after command (contains null + B9..B8 in [1:0])
  reg [7:0] rx_b1 = 8'h00; // second received byte (B7..B0)

  // Chip select handling (active-low)
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      adc_cs_n <= 1'b1;
    end else begin
      case (state)
        ST_ASSERT, ST_SEND1, ST_SEND2, ST_SEND3: adc_cs_n <= 1'b0;
        default:                                  adc_cs_n <= 1'b1;
      endcase
    end
  end

  // Busy flag
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      busy <= 1'b0;
    end else begin
      busy <= (state != ST_IDLE);
    end
  end

  // Issue TX bytes and collect RX bytes
  always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state       <= ST_IDLE;
      tx_dv       <= 1'b0;
      tx_byte     <= 8'h00;
      bytes_sent  <= 2'd0;
      last_channel<= 3'd0;
      rx_b0       <= 8'h00;
      rx_b1       <= 8'h00;
      update_mask <= 8'd0;
      sample_count<= 32'd0;
      debug_miso_trace <= 16'd0;
      debug_last_count <= 5'd0;
    end else begin
      tx_dv <= 1'b0; // default: pulse for 1 clk when used

      case (state)
        ST_IDLE: begin
          if (enable && (channel_mask != 8'd0) && interval_elapsed) begin
            // pick next channel
            next_ch      <= find_next_channel(last_channel, channel_mask);
            state        <= ST_ASSERT;
            bytes_sent   <= 2'd0;
            debug_last_count <= 5'd0;
          end
          // Clear requested update bits while idle
          if (clear_update_mask != 0)
            update_mask <= update_mask & ~clear_update_mask;
        end

        ST_ASSERT: begin
          // With CS low, wait for SPI ready then start sending the 3 bytes
          if (tx_ready) begin
            // Byte#1: 0000_0001 (start bit in bit0 per datasheet Fig 6-1)
            tx_byte   <= 8'h01;
            tx_dv     <= 1'b1;
            state     <= ST_SEND1;
          end
        end

        ST_SEND1: begin
          if (rx_dv) begin
            // First RX byte is don't-care for data collection
            rx_b0 <= rx_byte; // keep for debug visibility
            debug_miso_trace[15:8] <= rx_byte;
          end
          if (tx_ready && !tx_dv) begin
            // Byte#2: [SGL=1, D2, D1, D0, 0,0,0,0]
            // Build from next_ch
            tx_byte <= {1'b1, next_ch[2], next_ch[1], next_ch[0], 4'b0000};
            tx_dv   <= 1'b1;
            state   <= ST_SEND2;
          end
        end

        ST_SEND2: begin
          if (rx_dv) begin
            // Contains: [x x x x x null B9 B8]
            rx_b0 <= rx_byte;
            debug_miso_trace[15:8] <= rx_byte;
          end
          if (tx_ready && !tx_dv) begin
            // Byte#3: clock out remaining 8 bits
            tx_byte <= 8'h00;
            tx_dv   <= 1'b1;
            state   <= ST_SEND3;
          end
        end

        ST_SEND3: begin
          if (rx_dv) begin
            rx_b1 <= rx_byte; // B7..B0
            debug_miso_trace[7:0] <= rx_byte;
          end
          // When SPI reports ready again, the 3rd byte is complete
          if (tx_ready && !tx_dv) begin
            state <= ST_DONE;
          end
        end

        ST_DONE: begin
          // Assemble 10-bit result and write to selected channel
          // Per datasheet Fig 6-1 (Mode 0, 8-bit segments):
          // rx_b0[1:0] = B9..B8, rx_b1[7:0] = B7..B0
          // Note: there is a null bit before B9; our rx_b0 captured after Byte#2
          // already aligns so [1:0] are MSBs.
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

          update_mask[next_ch] <= 1'b1; // flag updated channel
          last_channel         <= next_ch;
          sample_count         <= sample_count + 1'b1;
          state                <= ST_IDLE;
        end

        default: state <= ST_IDLE;
      endcase
    end
  end

endmodule
