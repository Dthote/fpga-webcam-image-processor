`timescale 1ns / 1ps

// =============================================================================
// Arty A7-35T  -  UART frame link + hardware auto-exposure / auto-white-balance
//
// Protocol, 921600 baud, 8-N-1
//   PC   -> FPGA : A5 5A, then WIDTH*HEIGHT pixels, RGB565, high byte first
//   FPGA -> PC   : 5A A5                         (ACK after the handshake)
//   FPGA -> PC   : 5A A5, then the corrected frame (same size / format)
//
// Switches (sampled when the frame has been received)
//   sw[0] = colour negative
//   sw[1] = AWB : auto white balance, gray-world (equalise the R, G, B means)
//   sw[2] = AE  : auto exposure (scale so the mean level becomes TARGET)
//
// How it works
//   1. While the frame arrives, three accumulators add up R, G and B
//      (8-bit expanded values) - this is the statistics ("histogram-lite") unit.
//   2. When the frame is complete a sequential divider computes one gain per
//      channel, Q4.12 fixed point (4096 = 1.0, clamped to 8.0):
//          gain_c = (AE ? TARGET*N : sumGray) / (AWB ? sum_c : sumGray)
//      AWB=0 AE=0 -> every gain is exactly 1.0 (pass-through)
//      AWB=1 AE=1 -> gain_c = TARGET / mean_c  (neutral AND correctly exposed)
//   3. Each pixel is multiplied by its gains, rounded, saturated to 255,
//      repacked to RGB565 (and optionally inverted) while it is sent back.
//
// LEDs: led[0]=sw[0]  led[1]=toggles per returned frame  led[2]=UART byte seen
//       led[3]=busy
// =============================================================================
module image_processor #(
    parameter integer CLKS_PER_BIT  = 109,        // 100 MHz / 921600 = 108.5
    parameter integer WIDTH         = 160,
    parameter integer HEIGHT        = 120,
    parameter integer IDLE_TIMEOUT  = 10_000_000, // 100 ms without a byte
    parameter integer TARGET        = 120         // auto-exposure target mean (0..255)
) (
    input  wire       CLK100MHZ,
    input  wire [2:0] sw,
    output wire [3:0] led,
    input  wire       uart_rx,   // FPGA input  <- PC   (pin A9)
    output wire       uart_tx    // FPGA output -> PC   (pin D10)
);
    localparam integer PIXELS   = WIDTH * HEIGHT;
    localparam [63:0]  NUM_AE_W = (64'd1 * TARGET) * PIXELS * 64'd4096;
    localparam [35:0]  MAX_GAIN = 36'd32768;      // 8.0 in Q4.12

    localparam [4:0]
        S_WAIT_HEADER = 5'd0,
        S_ACK1        = 5'd1,
        S_ACK1W       = 5'd2,
        S_ACK2        = 5'd3,
        S_RX_FRAME    = 5'd4,
        S_G0          = 5'd5,
        S_G1          = 5'd6,
        S_G2          = 5'd7,
        S_G3          = 5'd8,
        S_OUT1        = 5'd9,
        S_OUT1W       = 5'd10,
        S_OUT2        = 5'd11,
        S_OUT2W       = 5'd12,
        S_FETCH       = 5'd13,
        S_FETCH1      = 5'd14,
        S_FETCH2      = 5'd15,
        S_MUL         = 5'd16,
        S_SAT         = 5'd17,
        S_HI          = 5'd18,
        S_HIW         = 5'd19,
        S_LO          = 5'd20,
        S_LOW         = 5'd21;

    reg [4:0] state  = S_WAIT_HEADER;
    reg       got_a5 = 1'b0;

    // ---------------- UART cores
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data  = 8'h00;
    reg        tx_start = 1'b0;
    wire       tx_busy, tx_done;

    uart_rx_core #(.CLKS_PER_BIT(CLKS_PER_BIT)) U_RX (
        .clk(CLK100MHZ), .rx(uart_rx), .data_out(rx_data), .valid(rx_valid));

    uart_tx_core #(.CLKS_PER_BIT(CLKS_PER_BIT)) U_TX (
        .clk(CLK100MHZ), .start(tx_start), .data_in(tx_data),
        .tx(uart_tx), .busy(tx_busy), .done(tx_done));

    // ---------------- frame memory (simple dual port, synchronous read)
    (* ram_style = "block" *) reg [15:0] frame_mem [0:PIXELS-1];
    reg [14:0] mem_waddr = 15'd0;
    reg [14:0] mem_raddr = 15'd0;
    reg [15:0] mem_din   = 16'h0000;
    reg        mem_we    = 1'b0;
    reg [15:0] mem_dout  = 16'h0000;

    always @(posedge CLK100MHZ) begin
        if (mem_we) frame_mem[mem_waddr] <= mem_din;
        mem_dout <= frame_mem[mem_raddr];
    end

    // ---------------- misc registers
    reg [14:0] pixel_index  = 15'd0;
    reg        byte_phase   = 1'b0;
    reg [7:0]  high_byte    = 8'h00;
    reg [15:0] tx_pixel     = 16'h0000;
    reg [26:0] idle_cnt     = 27'd0;
    reg        frame_toggle = 1'b0;
    reg        rx_activity  = 1'b0;

    reg [2:0] sw_m = 3'b000, sw_s = 3'b000;
    always @(posedge CLK100MHZ) begin
        sw_m <= sw;
        sw_s <= sw_m;
    end
    always @(posedge CLK100MHZ)
        if (rx_valid) rx_activity <= 1'b1;

    assign led[0] = sw_s[0];
    assign led[1] = frame_toggle;
    assign led[2] = rx_activity;
    assign led[3] = (state != S_WAIT_HEADER);

    // ---------------- statistics unit: running sums of expanded R, G, B
    wire [15:0] rx_pix = {high_byte, rx_data};
    wire [7:0]  rx_r8  = {rx_pix[15:11], rx_pix[15:13]};
    wire [7:0]  rx_g8  = {rx_pix[10:5],  rx_pix[10:9]};
    wire [7:0]  rx_b8  = {rx_pix[4:0],   rx_pix[4:2]};

    reg [23:0] sum_r = 24'd0, sum_g = 24'd0, sum_b = 24'd0;
    reg [25:0] sg3     = 26'd0;           // sum_r + sum_g + sum_b
    reg [41:0] sg_prod = 42'd0;
    wire [23:0] sgm    = sg_prod[39:16];  // sg3 / 3  (x * 21845 >> 16)

    // ---------------- gain unit
    reg        m_awb = 1'b0, m_ae = 1'b0, m_neg = 1'b0;   // modes latched per frame
    reg [1:0]  ch    = 2'd0;
    reg [15:0] gain_r = 16'd4096, gain_g = 16'd4096, gain_b = 16'd4096;
    reg        div_start = 1'b0;
    reg [35:0] div_num   = 36'd0;
    reg [23:0] div_den   = 24'd0;
    wire [35:0] div_q;
    wire        div_done;
    div_u #(.NW(36), .DW(24)) U_DIV (
        .clk(CLK100MHZ), .start(div_start), .num(div_num), .den(div_den),
        .quo(div_q), .done(div_done));
    wire [15:0] gain_new = (div_q > MAX_GAIN) ? MAX_GAIN[15:0] : div_q[15:0];

    // ---------------- per-pixel datapath
    reg [7:0]  r8 = 8'd0, g8 = 8'd0, b8 = 8'd0;
    reg [23:0] pr = 24'd0, pg = 24'd0, pb = 24'd0;

    function [7:0] sat8;                  // (p + 0.5) >> 12, saturated to 255
        input [23:0] p;
        reg   [24:0] t;
        begin
            t    = {1'b0, p} + 25'd2048;
            sat8 = (t[24:12] > 13'd255) ? 8'd255 : t[19:12];
        end
    endfunction

    wire [4:0] o_r5 = sat8(pr) >> 3;
    wire [5:0] o_g6 = sat8(pg) >> 2;
    wire [4:0] o_b5 = sat8(pb) >> 3;
    wire [15:0] out_px = {o_r5, o_g6, o_b5};

    // ---------------- main FSM
    always @(posedge CLK100MHZ) begin
        tx_start  <= 1'b0;
        mem_we    <= 1'b0;
        div_start <= 1'b0;

        if (state != S_RX_FRAME || rx_valid) idle_cnt <= 27'd0;
        else                                 idle_cnt <= idle_cnt + 1'b1;

        case (state)
            // ---- look for A5 5A
            S_WAIT_HEADER: begin
                if (rx_valid) begin
                    if (got_a5 && rx_data == 8'h5A) begin
                        got_a5 <= 1'b0;
                        state  <= S_ACK1;
                    end else
                        got_a5 <= (rx_data == 8'hA5);
                end
            end

            // ---- ACK = 5A A5
            S_ACK1: if (!tx_busy) begin
                tx_data <= 8'h5A; tx_start <= 1'b1; state <= S_ACK1W;
            end
            S_ACK1W: if (tx_done) state <= S_ACK2;
            S_ACK2: if (!tx_busy) begin
                tx_data <= 8'hA5; tx_start <= 1'b1;
                pixel_index <= 15'd0; byte_phase <= 1'b0;
                sum_r <= 24'd0; sum_g <= 24'd0; sum_b <= 24'd0;
                state <= S_RX_FRAME;
            end

            // ---- receive frame, accumulate statistics
            S_RX_FRAME: begin
                if (rx_valid) begin
                    if (!byte_phase) begin
                        high_byte  <= rx_data;
                        byte_phase <= 1'b1;
                    end else begin
                        mem_we     <= 1'b1;
                        mem_waddr  <= pixel_index;
                        mem_din    <= rx_pix;
                        byte_phase <= 1'b0;
                        sum_r <= sum_r + rx_r8;
                        sum_g <= sum_g + rx_g8;
                        sum_b <= sum_b + rx_b8;
                        if (pixel_index == PIXELS-1) begin
                            pixel_index <= 15'd0;
                            state       <= S_G0;
                        end else
                            pixel_index <= pixel_index + 1'b1;
                    end
                end else if (idle_cnt >= IDLE_TIMEOUT) begin
                    state       <= S_WAIT_HEADER;      // aborted transfer
                    got_a5      <= 1'b0;
                    byte_phase  <= 1'b0;
                    pixel_index <= 15'd0;
                end
            end

            // ---- compute the three gains
            S_G0: begin
                m_awb <= sw_s[1];
                m_ae  <= sw_s[2];
                m_neg <= sw_s[0];
                ch    <= 2'd0;
                sg3   <= sum_r + sum_g + sum_b;
                state <= S_G1;
            end
            S_G1: begin
                sg_prod <= sg3 * 16'd21845;            // /3 without a divider
                state   <= S_G2;
            end
            S_G2: begin
                div_num <= m_ae ? NUM_AE_W[35:0] : {sgm, 12'd0};
                if (m_awb) div_den <= (ch == 2'd0) ? sum_r : (ch == 2'd1) ? sum_g : sum_b;
                else       div_den <= sgm;
                div_start <= 1'b1;
                state     <= S_G3;
            end
            S_G3: if (div_done) begin
                case (ch)
                    2'd0:    gain_r <= gain_new;
                    2'd1:    gain_g <= gain_new;
                    default: gain_b <= gain_new;
                endcase
                if (ch == 2'd2) state <= S_OUT1;
                else begin ch <= ch + 1'b1; state <= S_G2; end
            end

            // ---- output header 5A A5
            S_OUT1: if (!tx_busy) begin
                tx_data <= 8'h5A; tx_start <= 1'b1; state <= S_OUT1W;
            end
            S_OUT1W: if (tx_done) state <= S_OUT2;
            S_OUT2: if (!tx_busy) begin
                tx_data <= 8'hA5; tx_start <= 1'b1; state <= S_OUT2W;
            end
            S_OUT2W: if (tx_done) begin
                pixel_index <= 15'd0;
                state       <= S_FETCH;
            end

            // ---- per pixel: read -> expand -> multiply -> saturate/pack -> send
            S_FETCH:  begin mem_raddr <= pixel_index; state <= S_FETCH1; end
            S_FETCH1: state <= S_FETCH2;
            S_FETCH2: begin
                r8 <= {mem_dout[15:11], mem_dout[15:13]};
                g8 <= {mem_dout[10:5],  mem_dout[10:9]};
                b8 <= {mem_dout[4:0],   mem_dout[4:2]};
                state <= S_MUL;
            end
            S_MUL: begin
                pr <= r8 * gain_r;
                pg <= g8 * gain_g;
                pb <= b8 * gain_b;
                state <= S_SAT;
            end
            S_SAT: begin
                tx_pixel <= m_neg ? ~out_px : out_px;
                state    <= S_HI;
            end

            S_HI: if (!tx_busy) begin
                tx_data <= tx_pixel[15:8]; tx_start <= 1'b1; state <= S_HIW;
            end
            S_HIW: if (tx_done) state <= S_LO;
            S_LO: if (!tx_busy) begin
                tx_data <= tx_pixel[7:0]; tx_start <= 1'b1; state <= S_LOW;
            end
            S_LOW: if (tx_done) begin
                if (pixel_index == PIXELS-1) begin
                    pixel_index  <= 15'd0;
                    frame_toggle <= ~frame_toggle;
                    state        <= S_WAIT_HEADER;
                end else begin
                    pixel_index <= pixel_index + 1'b1;
                    state       <= S_FETCH;
                end
            end

            default: state <= S_WAIT_HEADER;
        endcase
    end
endmodule


// =============================================================================
// Sequential restoring divider: quo = num / den  (NW cycles). den = 0 -> all ones
// =============================================================================
module div_u #(
    parameter integer NW = 36,
    parameter integer DW = 24
) (
    input  wire          clk,
    input  wire          start,
    input  wire [NW-1:0] num,
    input  wire [DW-1:0] den,
    output reg  [NW-1:0] quo  = {NW{1'b0}},
    output reg           done = 1'b0
);
    reg [NW-1:0] n_sh = {NW{1'b0}};
    reg [DW-1:0] d    = {DW{1'b0}};
    reg [DW-1:0] rem  = {DW{1'b0}};
    reg [NW-1:0] q    = {NW{1'b0}};
    reg [6:0]    cnt  = 7'd0;
    reg          busy = 1'b0;

    wire [DW:0] r_try = {rem, n_sh[NW-1]};
    wire        ge    = (r_try >= {1'b0, d});

    always @(posedge clk) begin
        done <= 1'b0;
        if (!busy) begin
            if (start) begin
                n_sh <= num;  d <= den;  rem <= {DW{1'b0}};  q <= {NW{1'b0}};
                cnt  <= NW;   busy <= 1'b1;
            end
        end else begin
            n_sh <= {n_sh[NW-2:0], 1'b0};
            rem  <= ge ? (r_try - {1'b0, d}) : r_try[DW-1:0];
            q    <= {q[NW-2:0], ge};
            cnt  <= cnt - 1'b1;
            if (cnt == 7'd1) begin
                busy <= 1'b0;
                done <= 1'b1;
                quo  <= {q[NW-2:0], ge};
            end
        end
    end
endmodule


// =============================================================================
// UART receiver, 8-N-1, samples in the middle of each bit
// =============================================================================
module uart_rx_core #(
    parameter integer CLKS_PER_BIT = 109
) (
    input  wire       clk,
    input  wire       rx,
    output reg  [7:0] data_out = 8'h00,
    output reg        valid    = 1'b0
);
    localparam [1:0] RX_IDLE = 2'd0, RX_START = 2'd1, RX_DATA = 2'd2, RX_STOP = 2'd3;
    reg [1:0]  state     = RX_IDLE;
    reg [15:0] count     = 16'd0;
    reg [2:0]  bit_index = 3'd0;
    reg [7:0]  shift     = 8'h00;
    reg        rx_meta   = 1'b1;
    reg        rx_sync   = 1'b1;

    always @(posedge clk) begin
        rx_meta <= rx;
        rx_sync <= rx_meta;
        valid   <= 1'b0;
        case (state)
            RX_IDLE: if (!rx_sync) begin
                count <= CLKS_PER_BIT/2;
                state <= RX_START;
            end
            RX_START: begin
                if (count == 0) begin
                    if (!rx_sync) begin
                        count     <= CLKS_PER_BIT-1;
                        bit_index <= 3'd0;
                        state     <= RX_DATA;
                    end else state <= RX_IDLE;
                end else count <= count - 1'b1;
            end
            RX_DATA: begin
                if (count == 0) begin
                    shift[bit_index] <= rx_sync;
                    count <= CLKS_PER_BIT-1;
                    if (bit_index == 3'd7) state <= RX_STOP;
                    else bit_index <= bit_index + 1'b1;
                end else count <= count - 1'b1;
            end
            RX_STOP: begin
                if (count == 0) begin
                    if (rx_sync) begin
                        data_out <= shift;
                        valid    <= 1'b1;
                    end
                    state <= RX_IDLE;
                end else count <= count - 1'b1;
            end
            default: state <= RX_IDLE;
        endcase
    end
endmodule


// =============================================================================
// UART transmitter, 8-N-1. 'done' pulses for one cycle when the stop bit ends.
// =============================================================================
module uart_tx_core #(
    parameter integer CLKS_PER_BIT = 109
) (
    input  wire       clk,
    input  wire       start,
    input  wire [7:0] data_in,
    output reg        tx   = 1'b1,
    output reg        busy = 1'b0,
    output reg        done = 1'b0
);
    reg [15:0] count     = 16'd0;
    reg [3:0]  bit_index = 4'd0;
    reg [7:0]  shift     = 8'h00;

    always @(posedge clk) begin
        done <= 1'b0;
        if (!busy) begin
            tx <= 1'b1;
            if (start) begin
                busy      <= 1'b1;
                shift     <= data_in;
                tx        <= 1'b0;
                count     <= CLKS_PER_BIT-1;
                bit_index <= 4'd0;
            end
        end else if (count == 0) begin
            count <= CLKS_PER_BIT-1;
            if (bit_index < 8) begin
                tx        <= shift[0];
                shift     <= {1'b0, shift[7:1]};
                bit_index <= bit_index + 1'b1;
            end else if (bit_index == 8) begin
                tx        <= 1'b1;
                bit_index <= 4'd9;
            end else begin
                busy <= 1'b0;
                done <= 1'b1;
            end
        end else count <= count - 1'b1;
    end
endmodule
