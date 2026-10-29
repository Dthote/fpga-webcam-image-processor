`timescale 1ns/1ps
// =============================================================================
// Self-checking testbench: acts as the PC.
//  - sends a dim, colour-tinted test frame through all 8 switch combinations
//    {AE, AWB, NEG} and checks EVERY returned pixel against a behavioural model
//  - checks: garbage before the header is ignored, an aborted frame recovers
// Run until it prints "ALL TESTS PASSED"  (Vivado: Run Behavioral Simulation, then 'run all')
// =============================================================================
module tb_image_processor #(
    parameter W = 40,          // small frame so simulation is fast
    parameter H = 30,
    parameter CPB = 8,         // clocks per UART bit (fast baud for simulation)
    parameter ONLY_MODE = -1   // >= 0 : run just that switch mode (used for the big frame)
);
    localparam PIX    = W * H;
    localparam TARGET = 120;

    reg clk = 0;
    always #5 clk = ~clk;
    reg  [2:0] sw = 3'b000;
    wire [3:0] led;
    reg  rx_line = 1'b1;       // PC -> FPGA
    wire tx_line;              // FPGA -> PC

    image_processor #(.CLKS_PER_BIT(CPB), .WIDTH(W), .HEIGHT(H),
                      .IDLE_TIMEOUT(3000), .TARGET(TARGET))
        dut (.CLK100MHZ(clk), .sw(sw), .led(led), .uart_rx(rx_line), .uart_tx(tx_line));

    // ---------------- PC-side UART transmit
    task uart_send(input [7:0] b);
        integer i;
        begin
            rx_line = 0; repeat (CPB) @(posedge clk);
            for (i = 0; i < 8; i = i + 1) begin rx_line = b[i]; repeat (CPB) @(posedge clk); end
            rx_line = 1; repeat (CPB) @(posedge clk);
        end
    endtask

    // ---------------- PC-side UART receive into a FIFO
    reg [7:0] fifo [0:2*PIX+63];
    integer wp = 0, rp = 0;
    always begin : rxmon
        reg [7:0] d; integer i;
        @(negedge tx_line);
        repeat (CPB/2) @(posedge clk);
        for (i = 0; i < 8; i = i + 1) begin repeat (CPB) @(posedge clk); d[i] = tx_line; end
        repeat (CPB) @(posedge clk);
        fifo[wp] = d; wp = wp + 1;
    end

    task wait_bytes(input integer n, input integer max_clks, output ok);
        integer t;
        begin
            t = 0;
            while ((wp - rp) < n && t < max_clks) begin @(posedge clk); t = t + 1; end
            ok = ((wp - rp) >= n);
        end
    endtask

    // ---------------- test image: dark, reddish, with a few bright pixels (to saturate)
    reg [15:0] inmem  [0:PIX-1];
    reg [15:0] expmem [0:PIX-1];

    function [15:0] gen_pix(input integer i);
        integer x, y, r, g, b;
        begin
            x = i % W;  y = i / W;
            r = 3 + (x % 7) + ((x * y) % 5);
            g = 6 + ((x + 2 * y) % 11);
            b = 1 + (y % 4);
            if (x == 5) b = b + 20;
            if (i % 17 == 0) begin r = 31; g = 63; b = 20; end
            gen_pix = {r[4:0], g[5:0], b[4:0]};
        end
    endfunction

    // ---------------- behavioural reference model (64-bit integer maths)
    function [63:0] gain_of(input [63:0] num, input [63:0] den);
        reg [63:0] q;
        begin
            if (den == 0) gain_of = 64'd32768;
            else begin
                q = num / den;
                gain_of = (q > 64'd32768) ? 64'd32768 : q;
            end
        end
    endfunction

    function [7:0] apply(input [7:0] v, input [63:0] g);
        reg [63:0] t;
        begin
            t = ((64'd0 + v) * g + 64'd2048) >> 12;
            apply = (t > 255) ? 8'd255 : t[7:0];
        end
    endfunction

    task model(input [2:0] mode);          // mode = {ae, awb, neg}
        integer i;
        reg [63:0] sr, sg, sb, sgm, num, gr, gg, gb;
        reg [7:0]  r8, g8, b8, ro, go, bo;
        reg [15:0] p, o;
        begin
            sr = 0; sg = 0; sb = 0;
            for (i = 0; i < PIX; i = i + 1) begin
                p  = inmem[i];
                r8 = {p[15:11], p[15:13]};  g8 = {p[10:5], p[10:9]};  b8 = {p[4:0], p[4:2]};
                sr = sr + r8;  sg = sg + g8;  sb = sb + b8;
            end
            sgm = ((sr + sg + sb) * 64'd21845) >> 16;
            num = mode[2] ? ((64'd1 * TARGET * PIX) * 64'd4096) : (sgm * 64'd4096);
            gr  = gain_of(num, mode[1] ? sr : sgm);
            gg  = gain_of(num, mode[1] ? sg : sgm);
            gb  = gain_of(num, mode[1] ? sb : sgm);
            for (i = 0; i < PIX; i = i + 1) begin
                p  = inmem[i];
                r8 = {p[15:11], p[15:13]};  g8 = {p[10:5], p[10:9]};  b8 = {p[4:0], p[4:2]};
                ro = apply(r8, gr);  go = apply(g8, gg);  bo = apply(b8, gb);
                o  = {ro[7:3], go[7:2], bo[7:3]};
                expmem[i] = mode[0] ? ~o : o;
            end
        end
    endtask

    // ---------------- one complete PC <-> FPGA exchange
    integer errors = 0, k, fd;
    reg ok;
    reg [15:0] got;

    task transaction(input [2:0] mode, input [255:0] name);
        begin
            repeat (CPB * 14) @(posedge clk);       // let the previous reply finish
            wp = 0; rp = 0;
            sw = mode;
            repeat (10) @(posedge clk);
            model(mode);
            uart_send(8'hA5); uart_send(8'h5A);
            wait_bytes(2, 5000, ok);
            if (!ok || fifo[0] !== 8'h5A || fifo[1] !== 8'hA5) begin
                $display("[%0s] FAIL: no/invalid ACK", name); errors = errors + 1;
            end else begin
                rp = 2;
                for (k = 0; k < PIX; k = k + 1) begin uart_send(inmem[k] >> 8); uart_send(inmem[k] & 8'hFF); end
                wait_bytes(2 + 2*PIX, 100000000, ok);
                if (!ok) begin
                    $display("[%0s] FAIL: output frame incomplete (%0d/%0d)", name, wp-rp, 2+2*PIX); errors = errors + 1;
                end else begin
                    if (fifo[rp] !== 8'h5A || fifo[rp+1] !== 8'hA5) begin $display("[%0s] FAIL: bad frame header", name); errors = errors + 1; end
                    for (k = 0; k < PIX; k = k + 1) begin
                        got = {fifo[rp+2+2*k], fifo[rp+3+2*k]};
`ifdef DUMP
                        $fdisplay(fd, "%0d %04h", mode, got);
`endif
                        if (got !== expmem[k]) begin
                            errors = errors + 1;
                            if (errors < 8) $display("[%0s] pixel %0d: got %h expected %h (input %h)", name, k, got, expmem[k], inmem[k]);
                        end
                    end
                    $display("[%0s] sw=%b  %0d pixels checked", name, mode, PIX);
                end
            end
        end
    endtask

    integer m, i;
    initial begin
        for (i = 0; i < PIX; i = i + 1) inmem[i] = gen_pix(i);
`ifdef DUMP
        fd = $fopen("rtl_out.hex", "w");
        begin : dump_in
            integer fi; fi = $fopen("rtl_in.hex", "w");
            for (i = 0; i < PIX; i = i + 1) $fdisplay(fi, "%04h", inmem[i]);
            $fclose(fi);
        end
`endif
        repeat (50) @(posedge clk);

        if (ONLY_MODE >= 0) transaction(ONLY_MODE, "single mode");
        else begin
            for (m = 0; m < 8; m = m + 1) transaction(m[2:0], "mode");

            // garbage before the header must be ignored
            uart_send(8'h00); uart_send(8'hA5); uart_send(8'hA5);
            transaction(3'b110, "after garbage");

            // aborted frame: handshake, send only 5 bytes, then stall -> FPGA must resync
            repeat (CPB * 14) @(posedge clk); wp = 0; rp = 0;
            uart_send(8'hA5); uart_send(8'h5A);
            wait_bytes(2, 5000, ok);
            for (i = 0; i < 5; i = i + 1) uart_send(8'h77);
            repeat (6000) @(posedge clk);
            if (led[3]) begin errors = errors + 1; $display("FAIL: FPGA stuck after aborted frame"); end
            else $display("[aborted frame] FPGA resynchronised (led3 = 0)");
            transaction(3'b111, "after abort");
        end

`ifdef DUMP
        $fclose(fd);
`endif
        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("TESTS FAILED: %0d errors", errors);
        $finish;
    end
    initial begin #4_000_000_000; $display("TIMEOUT"); $finish; end
endmodule
