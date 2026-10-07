// Adapted only at the DUT boundary and RTL-internal diagnostic references.
// The sixteen original Wishbone testcase transactions and expected results are
// otherwise unchanged.  Customer-visible behavior is checked at the direct
// Neuromorphic_X2_wb interface.


//--------------------------------------------------------------------------------------------------
//  RISC-V / Wishbone driven testbench
//
//  Description:
//    - 10 MHz Wishbone clock
//    - Reset generation
//    - Wishbone master write/read tasks
//    - Waits for DUT ACK instead of using arbitrary delays
//    - Sends data_packets[] through Wishbone
//    - SET/RESET automatic FIFO readback is sampled after the confirmed 400-clock update time
//
//  Wishbone packet register:
//      32'h3000_0004
//
//--------------------------------------------------------------------------------------------------

`timescale 1ns/1ps

module Neuromorphic_X2_wb_tb;

  //==============================================================================================
  // Parameters
  //==============================================================================================

  localparam logic [31:0] P_BS_ADR = 32'h3000_0000;

  // Packet/data register
  localparam logic [31:0] WB_DATA_ADR = P_BS_ADR + 32'h04;


  //==============================================================================================
  // Clock / Reset
  //==============================================================================================

  logic l_usr_clk;
  logic reset;


  //==============================================================================================
  // Test / Scan signals
  //==============================================================================================

  logic TM;
  logic scan_se1;
  logic scan_si1;


  //==============================================================================================
  // Outputs from DUT
  //==============================================================================================

  wire        TM_o;

  wire [31:0] bl_addr;
  wire [31:0] wl_addr;
  wire [31:0] sl_addr;

  wire        start_i;

  wire [31:0] o_sl_data;
  wire [31:0] o_wl_data;
  wire [31:0] o_bl_data;

  wire [31:0] o_sl_float;
  wire [31:0] o_bl_float;
  wire [31:0] o_wl_float;

  wire [1:0]  o_mux_sel;

  wire        o_precharge;
  wire        o_hold;
  wire        o_ramp;
  wire        o_start;
	
	wire [31:0] pwm_pulse_oo;

  wire o_v_count_0;


  //==============================================================================================
  // Wishbone signals
  //
  // These signals are driven exactly like a simple RISC-V Wishbone master.
  //==============================================================================================

  logic [3:0]  l_wbs_sel;

  logic [31:0] l_wb_addr;
  logic [31:0] li_wb_data;

  logic        l_wb_ack;
  logic        l_wb_cyc;
  logic        l_wb_stb;
  logic        l_wb_we;

  logic [31:0] lo_wb_data;


  //==============================================================================================
  // Stop bit
  //==============================================================================================

  logic [31:0] stop_bit;


  //==============================================================================================
  // Data packets
  //==============================================================================================

  logic [31:0] data_packets [0:300];

  integer packet_index;

  integer total_packets = 15;

  // Focused testcase selection:
  //   0 = run all sixteen cases
  //   1..16 = run only that numbered case
  // Default to the newly added testcase 16. This can still be overridden with
  // the command-line plusarg +CASE=<0..16>.
  integer selected_case = 0;

  localparam logic [31:0] EMPTY_WORD = 32'hA000_0000;
  localparam logic [1:0] MODE_RESET   = 2'b00;
  localparam logic [1:0] MODE_READ    = 2'b01;
  localparam logic [1:0] MODE_COMPUTE = 2'b10;
  localparam logic [1:0] MODE_SET     = 2'b11;

  // SET and RESET perform their own readback and place one result in the
  // operation FIFO.  The reference RTL test requires the complete 400-clock
  // cell-update interval before software reads that result through Wishbone.
  localparam integer SET_RESET_READBACK_WAIT_CYCLES = 400;

  integer total_tests  = 0;
  integer passed_tests = 0;
  integer failed_tests = 0;
  integer failed_checks = 0;


  //==============================================================================================
  // Current RTL command packet builder
  //==============================================================================================

  function automatic logic [31:0] command_packet(
    input logic [1:0] mode,
    input integer row,
    input integer column,
    input integer pwm_value,
    input bit bit18_full_column,
    input bit bit17_last_packet
  );

    begin

      command_packet = {mode,
                        row[4:0],
                        column[4:0],
                        1'b0,
                        bit18_full_column,
                        bit17_last_packet,
                        9'b0,
                        pwm_value[7:0]};

    end

  endfunction


  function automatic integer result_column(input logic [31:0] result_word);

    begin
      result_column = (result_word >> 14) & 31;
    end

  endfunction


  //==============================================================================================
  // Clock generation
  //
  // 10 MHz:
  // Period = 100 ns
  // Half period = 50 ns
  //==============================================================================================

  initial begin
    l_usr_clk = 1'b0;
  end

  always begin
    #50 l_usr_clk = ~l_usr_clk;
  end


  //==============================================================================================
  // DUT
  //==============================================================================================

  Neuromorphic_X2_wb uut (
    .user_clk       (l_usr_clk       ),
    .user_rst       (reset           ),
    .Iref           (1'b0            ),
    .Vcc_read       (1'b0            ),
    .Vcomp          (1'b0            ),
    .Bias_comp2     (1'b0            ),
    .Vcc_wl_read    (1'b0            ),
    .Vcc_wl_set     (1'b0            ),
    .Vbias          (1'b0            ),
    .Vcc_wl_reset   (1'b0            ),
    .Vcc_set        (1'b0            ),
    .dc_bias        (1'b0            ),

    .wb_clk_i        (l_usr_clk       ),
    .wb_rst_i        (reset           ),

    .ScanInDL        (scan_si1        ),
    .ScanInDR        (scan_se1        ),
    .TM              (TM              ),

    .ScanInCC        (),
    .ScanOutCC       (),

    .bl_data_o       (o_bl_data       ),
    .sl_data_o       (o_sl_data       ),

    .wl_float_o      (o_wl_float      ),
    .sl_float_o      (o_sl_float      ),
    .bl_float_o      (o_bl_float      ),

    .ramp_o          (o_ramp          ),

    .bl_addr_o       (bl_addr         ),
    .wl_addr_o       (wl_addr         ),
    .sl_addr_o       (sl_addr         ),

    .stop_bit_i      (stop_bit        ),

    // Wishbone master -> DUT
    .wbs_cyc_i       (l_wb_cyc        ),
    .wbs_stb_i       (l_wb_stb        ),
    .wbs_we_i        (l_wb_we         ),
    .wbs_sel_i       (l_wbs_sel       ),
    .wbs_dat_i       (li_wb_data      ),
    .wbs_adr_i       (l_wb_addr       ),

    // DUT -> Wishbone master
    .wbs_ack_o       (l_wb_ack        ),
    .wbs_dat_o       (lo_wb_data      ),

    .pwm_pulse_o     (pwm_pulse_oo),
    .v_count_o       (o_v_count_0     ),

    .precharge_o     (o_precharge     ),
    .hold_o          (o_hold          ),
    .start_o         (o_start         ),
    .mux_sel_o       (o_mux_sel       ),
    .wl_data_o       (o_wl_data       ),

    .TM_o            (TM_o            )
  );

  // Result writes are counted by private behavioral-model diagnostic state.

  //==============================================================================================
  // Stop-bit generation
  //==============================================================================================
  //
  // Keep your original behavior.
  //
  // When ramp is active, capture bl_addr after 240 ns.
  //
  //==============================================================================================

  always @(posedge l_usr_clk) begin

    if (o_ramp == 1'b1) begin

      #240;

      stop_bit[0]  <= bl_addr[0];
      stop_bit[1]  <= bl_addr[1];
      stop_bit[2]  <= bl_addr[2];
      stop_bit[3]  <= bl_addr[3];
      stop_bit[4]  <= bl_addr[4];
      stop_bit[5]  <= bl_addr[5];
      stop_bit[6]  <= bl_addr[6];
      stop_bit[7]  <= bl_addr[7];
      stop_bit[8]  <= bl_addr[8];
      stop_bit[9]  <= bl_addr[9];
      stop_bit[10] <= bl_addr[10];
      stop_bit[11] <= bl_addr[11];
      stop_bit[12] <= bl_addr[12];
      stop_bit[13] <= bl_addr[13];
      stop_bit[14] <= bl_addr[14];
      stop_bit[15] <= bl_addr[15];
      stop_bit[16] <= bl_addr[16];
      stop_bit[17] <= bl_addr[17];
      stop_bit[18] <= bl_addr[18];
      stop_bit[19] <= bl_addr[19];
      stop_bit[20] <= bl_addr[20];
      stop_bit[21] <= bl_addr[21];
      stop_bit[22] <= bl_addr[22];
      stop_bit[23] <= bl_addr[23];
      stop_bit[24] <= bl_addr[24];
      stop_bit[25] <= bl_addr[25];
      stop_bit[26] <= bl_addr[26];
      stop_bit[27] <= bl_addr[27];
      stop_bit[28] <= bl_addr[28];
      stop_bit[29] <= bl_addr[29];
      stop_bit[30] <= bl_addr[30];
      stop_bit[31] <= bl_addr[31];

    end

    else begin

      stop_bit <= 32'b0;

    end

  end


  //==============================================================================================
  // Wishbone IDLE
  //==============================================================================================

  task automatic wb_idle;

    begin

      l_wb_cyc   = 1'b0;
      l_wb_stb   = 1'b0;
      l_wb_we    = 1'b0;

      l_wbs_sel  = 4'b0000;

      l_wb_addr  = 32'h0000_0000;
      li_wb_data = 32'h0000_0000;

    end

  endtask


  //==============================================================================================
  // Wishbone WRITE
  //==============================================================================================
  //
  // Sequence:
  //
  //      Address
  //      Data
  //      WE = 1
  //      CYC = 1
  //      STB = 1
  //             |
  //             | wait
  //             v
  //           ACK = 1
  //             |
  //             v
  //          transaction complete
  //
  //==============================================================================================

  task automatic wb_write(
    input logic [31:0] addr,
    input logic [31:0] data
  );

    integer timeout_count;

    begin

      $display("");
      $display("------------------------------------------------------------");
      $display("[%0t] WB WRITE START", $time);
      $display("        ADDR = %08h", addr);
      $display("        DATA = %08h", data);
      $display("------------------------------------------------------------");


      //--------------------------------------------------------------------------
      // Make sure previous transaction is finished
      //--------------------------------------------------------------------------

      l_wb_cyc = 1'b0;
      l_wb_stb = 1'b0;
      l_wb_we  = 1'b0;

      @(posedge l_usr_clk);


      //--------------------------------------------------------------------------
      // Drive address/data/control
      //--------------------------------------------------------------------------

      l_wb_addr  = addr;
      li_wb_data = data;

      l_wbs_sel  = 4'b1111;

      l_wb_we    = 1'b1;

      //--------------------------------------------------------------------------
      // Start Wishbone transaction
      //--------------------------------------------------------------------------

      l_wb_cyc = 1'b1;
      l_wb_stb = 1'b1;


      //--------------------------------------------------------------------------
      // Wait for ACK
      //
      // The DUT is allowed to take multiple clocks.
      //--------------------------------------------------------------------------

      timeout_count = 0;

      while (l_wb_ack !== 1'b1) begin

        @(posedge l_usr_clk);

        timeout_count = timeout_count + 1;

        if (timeout_count > 1000) begin

          $error("[%0t] WB WRITE TIMEOUT! ADDR=%08h DATA=%08h",
                 $time,
                 addr,
                 data);

          $finish;

        end

      end


      //--------------------------------------------------------------------------
      // ACK received
      //--------------------------------------------------------------------------

      $display("[%0t] WB WRITE ACK", $time);


      //--------------------------------------------------------------------------
      // End transaction
      //--------------------------------------------------------------------------

      //@(posedge l_usr_clk);

      l_wb_stb = 1'b0;
      l_wb_cyc = 1'b0;
      l_wb_we  = 1'b0;


      //--------------------------------------------------------------------------
      // One idle clock between transfers
      //--------------------------------------------------------------------------

      @(posedge l_usr_clk);


      $display("[%0t] WB WRITE COMPLETE", $time);

    end

  endtask


  //==============================================================================================
  // Wishbone READ
  //==============================================================================================

  task automatic wb_read(
    input  logic [31:0] addr,
    output logic [31:0] data
  );

    integer timeout_count;

    begin

      $display("");
      $display("------------------------------------------------------------");
      $display("[%0t] WB READ START", $time);
      $display("        ADDR = %08h", addr);
      $display("------------------------------------------------------------");


      //--------------------------------------------------------------------------
      // Previous transaction must be idle
      //--------------------------------------------------------------------------

      l_wb_cyc = 1'b0;
      l_wb_stb = 1'b0;
      l_wb_we  = 1'b0;

      @(posedge l_usr_clk);


      //--------------------------------------------------------------------------
      // Drive read transaction
      //--------------------------------------------------------------------------

      l_wb_addr  = addr;
      li_wb_data = 32'h0000_0000;

      l_wbs_sel = 4'b1111;

      l_wb_we = 1'b0;

      l_wb_cyc = 1'b1;
      l_wb_stb = 1'b1;


      //--------------------------------------------------------------------------
      // Wait for ACK
      //--------------------------------------------------------------------------

      timeout_count = 0;

      while (l_wb_ack !== 1'b1) begin

        @(posedge l_usr_clk);

        timeout_count = timeout_count + 1;

        if (timeout_count > 1000) begin

          $error("[%0t] WB READ TIMEOUT! ADDR=%08h",
                 $time,
                 addr);

          $finish;

        end

      end


      //--------------------------------------------------------------------------
      // Capture read data
      //--------------------------------------------------------------------------

      data = lo_wb_data;

      $display("[%0t] WB READ ACK DATA=%08h",
               $time,
               data);


      //--------------------------------------------------------------------------
      // End transaction
      //--------------------------------------------------------------------------

      //@(posedge l_usr_clk);

      l_wb_stb = 1'b0;
      l_wb_cyc = 1'b0;


      @(posedge l_usr_clk);


      $display("[%0t] WB READ COMPLETE", $time); 
      

    end

  endtask


  //==============================================================================================
  // Reset task
  //==============================================================================================

  task automatic apply_reset;

    begin

      $display("");
      $display("============================================================");
      $display("[%0t] APPLY RESET", $time);
      $display("============================================================");

      reset = 1'b1;

      // Reset for 4 clock cycles
      repeat (4) @(posedge l_usr_clk);

      reset = 1'b0;

      @(posedge l_usr_clk);

      $display("[%0t] RESET RELEASED", $time);

    end

  endtask


  //==============================================================================================
  // Focused testcase support
  //==============================================================================================

  task automatic check_condition(
    input bit condition,
    input string description
  );

    begin

      if (!condition) begin
        failed_checks = failed_checks + 1;
        $error("CHECK FAILED: %s", description);
      end

    end

  endtask


  task automatic begin_test(
    input string test_name,
    output integer failure_mark
  );

    begin

      total_tests = total_tests + 1;
      failure_mark = failed_checks;

      $display("");
      $display("================================================================================");
      $display("TEST START: %s", test_name);
      $display("================================================================================");

    end

  endtask


  task automatic end_test(
    input string test_name,
    input integer failure_mark
  );

    begin

      if (failed_checks == failure_mark) begin
        passed_tests = passed_tests + 1;
        $display("TEST PASS: %s", test_name);
      end
      else begin
        failed_tests = failed_tests + 1;
        $display("TEST FAIL: %s", test_name);
      end

    end

  endtask


  // Send only the three configuration packets. The original wb_write handshake is used unchanged.
  task automatic send_three_configuration_packets;

    begin

      $display("[%0t] CONFIG 1 = 00036472", $time);
      wb_write(WB_DATA_ADR, 32'h0003_6472);
      repeat (2) @(posedge l_usr_clk);

      $display("[%0t] CONFIG 2 = 462B000B", $time);
      wb_write(WB_DATA_ADR, 32'h462B_000B);
      repeat (2) @(posedge l_usr_clk);

      $display("[%0t] CONFIG 3 = 83E01405", $time);
      wb_write(WB_DATA_ADR,
               {2'b10,
                10'd62,
                10'd05,
                10'd05});
      repeat (2) @(posedge l_usr_clk);

    end

  endtask


  task automatic expect_empty_read(input string description);
    logic [31:0] read_data;

    begin

      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data === EMPTY_WORD,
                      $sformatf("%s actual=%08h expected=A0000000",
                                description,
                                read_data));

    end

  endtask


  task automatic testcase_1_three_configuration_packets;

    integer failure_mark;
    string test_name;

    begin

      test_name = "CASE 1 - send three configuration packets";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();
      repeat (10) @(posedge l_usr_clk);

      check_condition({uut.target_set2,
                       uut.target_set1} === 32'h0003_6472,
                      "SET configuration registers decoded correctly");
      check_condition({uut.target_reset2,
                       uut.target_reset1} === 32'h462B_000B,
                      "RESET configuration registers decoded correctly");
      check_condition(uut.tdc_time_out === 7'd62,
                      "TDC timeout configuration decoded correctly");

      end_test(test_name, failure_mark);

    end

  endtask


  task automatic testcase_2_config_then_empty_read;

    integer failure_mark;
    string test_name;

    begin

      test_name = "CASE 2 - configure then read empty FIFO";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();
      expect_empty_read("FIFO must be empty after configuration only");

      end_test(test_name, failure_mark);

    end

  endtask


  task automatic testcase_3_reset_then_empty_read;

    integer failure_mark;
    string test_name;

    begin

      test_name = "CASE 3 - reset then read empty FIFO";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      expect_empty_read("FIFO must be empty immediately after reset");

      end_test(test_name, failure_mark);

    end

  endtask


  task automatic testcase_4_set_read_reset_read;

    integer failure_mark;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 4 - configure, SET readback, RESET readback";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      $display("CASE 4: SET row=2 column=4 PWM=3");
      wb_write(WB_DATA_ADR, command_packet(MODE_SET, 2, 4, 3, 0, 0));
      $display("CASE 4: wait %0d clocks for automatic SET readback",
               SET_RESET_READBACK_WAIT_CYCLES);
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);
      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "automatic SET readback must return a result");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 4,
                        "SET readback result column must be 4");

      $display("CASE 4: RESET row=2 column=4 PWM=9");
      wb_write(WB_DATA_ADR, command_packet(MODE_RESET, 2, 4, 9, 0, 0));
      $display("CASE 4: wait %0d clocks for automatic RESET readback",
               SET_RESET_READBACK_WAIT_CYCLES);
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);
      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "automatic RESET readback must return a result");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 4,
                        "RESET readback result column must be 4");

      end_test(test_name, failure_mark);

    end

  endtask


  // This case intentionally follows the requested packet fields exactly. Bit18 requests all
  // columns, but bit17 remains zero. In the current RTL this burst must remain buffered.
  task automatic testcase_5_six_compute_sixth_bit18;

    integer failure_mark;
    integer row;
    string test_name;

    begin

      test_name = "CASE 5 - six COMPUTE packets, bit18 on packet 6";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      for (row = 0; row < 6; row = row + 1) begin

        $display("CASE 5: COMPUTE %0d row=%0d col=%0d bit18=%0d bit17=0",
                 row + 1,
                 row,
                 row + 2,
                 row == 5);

        wb_write(WB_DATA_ADR,
                 command_packet(MODE_COMPUTE,
                                row,
                                row + 2,
                                8'h10 + row,
                                row == 5,
                                0));

        repeat (2) @(posedge l_usr_clk);

      end

      repeat (200) @(posedge l_usr_clk);

      check_condition(uut.compute_packet_count == 6,
                      "six packets remain accumulated without bit17");
      check_condition((uut.compute_packet_count != 0) == 1'b1,
                      "compute parser waits for bit17 end marker");
      check_condition(uut.compute_full_column == 1'b1,
                      "packet-6 bit18 full-column request is retained");
      expect_empty_read("bit18 without bit17 must not produce output data");

      end_test(test_name, failure_mark);

    end

  endtask


  task automatic testcase_6_two_compute_bit18_bit17_full_column;

    integer failure_mark;
    integer column;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 6 - two COMPUTE packets, packet 2 bit18=1 bit17=1";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      $display("CASE 6: COMPUTE 1 row=1 col=2 PWM=11 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 1, 2, 8'h11, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 6: COMPUTE 2 row=2 col=5 PWM=22 bit18=1 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 2, 5, 8'h22, 1, 1));
      repeat (500) @(posedge l_usr_clk);

      check_condition(uut.last_compute_full_column == 1'b1,
                      "bit18 selects the complete 32-column result window");

      for (column = 0; column < 32; column = column + 1) begin

        wb_read(WB_DATA_ADR, read_data);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("full-column result %0d must be present", column));

        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == column,
                          $sformatf("full-column result order expected=%0d actual=%0d",
                                    column,
                                    result_column(read_data)));

      end

      expect_empty_read("FIFO must be empty after 32 full-column results");

      end_test(test_name, failure_mark);

    end

  endtask


  task automatic testcase_7_set_read_reset_read_then_empty;

    integer failure_mark;
    integer empty_index;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 7 - SET/RESET automatic readbacks, then empty FIFO reads";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      $display("CASE 7: SET row=2 column=4 PWM=3");
      wb_write(WB_DATA_ADR, command_packet(MODE_SET, 2, 4, 3, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);
      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "CASE 7 automatic SET readback must return data");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 4,
                        "CASE 7 SET readback column must be 4");

      $display("CASE 7: RESET row=2 column=4 PWM=9");
      wb_write(WB_DATA_ADR, command_packet(MODE_RESET, 2, 4, 9, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);
      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "CASE 7 automatic RESET readback must return data");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 4,
                        "CASE 7 RESET readback column must be 4");

      for (empty_index = 0; empty_index < 5; empty_index = empty_index + 1) begin

        expect_empty_read($sformatf("CASE 7 empty FIFO read %0d",
                                    empty_index + 1));

      end

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 8 explicitly separates the automatic SET-side and RESET-side FIFO readbacks:
  //   SET -> wait 400 clocks -> three WB reads
  //   RESET -> wait 400 clocks -> three WB reads
  // Exactly one result is expected from each programming operation. The two
  // additional reads prove that no duplicate or stale result remains.
  task automatic testcase_8_three_reads_after_each_program_phase;

    integer failure_mark;
    integer read_index;
    integer set_non_empty_count;
    integer reset_non_empty_count;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 8 - three WB reads after automatic SET/RESET readback";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      //--------------------------------------------------------------------------
      // SET phase
      //--------------------------------------------------------------------------

      $display("CASE 8 SET PHASE: SET row=2 column=4 PWM=3");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 2, 4, 3, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      set_non_empty_count = 0;

      for (read_index = 0; read_index < 3; read_index = read_index + 1) begin

        wb_read(WB_DATA_ADR, read_data);

        $display("CASE 8 SET PHASE: WB READ %0d DATA=%08h",
                 read_index + 1,
                 read_data);

        if (read_data !== EMPTY_WORD) begin
          set_non_empty_count = set_non_empty_count + 1;
          check_condition(result_column(read_data) == 4,
                          $sformatf("SET-phase WB read %0d must have column tag 4",
                                    read_index + 1));
        end

      end

      check_condition(set_non_empty_count == 1,
                      $sformatf("SET must produce exactly one automatic readback, actual=%0d",
                                set_non_empty_count));

      //--------------------------------------------------------------------------
      // RESET phase
      //--------------------------------------------------------------------------

      $display("CASE 8 RESET PHASE: RESET row=2 column=4 PWM=9");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 2, 4, 9, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      reset_non_empty_count = 0;

      for (read_index = 0; read_index < 3; read_index = read_index + 1) begin

        wb_read(WB_DATA_ADR, read_data);

        $display("CASE 8 RESET PHASE: WB READ %0d DATA=%08h",
                 read_index + 1,
                 read_data);

        if (read_data !== EMPTY_WORD) begin
          reset_non_empty_count = reset_non_empty_count + 1;
          check_condition(result_column(read_data) == 4,
                          $sformatf("RESET-phase WB read %0d must have column tag 4",
                                    read_index + 1));
        end

      end

      $display("CASE 8 RESULT COUNTS: SET phase non-empty=%0d, RESET phase non-empty=%0d",
               set_non_empty_count,
               reset_non_empty_count);

      check_condition(reset_non_empty_count == 1,
                      $sformatf("RESET must produce exactly one automatic readback, actual=%0d",
                                reset_non_empty_count));

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 9 proves that SET and RESET independently create one automatic
  // operation-FIFO readback after the required 400-clock cell-update delay.
  // No explicit MODE_READ command is sent in either phase.
  task automatic testcase_9_isolate_set_reset_automatic_readback;

    integer failure_mark;
    integer writes_before_phase;
    integer set_phase_fifo_writes;
    integer reset_phase_fifo_writes;
    string test_name;
    logic [31:0] set_result;
    logic [31:0] reset_result;

    begin

      test_name = "CASE 9 - isolate automatic SET and RESET readbacks";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      //--------------------------------------------------------------------------
      // Phase 1: SET only, wait for completion, then read its automatic result.
      //--------------------------------------------------------------------------

      writes_before_phase = uut.response_write_count_total;

      $display("CASE 9 SET PHASE: SET row=2 column=4 PWM=3");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 2, 4, 3, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      set_phase_fifo_writes = uut.response_write_count_total - writes_before_phase;

      $display("CASE 9 SET PHASE: result-FIFO write pulses=%0d",
               set_phase_fifo_writes);

      wb_read(WB_DATA_ADR, set_result);

      $display("CASE 9 SET PHASE: automatic readback DATA=%08h",
               set_result);
      $display("CASE 9 SET PHASE: FIFO empty after consuming readback=%b",
               uut.response_empty);

      check_condition(set_phase_fifo_writes == 1,
                      $sformatf("SET must generate exactly one result-FIFO write, actual=%0d",
                                set_phase_fifo_writes));
      check_condition(set_result !== EMPTY_WORD,
                      "automatic SET readback must return data");
      if (set_result !== EMPTY_WORD)
        check_condition(result_column(set_result) == 4,
                        "automatic SET readback must have column tag 4");
      expect_empty_read("CASE 9 FIFO must be empty after consuming SET readback");

      //--------------------------------------------------------------------------
      // Phase 2: RESET only, wait for completion, then read its automatic result.
      //--------------------------------------------------------------------------

      writes_before_phase = uut.response_write_count_total;

      $display("CASE 9 RESET PHASE: RESET row=3 column=6 PWM=9");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 3, 6, 9, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      reset_phase_fifo_writes = uut.response_write_count_total - writes_before_phase;

      $display("CASE 9 RESET PHASE: result-FIFO write pulses=%0d",
               reset_phase_fifo_writes);

      wb_read(WB_DATA_ADR, reset_result);

      $display("CASE 9 RESET PHASE: automatic readback DATA=%08h",
               reset_result);

      check_condition(reset_phase_fifo_writes == 1,
                      $sformatf("RESET must generate exactly one result-FIFO write, actual=%0d",
                                reset_phase_fifo_writes));
      check_condition(reset_result !== EMPTY_WORD,
                      "automatic RESET readback must return data");

      if (reset_result !== EMPTY_WORD)
        check_condition(result_column(reset_result) == 6,
                        "automatic RESET readback must have column tag 6");

      $display("CASE 9 COMPARISON:");
      $display("  SET automatic result   = %08h", set_result);
      $display("  RESET automatic result = %08h", reset_result);
      $display("  SET FIFO writes        = %0d", set_phase_fifo_writes);
      $display("  RESET FIFO writes      = %0d", reset_phase_fifo_writes);

      expect_empty_read("CASE 9 FIFO must be empty after consuming RESET readback");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 10 verifies a non-full-column compute burst with six different rows but
  // one repeated column. Only one TDC/result is expected because the RTL combines
  // compute column addresses into a bit mask before starting the TDC counters.
  task automatic testcase_10_compute_six_rows_same_column;

    integer failure_mark;
    integer packet_number;
    integer writes_before_phase;
    integer compute_fifo_writes;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 10 - six COMPUTE packets, same column, bit17 on packet 6";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      writes_before_phase = uut.response_write_count_total;

      for (packet_number = 0; packet_number < 6; packet_number = packet_number + 1) begin

        $display("CASE 10: COMPUTE %0d row=%0d col=7 PWM=%02h bit18=0 bit17=%0d",
                 packet_number + 1,
                 packet_number,
                 8'h21 + packet_number,
                 packet_number == 5);

        wb_write(WB_DATA_ADR,
                 command_packet(MODE_COMPUTE,
                                packet_number,
                                7,
                                8'h21 + packet_number,
                                0,
                                packet_number == 5));

        repeat (2) @(posedge l_usr_clk);

      end

      repeat (500) @(posedge l_usr_clk);

      compute_fifo_writes = uut.response_write_count_total - writes_before_phase;

      check_condition(uut.last_compute_full_column == 1'b0,
                      "CASE 10 must remain a selected-column compute");
      check_condition(uut.last_compute_row_mask == 32'h0000_003F,
                      "CASE 10 row mask must contain rows 0 through 5");
      check_condition(uut.last_compute_col_mask == 32'h0000_0080,
                      "CASE 10 column mask must contain only column 7");
      check_condition(compute_fifo_writes == 1,
                      $sformatf("CASE 10 expected one result-FIFO write, actual=%0d",
                                compute_fifo_writes));

      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "CASE 10 selected column 7 result must be present");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 7,
                        $sformatf("CASE 10 expected result column 7, actual=%0d",
                                  result_column(read_data)));

      expect_empty_read("CASE 10 FIFO must be empty after one selected-column result");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 11 verifies a non-full-column compute burst using four different rows
  // and four different columns. The result FIFO must contain one result for each
  // unique selected column, returned in ascending column order.
  task automatic testcase_11_compute_four_rows_four_columns;

    integer failure_mark;
    integer read_index;
    integer expected_column;
    integer writes_before_phase;
    integer compute_fifo_writes;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 11 - four COMPUTE packets, varied rows/columns, bit17 on packet 4";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      writes_before_phase = uut.response_write_count_total;

      $display("CASE 11: COMPUTE 1 row=2 col=3 PWM=31 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 2, 3, 8'h31, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 11: COMPUTE 2 row=4 col=9 PWM=32 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 4, 9, 8'h32, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 11: COMPUTE 3 row=6 col=14 PWM=33 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 6, 14, 8'h33, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 11: COMPUTE 4 row=8 col=21 PWM=34 bit18=0 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 8, 21, 8'h34, 0, 1));

      repeat (500) @(posedge l_usr_clk);

      compute_fifo_writes = uut.response_write_count_total - writes_before_phase;

      check_condition(uut.last_compute_full_column == 1'b0,
                      "CASE 11 must remain a selected-column compute");
      check_condition(uut.last_compute_row_mask == 32'h0000_0154,
                      "CASE 11 row mask must contain rows 2, 4, 6 and 8");
      check_condition(uut.last_compute_col_mask == 32'h0020_4208,
                      "CASE 11 column mask must contain columns 3, 9, 14 and 21");
      check_condition(compute_fifo_writes == 4,
                      $sformatf("CASE 11 expected four result-FIFO writes, actual=%0d",
                                compute_fifo_writes));

      for (read_index = 0; read_index < 4; read_index = read_index + 1) begin

        case (read_index)
          0: expected_column = 3;
          1: expected_column = 9;
          2: expected_column = 14;
          default: expected_column = 21;
        endcase

        wb_read(WB_DATA_ADR, read_data);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("CASE 11 result %0d must be present",
                                  read_index + 1));

        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == expected_column,
                          $sformatf("CASE 11 result %0d expected column=%0d actual=%0d",
                                    read_index + 1,
                                    expected_column,
                                    result_column(read_data)));

      end

      expect_empty_read("CASE 11 FIFO must be empty after four selected-column results");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 12 verifies a three-packet compute burst whose last packet sets both
  // bit18 (full-column request) and bit17 (last-packet/end marker). Independent
  // of the three packet column fields, the RTL must return all 32 columns.
  task automatic testcase_12_compute_three_packets_full_column;

    integer failure_mark;
    integer column;
    integer writes_before_phase;
    integer compute_fifo_writes;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 12 - three COMPUTE packets, full column, bits18/17 on packet 3";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      writes_before_phase = uut.response_write_count_total;

      $display("CASE 12: COMPUTE 1 row=1 col=2 PWM=41 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 1, 2, 8'h41, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 12: COMPUTE 2 row=10 col=11 PWM=42 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 10, 11, 8'h42, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 12: COMPUTE 3 row=20 col=23 PWM=43 bit18=1 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 20, 23, 8'h43, 1, 1));

      repeat (500) @(posedge l_usr_clk);

      compute_fifo_writes = uut.response_write_count_total - writes_before_phase;

      check_condition(uut.last_compute_full_column == 1'b1,
                      "CASE 12 bit18 must select the complete 32-column result window");
      check_condition(uut.last_compute_row_mask == 32'h0010_0402,
                      "CASE 12 row mask must contain rows 1, 10 and 20");
      check_condition(uut.last_compute_col_mask == 32'hFFFF_FFFF,
                      "CASE 12 full-column mask must contain all 32 columns");
      check_condition(compute_fifo_writes == 32,
                      $sformatf("CASE 12 expected 32 result-FIFO writes, actual=%0d",
                                compute_fifo_writes));

      for (column = 0; column < 32; column = column + 1) begin

        wb_read(WB_DATA_ADR, read_data);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("CASE 12 full-column result %0d must be present",
                                  column));

        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == column,
                          $sformatf("CASE 12 result order expected=%0d actual=%0d",
                                    column,
                                    result_column(read_data)));

      end

      expect_empty_read("CASE 12 FIFO must be empty after 32 full-column results");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 13 combines programming and compute traffic without draining the result
  // FIFO between operations. Three SET and three RESET commands each create one
  // automatic readback. A five-packet selected-column compute burst then appends
  // five more results. The eleven final WB reads verify FIFO ordering:
  //   SET readbacks, RESET readbacks, then compute results in ascending column order.
  task automatic testcase_13_program_cells_then_compute_selected_columns;

    integer failure_mark;
    integer read_index;
    integer expected_column;
    integer writes_before_phase;
    integer phase_fifo_writes;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 13 - three SETs, three RESETs, then five selected-column COMPUTEs";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      writes_before_phase = uut.response_write_count_total;

      // Each programming command is allowed to complete its automatic readback
      // before the next command is sent. The readback remains queued in the FIFO.
      $display("CASE 13: SET 1 row=1 column=3 PWM=51");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 1, 3, 8'h51, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 13: SET 2 row=6 column=11 PWM=52");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 6, 11, 8'h52, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 13: SET 3 row=12 column=20 PWM=53");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 12, 20, 8'h53, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 13: RESET 1 row=1 column=3 PWM=61");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 1, 3, 8'h61, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 13: RESET 2 row=6 column=11 PWM=62");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 6, 11, 8'h62, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 13: RESET 3 row=15 column=25 PWM=63");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 15, 25, 8'h63, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      // Five different rows and columns. Bit18 remains zero on every packet,
      // so only the five explicitly selected columns are measured. Bit17 marks
      // packet 5 as the end of the compute burst.
      $display("CASE 13: COMPUTE 1 row=0  col=2  PWM=71 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 0, 2, 8'h71, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 13: COMPUTE 2 row=4  col=5  PWM=72 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 4, 5, 8'h72, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 13: COMPUTE 3 row=8  col=9  PWM=73 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 8, 9, 8'h73, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 13: COMPUTE 4 row=12 col=14 PWM=74 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 12, 14, 8'h74, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 13: COMPUTE 5 row=16 col=23 PWM=75 bit18=0 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 16, 23, 8'h75, 0, 1));

      repeat (500) @(posedge l_usr_clk);

      phase_fifo_writes = uut.response_write_count_total - writes_before_phase;

      check_condition(uut.last_compute_full_column == 1'b0,
                      "CASE 13 must remain a selected-column compute");
      check_condition(uut.last_compute_row_mask == 32'h0001_1111,
                      "CASE 13 compute row mask must contain rows 0, 4, 8, 12 and 16");
      check_condition(uut.last_compute_col_mask == 32'h0080_4224,
                      "CASE 13 compute column mask must contain columns 2, 5, 9, 14 and 23");
      check_condition(phase_fifo_writes == 11,
                      $sformatf("CASE 13 expected 11 result-FIFO writes, actual=%0d",
                                phase_fifo_writes));

      // Expected final FIFO order:
      //   0..2  = three SET readbacks (columns 3, 11, 20)
      //   3..5  = three RESET readbacks (columns 3, 11, 25)
      //   6..10 = five compute results (columns 2, 5, 9, 14, 23)
      for (read_index = 0; read_index < 11; read_index = read_index + 1) begin

        case (read_index)
          0:  expected_column = 3;
          1:  expected_column = 11;
          2:  expected_column = 20;
          3:  expected_column = 3;
          4:  expected_column = 11;
          5:  expected_column = 25;
          6:  expected_column = 2;
          7:  expected_column = 5;
          8:  expected_column = 9;
          9:  expected_column = 14;
          default: expected_column = 23;
        endcase

        wb_read(WB_DATA_ADR, read_data);
        $display("CASE 13: WB READ %0d DATA=%08h expected_column=%0d",
                 read_index + 1,
                 read_data,
                 expected_column);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("CASE 13 WB read %0d must contain data",
                                  read_index + 1));

        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == expected_column,
                          $sformatf("CASE 13 WB read %0d expected column=%0d actual=%0d",
                                    read_index + 1,
                                    expected_column,
                                    result_column(read_data)));

      end

      expect_empty_read("CASE 13 FIFO must be empty after eleven results");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 14 deliberately leaves four programming readbacks queued before a
  // full-column compute. The result FIFO has 32 entries, while the complete
  // sequence produces 4 + 32 = 36 results. The FIFO initially fills with the
  // four SET/RESET readbacks plus compute columns 0..27. FIFO-full then applies
  // backpressure to the TDC/scratchpad output path. As WB reads free entries,
  // compute columns 28..31 resume and are appended; no result is discarded.
  // This case validates bit18 full-column behavior and FIFO backpressure.
  task automatic testcase_14_program_cells_then_compute_full_column;

    integer failure_mark;
    integer read_index;
    integer expected_column;
    integer writes_before_phase;
    integer phase_fifo_writes;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 14 - two SETs, two RESETs, then three-packet full-column COMPUTE";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      writes_before_phase = uut.response_write_count_total;

      $display("CASE 14: SET 1 row=2 column=4 PWM=81");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 2, 4, 8'h81, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 14: SET 2 row=7 column=12 PWM=82");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 7, 12, 8'h82, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 14: RESET 1 row=2 column=4 PWM=91");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 2, 4, 8'h91, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 14: RESET 2 row=10 column=18 PWM=92");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_RESET, 10, 18, 8'h92, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      // The last compute packet sets bit18=1 (all columns) and bit17=1
      // (end of burst). The individual packet column fields are intentionally
      // different but are overridden by the final full-column request.
      $display("CASE 14: COMPUTE 1 row=1  col=3  PWM=A1 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 1, 3, 8'hA1, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 14: COMPUTE 2 row=9  col=13 PWM=A2 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 9, 13, 8'hA2, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 14: COMPUTE 3 row=17 col=27 PWM=A3 bit18=1 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 17, 27, 8'hA3, 1, 1));

      repeat (500) @(posedge l_usr_clk);

      phase_fifo_writes = uut.response_write_count_total - writes_before_phase;

      check_condition(uut.last_compute_full_column == 1'b1,
                      "CASE 14 bit18 must select the complete 32-column result window");
      check_condition(uut.last_compute_row_mask == 32'h0002_0202,
                      "CASE 14 compute row mask must contain rows 1, 9 and 17");
      check_condition(uut.last_compute_col_mask == 32'hFFFF_FFFF,
                      "CASE 14 full-column mask must contain all 32 columns");
      check_condition(phase_fifo_writes == 32,
                      $sformatf("CASE 14 FIFO must contain 32 writes before draining, actual=%0d",
                                phase_fifo_writes));
      check_condition(uut.response_full == 1'b1,
                      "CASE 14 result FIFO must be full before final WB reads");

      // Read all 36 logical results. The first 32 were already stored. The final
      // four compute results are produced as these reads release FIFO backpressure.
      for (read_index = 0; read_index < 36; read_index = read_index + 1) begin

        case (read_index)
          0: expected_column = 4;
          1: expected_column = 12;
          2: expected_column = 4;
          3: expected_column = 18;
          default: expected_column = read_index - 4;
        endcase

        wb_read(WB_DATA_ADR, read_data);
        $display("CASE 14: WB READ %0d DATA=%08h expected_column=%0d",
                 read_index + 1,
                 read_data,
                 expected_column);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("CASE 14 WB read %0d must contain data",
                                  read_index + 1));

        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == expected_column,
                          $sformatf("CASE 14 WB read %0d expected column=%0d actual=%0d",
                                    read_index + 1,
                                    expected_column,
                                    result_column(read_data)));

      end

      phase_fifo_writes = uut.response_write_count_total - writes_before_phase;
      check_condition(phase_fifo_writes == 36,
                      $sformatf("CASE 14 expected 36 total FIFO writes after backpressure release, actual=%0d",
                                phase_fifo_writes));

      expect_empty_read("CASE 14 FIFO must be empty after all 36 mixed-operation results");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 15 drains three automatic SET readbacks before compute traffic, proving
  // that the operation FIFO starts empty. It then performs two independent
  // bit18=1 full-column compute operations. No WB read is issued between them:
  // the first operation fills the 32-entry output FIFO and the second operation
  // is queued/stalled by FIFO backpressure. Final WB draining must release the
  // second operation and return both complete 32-column result sets in order.
  task automatic testcase_15_two_full_column_computes_without_intermediate_read;

    integer failure_mark;
    integer column;
    integer writes_before_set;
    integer writes_before_compute;
    integer compute_fifo_writes;
    string test_name;
    logic [31:0] read_data;

    begin

      test_name = "CASE 15 - drain three SET readbacks, then queue two full-column COMPUTEs";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      //-----------------------------------------------------------------------
      // Phase 1: three SET operations and their automatic readbacks.
      //-----------------------------------------------------------------------
      writes_before_set = uut.response_write_count_total;

      $display("CASE 15 SET PHASE: SET 1 row=1 column=2 PWM=B1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 1, 2, 8'hB1, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 15 SET PHASE: SET 2 row=4 column=7 PWM=B2");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 4, 7, 8'hB2, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      $display("CASE 15 SET PHASE: SET 3 row=9 column=15 PWM=B3");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_SET, 9, 15, 8'hB3, 0, 0));
      repeat (SET_RESET_READBACK_WAIT_CYCLES) @(posedge l_usr_clk);

      check_condition((uut.response_write_count_total - writes_before_set) == 3,
                      $sformatf("CASE 15 expected three SET readbacks, actual=%0d",
                                uut.response_write_count_total - writes_before_set));

      // Consume exactly the three SET results and verify their column tags.
      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "CASE 15 SET readback 1 must contain data");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 2,
                        $sformatf("CASE 15 SET readback 1 expected column=2 actual=%0d",
                                  result_column(read_data)));

      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "CASE 15 SET readback 2 must contain data");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 7,
                        $sformatf("CASE 15 SET readback 2 expected column=7 actual=%0d",
                                  result_column(read_data)));

      wb_read(WB_DATA_ADR, read_data);
      check_condition(read_data !== EMPTY_WORD,
                      "CASE 15 SET readback 3 must contain data");
      if (read_data !== EMPTY_WORD)
        check_condition(result_column(read_data) == 15,
                        $sformatf("CASE 15 SET readback 3 expected column=15 actual=%0d",
                                  result_column(read_data)));

      expect_empty_read("CASE 15 operation FIFO must be empty after three SET readbacks");

      //-----------------------------------------------------------------------
      // Phase 2: first full-column compute. The final packet sets bit18=1 and
      // bit17=1. Do not issue any WB read after this operation.
      //-----------------------------------------------------------------------
      writes_before_compute = uut.response_write_count_total;

      $display("CASE 15 COMPUTE 1: packet 1 row=0 col=3 PWM=C1 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 0, 3, 8'hC1, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 15 COMPUTE 1: packet 2 row=8 col=13 PWM=C2 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 8, 13, 8'hC2, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 15 COMPUTE 1: packet 3 row=16 col=23 PWM=C3 bit18=1 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 16, 23, 8'hC3, 1, 1));

      repeat (500) @(posedge l_usr_clk);

      compute_fifo_writes = uut.response_write_count_total - writes_before_compute;
      check_condition(uut.last_compute_full_column == 1'b1,
                      "CASE 15 first compute must select all 32 columns");
      check_condition(uut.last_compute_row_mask == 32'h0001_0101,
                      "CASE 15 first compute row mask must contain rows 0, 8 and 16");
      check_condition(uut.last_compute_col_mask == 32'hFFFF_FFFF,
                      "CASE 15 first compute column mask must contain all columns");
      check_condition(compute_fifo_writes == 32,
                      $sformatf("CASE 15 first compute expected 32 FIFO writes, actual=%0d",
                                compute_fifo_writes));
      check_condition(uut.response_full == 1'b1,
                      "CASE 15 FIFO must be full after first compute and before second compute");

      //-----------------------------------------------------------------------
      // Phase 3: send a second full-column compute while the first 32 results
      // remain unread. These command packets enter the Wishbone command path,
      // but their output is held until final WB reads release backpressure.
      //-----------------------------------------------------------------------
      $display("CASE 15 COMPUTE 2: packet 1 row=1 col=5 PWM=D1 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 1, 5, 8'hD1, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 15 COMPUTE 2: packet 2 row=9 col=15 PWM=D2 bit18=0 bit17=0");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 9, 15, 8'hD2, 0, 0));
      repeat (2) @(posedge l_usr_clk);

      $display("CASE 15 COMPUTE 2: packet 3 row=17 col=25 PWM=D3 bit18=1 bit17=1");
      wb_write(WB_DATA_ADR,
               command_packet(MODE_COMPUTE, 17, 25, 8'hD3, 1, 1));

      repeat (500) @(posedge l_usr_clk);

      compute_fifo_writes = uut.response_write_count_total - writes_before_compute;
      check_condition(compute_fifo_writes == 32,
                      $sformatf("CASE 15 no-read interval must remain at 32 FIFO writes, actual=%0d",
                                compute_fifo_writes));
      check_condition(uut.response_full == 1'b1,
                      "CASE 15 FIFO must remain full before final draining begins");

      //-----------------------------------------------------------------------
      // Phase 4: drain the first compute result set. Every column must appear
      // once and in ascending order. This simultaneously releases backpressure.
      //-----------------------------------------------------------------------
      for (column = 0; column < 32; column = column + 1) begin

        wb_read(WB_DATA_ADR, read_data);
        $display("CASE 15 COMPUTE 1: WB READ %0d DATA=%08h expected_column=%0d",
                 column + 1,
                 read_data,
                 column);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("CASE 15 first compute result column %0d must be present",
                                  column));
        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == column,
                          $sformatf("CASE 15 first compute expected column=%0d actual=%0d",
                                    column,
                                    result_column(read_data)));

      end

      // Allow the queued second operation to complete and refill the FIFO.
      repeat (500) @(posedge l_usr_clk);

      compute_fifo_writes = uut.response_write_count_total - writes_before_compute;
      check_condition(uut.last_compute_full_column == 1'b1,
                      "CASE 15 second compute must select all 32 columns");
      check_condition(uut.last_compute_row_mask == 32'h0002_0202,
                      "CASE 15 second compute row mask must contain rows 1, 9 and 17");
      check_condition(uut.last_compute_col_mask == 32'hFFFF_FFFF,
                      "CASE 15 second compute column mask must contain all columns");
      check_condition(compute_fifo_writes == 64,
                      $sformatf("CASE 15 expected 64 total compute FIFO writes, actual=%0d",
                                compute_fifo_writes));

      //-----------------------------------------------------------------------
      // Phase 5: drain the second complete 32-column result set.
      //-----------------------------------------------------------------------
      for (column = 0; column < 32; column = column + 1) begin

        wb_read(WB_DATA_ADR, read_data);
        $display("CASE 15 COMPUTE 2: WB READ %0d DATA=%08h expected_column=%0d",
                 column + 1,
                 read_data,
                 column);

        check_condition(read_data !== EMPTY_WORD,
                        $sformatf("CASE 15 second compute result column %0d must be present",
                                  column));
        if (read_data !== EMPTY_WORD)
          check_condition(result_column(read_data) == column,
                          $sformatf("CASE 15 second compute expected column=%0d actual=%0d",
                                    column,
                                    result_column(read_data)));

      end

      expect_empty_read("CASE 15 FIFO must be empty after both full-column results");

      end_test(test_name, failure_mark);

    end

  endtask


  // CASE 16 imports the requested PHASE 3, PHASE 5 and PHASE 6 transactions
  // from topmodule_tb into one focused testcase. The command words, actual
  // wait intervals and Wishbone read counts match that source testbench:
  //   PHASE 3: SET   cell (2,1), wait 800 clocks, perform 3 WB reads
  //   PHASE 5: RESET cell (4,5), wait 800 clocks, perform 3 WB reads
  //   PHASE 6: READ  cell (9,0), wait 800 clocks, perform 2 WB reads
  task automatic testcase_16_topmodule_set_reset_read_sequence;

    integer failure_mark;
    integer read_index;
    integer set_non_empty_count;
    integer reset_non_empty_count;
    integer read_non_empty_count;
    string test_name;
    logic [31:0] read_data;
    logic [31:0] set_packet;
    logic [31:0] reset_packet;
    logic [31:0] read_packet;

    begin

      test_name = "CASE 16 - topmodule_tb PHASE 3 SET, PHASE 5 RESET and PHASE 6 READ";
      begin_test(test_name, failure_mark);

      apply_reset();
      repeat (5) @(posedge l_usr_clk);
      send_three_configuration_packets();

      // Use the local packet builder while checking that it produces
      // the exact command words used by topmodule_tb.
      set_packet   = command_packet(MODE_SET,   2, 1, 8'hFF, 0, 0);
      reset_packet = command_packet(MODE_RESET, 4, 5, 8'hFF, 0, 0);
      read_packet  = command_packet(MODE_READ,  9, 0, 8'hFF, 0, 0);

      check_condition(set_packet === 32'hC410_00FF,
                      $sformatf("CASE 16 SET packet actual=%08h expected=C41000FF",
                                set_packet));
      check_condition(reset_packet === 32'h0850_00FF,
                      $sformatf("CASE 16 RESET packet actual=%08h expected=085000FF",
                                reset_packet));
      check_condition(read_packet === 32'h5200_00FF,
                      $sformatf("CASE 16 READ packet actual=%08h expected=520000FF",
                                read_packet));

      //-----------------------------------------------------------------------
      // Imported PHASE 3: SET cell (2,1), then perform three WB reads.
      //-----------------------------------------------------------------------
      packet_index = 8;
      $display("[%0t] CASE 16 / PHASE 3: SET cell (2,1), DATA=%08h",
               $time,
               set_packet);
      wb_write(WB_DATA_ADR, set_packet);

      $display("[%0t] CASE 16 / PHASE 3: waiting 800 clocks", $time);
      repeat (800) @(posedge l_usr_clk);

      set_non_empty_count = 0;
      for (read_index = 0; read_index < 3; read_index = read_index + 1) begin
        wb_read(WB_DATA_ADR, read_data);
        $display("[%0t] CASE 16 / PHASE 3 WB READ %0d / 3 = %08h",
                 $time,
                 read_index + 1,
                 read_data);

        if (read_data !== EMPTY_WORD) begin
          set_non_empty_count = set_non_empty_count + 1;
          check_condition(result_column(read_data) == 1,
                          $sformatf("CASE 16 SET read %0d expected column=1 actual=%0d",
                                    read_index + 1,
                                    result_column(read_data)));
        end
      end

      check_condition(set_non_empty_count == 1,
                      $sformatf("CASE 16 SET expected one non-empty read, actual=%0d",
                                set_non_empty_count));

      $display("[%0t] CASE 16 / PHASE 3: waiting 100 clocks", $time);
      repeat (100) @(posedge l_usr_clk);

      //-----------------------------------------------------------------------
      // Imported PHASE 5: RESET cell (4,5), then perform three WB reads.
      //-----------------------------------------------------------------------
      packet_index = 11;
      $display("[%0t] CASE 16 / PHASE 5: RESET cell (4,5), DATA=%08h",
               $time,
               reset_packet);
      wb_write(WB_DATA_ADR, reset_packet);

      $display("[%0t] CASE 16 / PHASE 5: waiting 800 clocks", $time);
      repeat (800) @(posedge l_usr_clk);

      reset_non_empty_count = 0;
      for (read_index = 0; read_index < 3; read_index = read_index + 1) begin
        wb_read(WB_DATA_ADR, read_data);
        $display("[%0t] CASE 16 / PHASE 5 WB READ %0d / 3 = %08h",
                 $time,
                 read_index + 1,
                 read_data);

        if (read_data !== EMPTY_WORD) begin
          reset_non_empty_count = reset_non_empty_count + 1;
          check_condition(result_column(read_data) == 5,
                          $sformatf("CASE 16 RESET read %0d expected column=5 actual=%0d",
                                    read_index + 1,
                                    result_column(read_data)));
        end
      end

      check_condition(reset_non_empty_count == 1,
                      $sformatf("CASE 16 RESET expected one non-empty read, actual=%0d",
                                reset_non_empty_count));

      $display("[%0t] CASE 16 / PHASE 5: waiting 100 clocks", $time);
      repeat (100) @(posedge l_usr_clk);

      //-----------------------------------------------------------------------
      // Imported PHASE 6: READ cell (9,0), then perform two WB reads.
      //-----------------------------------------------------------------------
      packet_index = 12;
      $display("[%0t] CASE 16 / PHASE 6: READ cell (9,0), DATA=%08h",
               $time,
               read_packet);
      wb_write(WB_DATA_ADR, read_packet);

      $display("[%0t] CASE 16 / PHASE 6: waiting 800 clocks", $time);
      repeat (800) @(posedge l_usr_clk);

      read_non_empty_count = 0;
      for (read_index = 0; read_index < 2; read_index = read_index + 1) begin
        wb_read(WB_DATA_ADR, read_data);
        $display("[%0t] CASE 16 / PHASE 6 WB READ %0d / 2 = %08h",
                 $time,
                 read_index + 1,
                 read_data);

        if (read_data !== EMPTY_WORD) begin
          read_non_empty_count = read_non_empty_count + 1;
          check_condition(result_column(read_data) == 0,
                          $sformatf("CASE 16 READ result %0d expected column=0 actual=%0d",
                                    read_index + 1,
                                    result_column(read_data)));
        end
      end

      check_condition(read_non_empty_count == 1,
                      $sformatf("CASE 16 READ expected one non-empty read, actual=%0d",
                                read_non_empty_count));

      $display("[%0t] CASE 16 / PHASE 6: waiting 80 clocks", $time);
      repeat (80) @(posedge l_usr_clk);

      end_test(test_name, failure_mark);

    end

  endtask


  //==============================================================================================
  // Packet initialization
  //==============================================================================================

  initial begin

    //--------------------------------------------------------------------------
    // Default all packet memory to zero
    //--------------------------------------------------------------------------

    for (integer i = 0; i <= 300; i = i + 1) begin
      data_packets[i] = 32'h0000_0000;
    end


    //--------------------------------------------------------------------------
    // Configuration packets
    //--------------------------------------------------------------------------

    data_packets[0] = 32'h0003_6472;

    data_packets[1] = 32'h462B_000B;


    //--------------------------------------------------------------------------
    // Configuration / dead count packet
    //--------------------------------------------------------------------------

    data_packets[2] =
        {2'b10,
         10'd62,
         10'd05,
         10'd05};


    //--------------------------------------------------------------------------
    // Cell packets
    //--------------------------------------------------------------------------

    data_packets[3] =
        {2'b01,
         5'b00000,
         5'b00000,
         4'd0,
         16'hAAFF};


    data_packets[4] =
        {2'b11,
         5'b00001,
         5'b00001,
         4'd0,
         16'h673A};


    data_packets[5] =
        {2'b01,
         5'b00010,
         5'b00010,
         4'd0,
         16'h88A2};


    data_packets[6] =
        {2'b10,
         5'b00011,
         5'b00011,
         4'd3,
         16'h8972};


    data_packets[7] =
        {2'b10,
         5'b00101,
         5'b00101,
         4'd3,
         16'h89FA};


    data_packets[8] =
        {2'b10,
         5'b00100,
         5'b00100,
         4'd3,
         16'h8924};


    data_packets[9] =
        {2'b10,
         5'b00110,
         5'b00110,
         4'd3,
         16'h89FF};


    data_packets[10] =
        {2'b10,
         5'b00111,
         5'b00111,
         4'd3,
         16'h89BC};


    data_packets[11] =
        {2'b10,
         5'b01000,
         5'b01000,
         4'd3,
         16'h8989};


    data_packets[12] =
        {2'b10,
         5'b01001,
         5'b01001,
         4'd3,
         16'h0006};


    data_packets[13] =
        {2'b10,
         5'b01010,
         5'b01010,
         4'd3,
         16'h0006};


    data_packets[14] =
        {2'b10,
         5'b01011,
         5'b01011,
         4'd3,
         16'h0007};


    data_packets[15] =
        {2'b10,
         5'b01100,
         5'b01100,
         4'd3,
         16'h0007};


    data_packets[16] =
        {2'b10,
         5'b01101,
         5'b01101,
         4'd3,
         16'h0008};


    data_packets[17] =
        {2'b10,
         5'b01110,
         5'b01110,
         4'd3,
         16'h0008};


    data_packets[18] =
        {2'b10,
         5'b01111,
         5'b01111,
         4'd3,
         16'h0009};


    data_packets[19] =
        {2'b10,
         5'b10000,
         5'b10000,
         4'd3,
         16'h0009};


    data_packets[20] =
        {2'b10,
         5'b10001,
         5'b10001,
         4'd3,
         16'h0010};


    data_packets[21] =
        {2'b10,
         5'b10010,
         5'b10010,
         4'd3,
         16'h0010};


    data_packets[22] =
        {2'b10,
         5'b10011,
         5'b10011,
         4'd3,
         16'h0011};


    data_packets[23] =
        {2'b10,
         5'b10100,
         5'b10100,
         4'd3,
         16'h0011};


    data_packets[24] =
        {2'b10,
         5'b10101,
         5'b10101,
         4'd3,
         16'h0012};


    data_packets[25] =
        {2'b10,
         5'b10110,
         5'b10110,
         4'd3,
         16'h0012};


    data_packets[26] =
        {2'b10,
         5'b10111,
         5'b10111,
         4'd3,
         16'h0013};


    data_packets[27] =
        {2'b10,
         5'b11000,
         5'b11000,
         4'd3,
         16'h0013};


    data_packets[28] =
        {2'b10,
         5'b11001,
         5'b11001,
         4'd3,
         16'h001D};


    data_packets[29] =
        {2'b10,
         5'b11010,
         5'b11010,
         4'd3,
         16'h0015};


    data_packets[30] =
        {2'b10,
         5'b11011,
         5'b11011,
         4'd3,
         16'h0FE2};


    data_packets[31] =
        {2'b10,
         5'b11100,
         5'b11100,
         4'd3,
         16'hAA2D};


    data_packets[32] =
        {2'b10,
         5'b11101,
         5'b11101,
         4'd1,
         16'hAAFE};


    data_packets[33] =
        {2'b10,
         5'b11110,
         5'b11110,
         4'd2,
         16'h1AFA};


    data_packets[34] =
        {2'b10,
         5'b11111,
         5'b11111,
         4'd3,
         16'hA0FD};


    data_packets[35] =
        {2'b10,
         5'b00000,
         5'b01010,
         4'd4,
         16'h1AF0};


    data_packets[36] =
        {2'b10,
         5'b00000,
         5'b01010,
         4'd5,
         16'h110F};


    $display("[%0t] Packet memory initialized", $time);

  end


  //==============================================================================================
  // Main RISC-V / Wishbone master sequence
  //==============================================================================================

  // Retained from the supplied file for reference. It is disabled so only the sixteen
  // focused testcases below drive the Wishbone master.
  initial begin : LEGACY_PACKET_SEQUENCE_DISABLED
  
    integer i;
    logic [31:0] read_data;

    if (1'b0) begin : DISABLED_BODY


    //--------------------------------------------------------------------------
    // Initial values
    //--------------------------------------------------------------------------

    TM       = 1'b0;
    scan_se1 = 1'b0;
    scan_si1 = 1'b0;

    stop_bit = 32'h0000_0000;

    packet_index = 0;

    wb_idle();


    //--------------------------------------------------------------------------
    // Apply reset
    //--------------------------------------------------------------------------

    apply_reset();


    //--------------------------------------------------------------------------
    // Allow DUT to settle
    //--------------------------------------------------------------------------

    repeat (5) @(posedge l_usr_clk);


    //--------------------------------------------------------------------------
    // Initial register access
    //
    // This corresponds to:
    //
    //      Address = 3000_0004
    //      Write   = 1
    //      Data    = 0003_6472
    //
    //--------------------------------------------------------------------------

    $display("");
    $display("============================================================");
    $display("STARTING RISC-V / WISHBONE PACKET TRANSFER");
    $display("============================================================");

 //------------------------------------------------------------------------------------------
    // Send packets
    //------------------------------------------------------------------------------------------

    for (i = 0; i < total_packets; i = i + 1) begin

        packet_index = i;

        $display("");
        $display("============================================================");
        $display("[%0t] PACKET %0d / %0d",
                 $time,
                 i,
                 total_packets-1);

        $display("ADDR = %08h", WB_DATA_ADR);
        $display("DATA = %08h", data_packets[i]);

        $display("============================================================");


        //--------------------------------------------------------------------------------------
        // Perform Wishbone write
        //--------------------------------------------------------------------------------------

        wb_write(
            WB_DATA_ADR,
            data_packets[i]
        );


        //--------------------------------------------------------------------------------------
        // Gap AFTER the write has completed
        //
        // First four packets:
        //
        //   packet 0 -> packet 1 : 2 clocks
        //   packet 1 -> packet 2 : 2 clocks
        //   packet 2 -> packet 3 : 2 clocks
        //
        // From packet 4 onward:
        //
        //   500 clocks
        //--------------------------------------------------------------------------------------

        if (i < 3) begin

            $display("[%0t] Waiting 2 clocks before next packet",
                     $time);

            repeat (2) @(posedge l_usr_clk);

        end

        else if (i < total_packets-1) begin

            $display("[%0t] Waiting 500 clocks before next packet",
                     $time);

            repeat (200) @(posedge l_usr_clk);

        end

    end

/*
    //--------------------------------------------------------------------------
    // Send packets
    //--------------------------------------------------------------------------

    for (packet_index = 0;
         packet_index < total_packets;
         packet_index = packet_index + 1) begin


      $display("");
      $display("============================================================");
      $display("PACKET %0d / %0d", packet_index + 1, total_packets);
      $display("DATA = %08h", data_packets[packet_index]);
      $display("============================================================");


      wb_write(
          WB_DATA_ADR,
          data_packets[packet_index]
      );


    end
*/
    //--------------------------------------------------------------------------
    // All packets sent
    //--------------------------------------------------------------------------

    $display("");
    $display("============================================================");
    $display("[%0t] ALL %0d PACKETS SENT SUCCESSFULLY",
             $time,
             total_packets);
    $display("============================================================");


    //--------------------------------------------------------------------------
    // Optional readback
    //
    // Only keep this if 3000_0004 is actually readable.
    //--------------------------------------------------------------------------
for (int m = 0 ;m<32 ; m=m+1) begin 
    
    wb_read(
        WB_DATA_ADR,
        read_data
    );

    $display("[%0t] READBACK = %08h",
             $time,
             read_data);
    


    //--------------------------------------------------------------------------
    // Allow DUT to finish internal operation
    //--------------------------------------------------------------------------
end
     //repeat (20) @(posedge l_usr_clk);


    $display("");
    $display("============================================================");
    $display("[%0t] TEST COMPLETE",
             $time);
    $display("============================================================");


    $finish;

    end

  end


  //==============================================================================================
  // Main focused testcase sequence
  //==============================================================================================

  initial begin : MAIN_TEST

    //--------------------------------------------------------------------------
    // Initial values - DUT wiring, clock, reset task and Wishbone tasks above are
    // exactly those from the supplied reference testbench.
    //--------------------------------------------------------------------------

    TM       = 1'b0;
    scan_se1 = 1'b0;
    scan_si1 = 1'b0;
    stop_bit = 32'h0000_0000;
    reset    = 1'b0;

    packet_index = 0;
    wb_idle();

    total_tests   = 0;
    passed_tests  = 0;
    failed_tests  = 0;
    failed_checks = 0;

    // Command-line +CASE=<0..16> overrides the selected_case value declared near the top.
    if ($value$plusargs("CASE=%d", selected_case)) begin
      $display("Selected testcase from plusarg: CASE=%0d", selected_case);
    end

    $display("");
    $display("================================================================================");
    $display("FOCUSED RTL TESTS USING SUPPLIED WISHBONE MASTER HANDSHAKE");
    $display("selected_case=%0d (0 means run all sixteen)", selected_case);
    $display("================================================================================");

    if ((selected_case == 0) || (selected_case == 1))
      testcase_1_three_configuration_packets();

    if ((selected_case == 0) || (selected_case == 2))
      testcase_2_config_then_empty_read();

    if ((selected_case == 0) || (selected_case == 3))
      testcase_3_reset_then_empty_read();

    if ((selected_case == 0) || (selected_case == 4))
      testcase_4_set_read_reset_read();

    if ((selected_case == 0) || (selected_case == 5))
      testcase_5_six_compute_sixth_bit18();

    if ((selected_case == 0) || (selected_case == 6))
      testcase_6_two_compute_bit18_bit17_full_column();

    if ((selected_case == 0) || (selected_case == 7))
      testcase_7_set_read_reset_read_then_empty();

    if ((selected_case == 0) || (selected_case == 8))
      testcase_8_three_reads_after_each_program_phase();

    if ((selected_case == 0) || (selected_case == 9))
      testcase_9_isolate_set_reset_automatic_readback();

    if ((selected_case == 0) || (selected_case == 10))
      testcase_10_compute_six_rows_same_column();

    if ((selected_case == 0) || (selected_case == 11))
      testcase_11_compute_four_rows_four_columns();

    if ((selected_case == 0) || (selected_case == 12))
      testcase_12_compute_three_packets_full_column();

    if ((selected_case == 0) || (selected_case == 13))
      testcase_13_program_cells_then_compute_selected_columns();

    if ((selected_case == 0) || (selected_case == 14))
      testcase_14_program_cells_then_compute_full_column();

    if ((selected_case == 0) || (selected_case == 15))
      testcase_15_two_full_column_computes_without_intermediate_read();

    if ((selected_case == 0) || (selected_case == 16))
      testcase_16_topmodule_set_reset_read_sequence();

    $display("");
    $display("================================================================================");
    $display("FINAL SUMMARY: TOTAL=%0d PASS=%0d FAIL=%0d FAILED_CHECKS=%0d",
             total_tests,
             passed_tests,
             failed_tests,
             failed_checks);
    $display("================================================================================");

    if (total_tests == 0)
      $fatal(1, "Invalid selected_case=%0d. Use 0 through 16.", selected_case);
    else if ((failed_tests != 0) || (failed_checks != 0))
      $fatal(1, "One or more focused RTL tests failed");
    else
      $finish;

  end


  //==============================================================================================
  // Optional Wishbone monitor
  //
  // This is very useful when comparing against your RISC-V waveform.
  //==============================================================================================

  always @(posedge l_usr_clk) begin

    if (l_wb_cyc && l_wb_stb) begin

      $display("[%0t] WB BUS: CYC=%b STB=%b WE=%b ADR=%08h DAT=%08h SEL=%h ACK=%b",
               $time,
               l_wb_cyc,
               l_wb_stb,
               l_wb_we,
               l_wb_addr,
               li_wb_data,
               l_wbs_sel,
               l_wb_ack);

    end

  end


  //==============================================================================================
  // Scan shift task
  //==============================================================================================

  task automatic scan_shift_in;

    input [15:0] bits;

    integer i;

    begin

      if (TM == 1'b1) begin

        @(negedge l_usr_clk);

        scan_se1 <= 1'b0;

        for (i = 0; i < 16; i = i + 1) begin

          @(negedge l_usr_clk);

          scan_si1 <= bits[i];

          @(posedge l_usr_clk);

        end

      end

    end

  endtask


endmodule

