`timescale 1ns / 1ps
`include "uvm_macros.svh"
import uvm_pkg::*;

//////////////////////////////////////////////////
// I2C Interface
//////////////////////////////////////////////////
interface i2c_intf (
    input logic iClk,
    input logic iRst
);

    // Master Control Signals
    logic       iI2C_Start;
    logic       iI2C_Stop;
    logic       iI2C_Write;
    logic       iI2C_Read;
    logic [7:0] iTx_Data;

    // Master Status & Data
    logic       oTx_Done;
    logic       oTx_Ready;
    logic [7:0] oRx_Data;
    logic       oRx_Done;

    // I2C Signals
    tri         SCL;
    tri         SDA;

    // tr -> MON -> SCB Check
    logic [7:0] reg0;
    logic [7:0] reg1;
    logic [7:0] reg2;
    logic [7:0] reg3;

endinterface


//////////////////////////////////////////////////
// Transaction
//////////////////////////////////////////////////
class i2c_trans extends uvm_sequence_item;

    // UVM Driver Variables
    rand bit [1:0] addr;  // register_pointer
    rand bit       rw;  // 0: write, 1: read
    rand bit [7:0] wdata;
    // UVM Monitor Variables
    rand bit [7:0] rdata;

    function new(string name = "i2c_trans");
        super.new(name);
    endfunction

    `uvm_object_utils_begin(i2c_trans)
        `uvm_field_int(addr, UVM_ALL_ON)
        `uvm_field_int(rw, UVM_ALL_ON)
        `uvm_field_int(wdata, UVM_ALL_ON)
        `uvm_field_int(rdata, UVM_DEFAULT)
    `uvm_object_utils_end

endclass


//////////////////////////////////////////////////
// Sequence
//////////////////////////////////////////////////
class i2c_sequence extends uvm_sequence #(i2c_trans);

    `uvm_object_utils(i2c_sequence)

    function new(string name = "i2c_sequence");
        super.new(name);
    endfunction

    virtual task body();
        i2c_trans tr;
        bit [7:0] test_data[4];

        foreach (test_data[i]) test_data[i] = $urandom();

        `uvm_info("SEQ", "--- REG WRITE/READ TEST START ---", UVM_LOW)

        for (int i = 0; i < 4; i++) begin
            `uvm_info("SEQ", $sformatf("Test Register %0d", i), UVM_LOW)

            // WRITE Operation
            tr = i2c_trans::type_id::create("write.tr");

            start_item(tr);
            if (!tr.randomize() with {
                    rw == 0;  // Write
                    addr == i;  // reg[i]
                    wdata == test_data[i];  // readom data
                })
                `uvm_error("SEQ", "Write TR Randomize Fail")
            finish_item(tr);

            // READ Operation
            tr = i2c_trans::type_id::create("read.tr");

            start_item(tr);
            if (!tr.randomize() with {
                    rw == 1;  // Read
                    addr == i;  // reg[i]
                })
                `uvm_error("SEQ", "Read TR Randomize Fail")
            finish_item(tr);
        end
        `uvm_info("SEQ", "--- REG WRITE/READ TEST END ---", UVM_LOW)
    endtask

endclass


//////////////////////////////////////////////////
// Sequencer
//////////////////////////////////////////////////
class i2c_sequencer extends uvm_sequencer #(i2c_trans);

    `uvm_component_utils(i2c_sequencer)

    function new(string name = "i2c_sequencer", uvm_component parent = null);
        super.new(name, parent);
    endfunction

endclass


//////////////////////////////////////////////////
// Driver
//////////////////////////////////////////////////
class i2c_driver extends uvm_driver #(i2c_trans);

    `uvm_component_utils(i2c_driver)
    virtual i2c_intf i2c_if;
    localparam bit [6:0] SLAVE_ADDR = 7'h54;

    function new(string name = "i2c_driver", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("DRV", "build_phase start", UVM_LOW)
        if (!uvm_config_db#(virtual i2c_intf)::get(this, "", "i2c_if", i2c_if))
            `uvm_fatal("DRV", "i2c_if not found");
        `uvm_info("DRV", "build_phase end : i2c_if connected", UVM_LOW)
    endfunction

    //////////////////////////////
    // Control Task
    //////////////////////////////
    // START + ADDR
    task start_addr(input [7:0] addr_byte);
        @(posedge i2c_if.iClk);
        i2c_if.iTx_Data   = addr_byte;
        i2c_if.iI2C_Start = 1;

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Start = 0;

        wait (i2c_if.oTx_Done);
        @(posedge i2c_if.iClk);
    endtask

    // REG_ADDR WRITE
    task write_reg_addr(input [7:0] addr);
        wait (i2c_if.oTx_Ready);

        @(posedge i2c_if.iClk);
        i2c_if.iTx_Data   = addr;
        i2c_if.iI2C_Write = 1;

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Write = 0;

        wait (i2c_if.oTx_Done);
        @(posedge i2c_if.iClk);
    endtask

    // WRITE 1 BYTE
    task write_byte(input [7:0] data);
        wait (i2c_if.oTx_Ready);

        @(posedge i2c_if.iClk);
        i2c_if.iTx_Data   = data;
        i2c_if.iI2C_Write = 1;
        `uvm_info("DRV", $sformatf("Sending data = 0x%02h", data), UVM_LOW)

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Write = 0;

        wait (i2c_if.oTx_Done);
        @(posedge i2c_if.iClk);
    endtask

    // READ 1 BYTE
    task read_byte(output [7:0] data);
        wait (i2c_if.oTx_Ready);

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Read = 1'b1;
        i2c_if.iI2C_Stop = 1'b1;  // NACK + STOP

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Read = 1'b0;
        i2c_if.iI2C_Stop = 1'b0;

        wait (i2c_if.oRx_Done);

        @(posedge i2c_if.iClk);
        data = i2c_if.oRx_Data;
    endtask

    // RESTART
    task restart_addr(input [7:0] addr_byte);
        wait (i2c_if.oTx_Ready);

        @(posedge i2c_if.iClk);
        i2c_if.iTx_Data   = addr_byte;
        i2c_if.iI2C_Start = 1'b1;

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Start = 1'b0;

        wait (i2c_if.oTx_Done);
        @(posedge i2c_if.iClk);
    endtask

    // STOP
    task send_stop();
        wait (i2c_if.oTx_Ready);

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Stop = 1;

        @(posedge i2c_if.iClk);
        i2c_if.iI2C_Stop = 0;

        @(posedge i2c_if.iClk);
    endtask

    // Stop Detect
    task wait_i2c_stop();
        bit prev_sda;
        prev_sda = i2c_if.SDA;

        forever begin
            @(posedge i2c_if.iClk);
            if (prev_sda == 0 && i2c_if.SDA == 1 && i2c_if.SCL == 1) return;
            prev_sda = i2c_if.SDA;
        end
    endtask


    //////////////////////////////
    // Write / Read Op
    //////////////////////////////
    // Write
    task drive_write_op(i2c_trans tr);
        byte addr_w = {SLAVE_ADDR, 1'b0};

        `uvm_info("DRV", $sformatf("DRV_WRITE : reg[%0d] <= 0x%02h", tr.addr,
                                   tr.wdata), UVM_LOW)

        start_addr(addr_w);  // Start + Addr
        write_reg_addr({6'b0, tr.addr});  // Reg_Addr
        write_byte(tr.wdata);  // Write
        send_stop();  // Stop

        wait_i2c_stop();
    endtask

    // Read
    task drive_read_op(i2c_trans tr);
        byte addr_w = {SLAVE_ADDR, 1'b0};
        byte addr_r = {SLAVE_ADDR, 1'b1};
        byte rd_data;
        `uvm_info("DRV", $sformatf("DRV_READ : reg[%0d]", tr.addr), UVM_LOW)

        // Pointer Set
        start_addr(addr_w);  // Start + Addr
        write_reg_addr({6'b0, tr.addr});  // Reg_Addr

        // Restart + Read
        restart_addr(addr_r);  // Start + Addr
        read_byte(rd_data);  // Read

        tr.rdata = rd_data;
        wait_i2c_stop();
    endtask

    virtual task run_phase(uvm_phase phase);
        @(negedge i2c_if.iRst);

        // Initialize
        i2c_if.iI2C_Start = 0;
        i2c_if.iI2C_Stop  = 0;
        i2c_if.iI2C_Write = 0;
        i2c_if.iI2C_Read  = 0;
        i2c_if.iTx_Data   = 0;

        forever begin
            i2c_trans tr;
            `uvm_info("DRV", "wait for next item", UVM_LOW)

            seq_item_port.get_next_item(tr);

            if (!tr.rw) begin  // Write
                drive_write_op(tr);
            end else begin
                drive_read_op(tr);
            end

            seq_item_port.item_done();

            wait (i2c_if.oTx_Ready);
            @(posedge i2c_if.iClk);
        end
    endtask

endclass


//////////////////////////////////////////////////
// Monitor (FINAL VERSION)
//////////////////////////////////////////////////
class i2c_monitor extends uvm_component;
    `uvm_component_utils(i2c_monitor)

    uvm_analysis_port #(i2c_trans) mon_ap;
    virtual i2c_intf i2c_if;

    localparam bit [6:0] SLAVE_ADDR = 7'h54;

    function new(string name = "i2c_monitor", uvm_component parent = null);
        super.new(name, parent);
        mon_ap = new("mon_ap", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual i2c_intf)::get(this, "", "i2c_if", i2c_if))
            `uvm_fatal("MON", "i2c_if not found for monitor")
    endfunction


    ///////////////////////////////////////////////////////
    // 1. START / STOP Detector (SCL 기반)
    ///////////////////////////////////////////////////////
    task wait_for_start();
        forever begin
            @(negedge i2c_if.SDA);  // SDA 1→0
            if (i2c_if.SCL == 1) begin
                `uvm_info("MON", "I2C START DETECT", UVM_LOW)
                return;
            end
        end
    endtask

    task wait_for_stop();
        forever begin
            @(posedge i2c_if.SDA);  // SDA 0→1
            if (i2c_if.SCL == 1) begin
                `uvm_info("MON", "I2C STOP DETECT", UVM_LOW)
                return;
            end
        end
    endtask


    ///////////////////////////////////////////////////////
    // 2. SCL 기반 바이트 수집
    ///////////////////////////////////////////////////////
    task read_byte(output bit [7:0] data, output bit ack);
        // 8bit data
        for (int i = 7; i >= 0; i--) begin
            @(posedge i2c_if.SCL);
            data[i] = i2c_if.SDA;
            @(negedge i2c_if.SCL);
        end

        // ACK bit (0=ACK, 1=NACK)
        @(posedge i2c_if.SCL);
        ack = i2c_if.SDA;
        @(negedge i2c_if.SCL);
    endtask


    ///////////////////////////////////////////////////////
    // 3. WRITE Transaction Recognizer
    ///////////////////////////////////////////////////////
    task monitor_write_bytes(ref i2c_trans tr);
        bit [7:0] data_byte;
        bit ack;

        wait_for_start();

        // 1) SLAVE + W
        read_byte(data_byte, ack);
        if ((data_byte[7:1] != SLAVE_ADDR) || (ack) || (data_byte[0] != 1'b0)) begin
            wait_for_stop();
            return;
        end

        // 2) REG_ADDR
        read_byte(data_byte, ack);
        if (ack) begin
            wait_for_stop();
            return;
        end

        tr.rw   = 1'b0;
        tr.addr = data_byte[1:0];

        // 3) DATA
        read_byte(data_byte, ack);
        tr.wdata = data_byte;

        wait_for_stop();
    endtask


    ///////////////////////////////////////////////////////
    // 4. READ Transaction Recognizer
    ///////////////////////////////////////////////////////
    task monitor_read_bytes(ref i2c_trans tr);
        bit [7:0] data_byte;
        bit ack;

        wait_for_start();

        // 1) SLAVE + W (pointer)
        read_byte(data_byte, ack);
        if ((data_byte[7:1] != SLAVE_ADDR) || (ack) || (data_byte[0] != 1'b0)) begin
            wait_for_stop();
            return;
        end

        // 2) REG_ADDR
        read_byte(data_byte, ack);
        if (ack) begin
            wait_for_stop();
            return;
        end

        tr.addr = data_byte[1:0];
        tr.rw   = 1'b1;

        // 3) RESTART
        wait_for_start();

        // 4) SLAVE + R
        read_byte(data_byte, ack);
        if ((data_byte[7:1] != SLAVE_ADDR) || (ack) || (data_byte[0] != 1'b1)) begin
            wait_for_stop();
            return;
        end

        // 5) READ DATA
        read_byte(data_byte, ack);
        tr.rdata = data_byte;

        wait_for_stop();
    endtask


    ///////////////////////////////////////////////////////
    // 5. Run Phase
    ///////////////////////////////////////////////////////
    virtual task run_phase(uvm_phase phase);
        @(negedge i2c_if.iRst);

        forever begin
            i2c_trans wr = i2c_trans::type_id::create("wr", this);
            i2c_trans rd = i2c_trans::type_id::create("rd", this);

            monitor_write_bytes(wr);
            mon_ap.write(wr);

            monitor_read_bytes(rd);
            mon_ap.write(rd);
        end
    endtask

endclass



//////////////////////////////////////////////////
// Scoreboard
//////////////////////////////////////////////////
class i2c_scoreboard extends uvm_component;
    `uvm_component_utils(i2c_scoreboard)

    uvm_analysis_imp #(i2c_trans, i2c_scoreboard) sb_imp;
    virtual i2c_intf i2c_if;

    bit [7:0] model_reg[4];

    int total_tr = 0;
    int write_count = 0;
    int read_count = 0;

    int write_pass = 0;
    int write_fail = 0;
    int read_pass = 0;
    int read_fail = 0;

    int expected_regs = 4;

    function new(string name = "i2c_scoreboard", uvm_component parent = null);
        super.new(name, parent);
        sb_imp = new("sb_imp", this);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        if (!uvm_config_db#(virtual i2c_intf)::get(this, "", "i2c_if", i2c_if))
            `uvm_fatal("SCB", "i2c_if not found for scoreboard")

        for (int i = 0; i < 4; i++) model_reg[i] = 8'b0;
    endfunction

    function void write(i2c_trans tr);
        bit [7:0] dut_reg_val;

        total_tr++;

        if (tr.rw == 1'b0) begin
            // ===== WRITE TRANSACTION =====
            write_count++;

            // Update golden model
            model_reg[tr.addr] = tr.wdata;
            `uvm_info("SCB", $sformatf("MODEL UPDATE : reg[%0d] <= 0x%02h",
                                       tr.addr, tr.wdata), UVM_LOW)

            // Get DUT register value
            case (tr.addr)
                2'd0:    dut_reg_val = i2c_if.reg0;
                2'd1:    dut_reg_val = i2c_if.reg1;
                2'd2:    dut_reg_val = i2c_if.reg2;
                2'd3:    dut_reg_val = i2c_if.reg3;
                default: dut_reg_val = 8'hxx;
            endcase

            // Compare
            if (model_reg[tr.addr] === dut_reg_val) begin
                write_pass++;
                `uvm_info(
                    "SCB",
                    $sformatf(
                        "✓ WRITE PASS: reg[%0d] Model=0x%02h, DUT=0x%02h",
                        tr.addr, model_reg[tr.addr], dut_reg_val), UVM_LOW)
            end else begin
                write_fail++;
                `uvm_error("SCB", $sformatf(
                           "✗ WRITE FAIL: reg[%0d] Model=0x%02h, DUT=0x%02h",
                           tr.addr,
                           model_reg[tr.addr],
                           dut_reg_val
                           ))
            end

        end else begin
            // ===== READ TRANSACTION =====
            read_count++;

            // Compare read data with golden model
            if (tr.rdata === model_reg[tr.addr]) begin
                read_pass++;
                `uvm_info(
                    "SCB",
                    $sformatf(
                        "✓ READ PASS: reg[%0d] Expected=0x%02h, Read=0x%02h",
                        tr.addr, model_reg[tr.addr], tr.rdata), UVM_LOW)
            end else begin
                read_fail++;
                `uvm_error("SCB", $sformatf(
                           "✗ READ FAIL: reg[%0d] Expected=0x%02h, Read=0x%02h",
                           tr.addr,
                           model_reg[tr.addr],
                           tr.rdata
                           ))
            end
        end
    endfunction


    function bit is_done();
        return (write_count == expected_regs) && (read_count == expected_regs);
    endfunction

    function void report_phase(uvm_phase phase);
        super.report_phase(phase);

        `uvm_info("SCB_REPORT", "========================================",
                  UVM_LOW)
        `uvm_info("SCB_REPORT", "        SCOREBOARD REPORT                 ",
                  UVM_LOW)
        `uvm_info("SCB_REPORT", "========================================",
                  UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf("Total Transactions  : %0d", total_tr
                  ), UVM_LOW)
        `uvm_info("SCB_REPORT", "----------------------------------------",
                  UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf(
                  "WRITE Transactions  : %0d", write_count), UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf(
                  "  -> WRITE Passed   : %0d", write_pass), UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf(
                  "  -> WRITE Failed   : %0d", write_fail), UVM_LOW)
        `uvm_info("SCB_REPORT", "----------------------------------------",
                  UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf(
                  "READ Transactions   : %0d", read_count), UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf("  -> READ Passed    : %0d", read_pass
                  ), UVM_LOW)
        `uvm_info("SCB_REPORT", $sformatf("  -> READ Failed    : %0d", read_fail
                  ), UVM_LOW)
        `uvm_info("SCB_REPORT", "========================================",
                  UVM_LOW)

        if (write_fail == 0 && read_fail == 0) begin
            `uvm_info("SCB_REPORT", "*********** ALL TESTS PASSED ***********",
                      UVM_LOW)
        end else begin
            `uvm_error("SCB_REPORT", "********** SOME TESTS FAILED **********")
        end
        `uvm_info("SCB_REPORT", "========================================",
                  UVM_LOW)
    endfunction

endclass


//////////////////////////////////////////////////
// Agent
//////////////////////////////////////////////////
class i2c_agent extends uvm_component;
    `uvm_component_utils(i2c_agent)

    i2c_sequencer seqr;
    i2c_driver drv;
    i2c_monitor mon;

    function new(string name = "i2c_agent", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("AGT", "build_phase start", UVM_LOW)
        seqr = i2c_sequencer::type_id::create("SEQR", this);
        drv  = i2c_driver::type_id::create("DRV", this);
        mon  = i2c_monitor::type_id::create("MON", this);
        `uvm_info("AGT", "build_phase end", UVM_LOW)
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("AGT", "connect_phase: DRV <-> SEQR", UVM_LOW)
        drv.seq_item_port.connect(seqr.seq_item_export);
    endfunction

endclass


//////////////////////////////////////////////////
// Environment
//////////////////////////////////////////////////
class i2c_env extends uvm_env;
    `uvm_component_utils(i2c_env)

    i2c_agent agt;
    i2c_scoreboard scb;

    function new(string name = "i2c_env", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("ENV", "build_phase start", UVM_LOW)
        agt = i2c_agent::type_id::create("AGT", this);
        scb = i2c_scoreboard::type_id::create("SCB", this);
        `uvm_info("ENV", "build_phase end", UVM_LOW)
    endfunction

    function void connect_phase(uvm_phase phase);
        super.connect_phase(phase);
        `uvm_info("ENV", "connect_phase: AGT.MON -> SCB", UVM_LOW)
        agt.mon.mon_ap.connect(scb.sb_imp);
    endfunction

endclass


//////////////////////////////////////////////////
// Test
//////////////////////////////////////////////////
class i2c_test extends uvm_test;
    `uvm_component_utils(i2c_test)

    i2c_env e;
    i2c_sequence seq;

    function new(string name = "i2c_test", uvm_component parent = null);
        super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
        super.build_phase(phase);
        `uvm_info("TEST", "build_phase start", UVM_LOW)
        e   = i2c_env::type_id::create("ENV", this);
        seq = i2c_sequence::type_id::create("SEQ");
        `uvm_info("TEST", "build_phase end", UVM_LOW)
    endfunction

    virtual task run_phase(uvm_phase phase);
        `uvm_info("TEST", "run_phase start : raise_objection", UVM_LOW)
        phase.raise_objection(this);

        `uvm_info("TEST", "start sequence", UVM_LOW)
        seq.start(e.agt.seqr);
        `uvm_info("TEST", "sequence done", UVM_LOW)

        wait (e.scb.is_done());
        `uvm_info("TEST", "run_phase drop_objection", UVM_LOW)
        phase.drop_objection(this);
        `uvm_info("TEST", "run_phase end", UVM_LOW)
    endtask

endclass


//////////////////////////////////////////////////
// TB TOP
//////////////////////////////////////////////////
module tb_i2c_uvm;
    logic iClk;
    logic iRst;

    i2c_intf i2c_if (
        .iClk(iClk),
        .iRst(iRst)
    );

    logic [7:0] reg0, reg1, reg2, reg3;

    I2C_Master #(
        .CLK_FREQ(100_000_000),
        .I2C_FREQ(100_000)
    ) U_MASTER (
        .iClk      (iClk),
        .iRst      (iRst),
        .iI2C_Start(i2c_if.iI2C_Start),
        .iI2C_Stop (i2c_if.iI2C_Stop),
        .iI2C_Write(i2c_if.iI2C_Write),
        .iI2C_Read (i2c_if.iI2C_Read),
        .iTx_Data  (i2c_if.iTx_Data),
        .oTx_Done  (i2c_if.oTx_Done),
        .oTx_Ready (i2c_if.oTx_Ready),
        .oRx_Data  (i2c_if.oRx_Data),
        .oRx_Done  (i2c_if.oRx_Done),
        .ioSCL     (i2c_if.SCL),
        .ioSDA     (i2c_if.SDA),
        .oLed      ()
    );

    I2C_Slave #(
        .SLAVE_ADDR(7'h54)
    ) U_SLAVE (
        .iClk (iClk),
        .iRst (iRst),
        .ioSCL(i2c_if.SCL),
        .ioSDA(i2c_if.SDA),
        .oReg0(reg0),
        .oReg1(reg1),
        .oReg2(reg2),
        .oReg3(reg3)
    );

    pullup (i2c_if.SCL);
    pullup (i2c_if.SDA);

    assign i2c_if.reg0 = reg0;
    assign i2c_if.reg1 = reg1;
    assign i2c_if.reg2 = reg2;
    assign i2c_if.reg3 = reg3;

    initial iClk = 0;
    always #5 iClk = ~iClk;

    initial begin
        iRst              = 1'b1;
        i2c_if.iI2C_Start = 0;
        i2c_if.iI2C_Stop  = 0;
        i2c_if.iI2C_Write = 0;
        i2c_if.iI2C_Read  = 0;
        i2c_if.iTx_Data   = 8'h00;

        #100 iRst = 1'b0;
    end

    initial begin
        $fsdbDumpvars(0);
        $fsdbDumpfile("wave.fsdb");

        uvm_config_db#(virtual i2c_intf)::set(null, "*", "i2c_if", i2c_if);
        run_test("i2c_test");
    end

endmodule
