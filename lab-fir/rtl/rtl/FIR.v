// ------------------------------------------------------------------------
// The MIT License (MIT)

// Copyright (c) <year> Adam Veldhousen

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
// ------------------------------------------------------------------------
// ========================================================================
// Module Name: fir
// Author: Brian SU
// Create Date: 2025-03-06
// Features & Functions:
// . Bullet points
// .
// ========================================================================
// Revision History:
// Date      by     Version  Change Description
// 3/6/2025  Brian  1.0      Initial release
// 3/12/2025 Brian  2.0      Modify FSM & structure
// 3/14/2025 Brian  2.1      Use 3 FSM to control Engine, Memory, and data path individually. Also attempt to 
//                           reach 'Tap Cycles' to complete a single y[n] with a constructed FSM controlling the address of tap and data, DataRAM
// ------------------------------------------------------------------------
// 若Tap不足32，data RAM剩餘的空間可用來存下次FIR的資料
// 把data in latch住，先回tready，可從12cycle(11讀1寫) 降到11cycle
// Global FSM控制內部FSM迴圈，裡面TAP COUNT外面DATA COUNT
// STATE MACHINE可反映PIPELINE 有幾個STAGE(EX: MUL有3T，則STATE loop可跑三次)
// 控制訊號集中，用FSM訊號控制整體邏輯
// tap_A = 32(arvalid)&araddr | 32(~arvalid&awvalid)&awaddr
// ------------------------------------------------------------------------

`define Data_Num 600
`define Param_Start  12'h80
`define Config_Size  12'hff
`define RAM_Size     32

module FIR #(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 32
)
(
    //----------------------------AXI Lite ports---------------------------
    output  wire                     awready,      // write address ready
    output  wire                     wready,       // write data ready
    input   wire                     awvalid,      // write address valid
    input   wire [(pADDR_WIDTH-1):0] awaddr,       // write address
    input   wire                     wvalid,       // write data valid
    input   wire [(pDATA_WIDTH-1):0] wdata,        // write data
    output  wire                     arready,      // read address ready
    input   wire                     rready,       // read data ready
    input   wire                     arvalid,      // read address valid
    input   wire [(pADDR_WIDTH-1):0] araddr,       // read address
    output  wire                     rvalid,       // read data valid
    output  wire [(pDATA_WIDTH-1):0] rdata,        // read data
    //----------------------------AXI Stream ports---------------------------
    input   wire                     ss_tvalid,    // stream transfer valid from slave
    input   wire [(pDATA_WIDTH-1):0] ss_tdata,     // stream transfer data from slave
    input   wire                     ss_tlast,     // stream transfer last from slave
    output  wire                     ss_tready,    // stream transfer ready from slave 
    input   wire                     sm_tready,    // stream transfer ready to slave 
    output  wire                     sm_tvalid,    // stream transfer valid to slave 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata,     // stream transfer data to slave
    output  wire                     sm_tlast,     // stream transfer last to slave 
    
    //--------------------tap RAM ports---------------------
    output  wire [3:0]               tap_WE,       // write enable
    output  wire                     tap_EN,       // R/W enable
    output  wire [(pDATA_WIDTH-1):0] tap_Di,       // input data
    output  wire [(pADDR_WIDTH-1):0] tap_A,        // address
    input   wire [(pDATA_WIDTH-1):0] tap_Do,       // output data

    //--------------------data RAM ports---------------------
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

    // write your code here!
    //---------------------------------------------------------------ENGINE FSM PARAM--------------------------------
    localparam CONF     = 4'd0;
    localparam STREAM   = 4'd1;
    localparam STALL_I  = 4'd2;
    localparam STALL_O  = 4'd3;
    localparam DONE     = 4'd4;
    //-------------------------------------------------------------Data condition PARAM---------------------------
    localparam DC_Idle = 3'd0;
    localparam DC_Write = 3'd1;
    localparam DC_Reuse = 3'd2;
    localparam DC_Read = 3'd3;
    localparam DC_Stall = 3'd4;
    localparam DC_Done = 3'd5;
    //-----------------------------CALCULATOR FSM--------------------------------
    localparam Cal_Idle = 3'd0;
    localparam Cal_Start = 3'd1;
    localparam Cal_MUL = 3'd2;
    localparam Cal_Accum = 3'd3;
    localparam Cal_Buf_Out = 3'd4;
    localparam Cal_Stall = 3'd5;
    localparam Cal_Done = 3'd6;

    wire [pDATA_WIDTH-1:0] DATA_LENGTH;
    wire [pDATA_WIDTH-1:0] TAP_NUM;
    // State signal
    reg [3:0] ap_state;
    reg [3:0] ap_state_nxt;
    wire [2:0] ap_ctrl; // {ap_idle, ap_done, ap_start}
    wire fsm_stall;
    //-----------------------------------------------------------config REG signal-----------------
    reg [pDATA_WIDTH-1:0] config_reg_r[0:(`Config_Size>>2)];// address/4 is the the number of reg
    reg [pDATA_WIDTH-1:0] config_reg_w[0:(`Config_Size>>2)];
    wire [pDATA_WIDTH-1:0] next_reg_val;
    wire config_en;
    wire [pADDR_WIDTH-1:0] config_end; // last address of config register
    integer i;
    //---------------------------------------------------------input data buffer signal-----------------
    reg [pDATA_WIDTH:0] input_buf_r; // including valid bit
    reg [pDATA_WIDTH:0] input_buf_w;
    reg [31:0] stream_in_data_cnt_r; // count how many data has been input
    reg [31:0] stream_in_data_cnt_w;
    wire input_buf_valid;
    //---------------------------------------------------------MULTIPLICATION signal----------------------------------
    reg signed [(pDATA_WIDTH<<1)-1:0] data_mul; // data_o * tap_o
    reg signed [(pDATA_WIDTH<<1)-1:0] pipelined_mul_r1; // pipeline after multiplication
    reg signed [(pDATA_WIDTH<<1)-1:0] pipelined_mul_w1;
    wire last_of_each_y; // 每項y的最後一筆read data
    reg last_of_each_y_r1;// accum in的最後一筆data
    reg last_of_each_y_r2;
    //------------------------------------------------------------accumulator signal----------------------------------
    wire signed [pDATA_WIDTH<<1:0] data_add;
    reg signed [(pDATA_WIDTH<<1):0] pipelined_add_r1; // pipeline after addition
    reg signed [(pDATA_WIDTH<<1):0] pipelined_add_w1;
    reg [5:0] accum_in_cnt_r; // 計算現在加到y[n]的第幾個
    reg [5:0] accum_in_cnt_w;
    reg [31:0] accum_out_cnt_r;
    reg [31:0] accum_out_cnt_w;
    reg add_input_sel;
    //------------------------------------------------------------output buffer signal---------------------------
    reg [(pDATA_WIDTH<<1)+1:0] out_buffer_r; // including valid bit
    wire [(pDATA_WIDTH<<1)+1:0] out_buffer_w;
    reg [31:0] stream_out_data_cnt_r; // count how many y has been transmitted
    wire [31:0] stream_out_data_cnt_w;
    wire output_buf_valid;
//=================================================DATA CONDITION Signal=======================================================================
    reg [2:0] dc_state;
    reg [2:0] dc_state_nxt;

    reg [4:0] dc_read_cycle_r;// define read how many cycles
    reg [4:0] dc_read_cycle_w;

    reg [4:0] dc_read_cnt_r; // count cycles in read state
    reg [4:0] dc_read_cnt_w;
    reg [31:0] dc_written_cnt_r; // to check if it is the first time to write
    reg [31:0] dc_written_cnt_w;
    //--------------------------data address generator signal---------------------------
    reg [pADDR_WIDTH-1:0] data_addr;
    reg [4:0] m_temp;
    reg [4:0] m;
    //--------------------------------------------------------tap address generator signal-----------------
    reg [$clog2(Tape_Num)-1:0] k_temp;
    reg [$clog2(Tape_Num)-1:0] k;      // address累加器
    reg [pADDR_WIDTH-1:0] tap_addr;
    //--------------------------------------------------------------tap RAM signal---------------------------
    wire tap_we_w;
    reg tap_en;
    reg [pADDR_WIDTH-1:0] awaddr_r;
    reg tap_we_r;
    //-------------------------------------------------------------data RAM signal---------------------------
    wire sel_dram_out;
    reg data_we_w;
    reg data_en;
    //-------------------------------------------------------data RAM address generator signal--------------------------
    reg [$clog2(`Data_Num)-1:0] data_count_r;
    wire [$clog2(`Data_Num)-1:0] data_count_w;
    // wire [pADDR_WIDTH-1:0] data_w_addr;
    // wire [pADDR_WIDTH-1:0] data_r_addr;
    // reg [4:0] ptr_dataRAM_read_r;
    wire [4:0] ptr_dataRAM_write;
    // reg [4:0] ptr_dataRAM_read_w;
    reg [4:0] dataRAM_read_cnt_r;
    reg [4:0] dataRAM_read_cnt_w;
    // reg [pDATA_WIDTH:0] data_reuse_buf_r;
    // reg [pDATA_WIDTH:0] data_reuse_buf_w;

    reg awready_r;
    reg wready_r;
    wire awready_w;
    wire wready_w;
    reg[1:0] lite_sel_r;
    wire[1:0] lite_sel_w;
    wire [pADDR_WIDTH-1:0] awaddr_w;
    reg [pDATA_WIDTH-1:0] wdata_r;
    wire [pDATA_WIDTH-1:0] wdata_w;
//--------------------------------------------------------------AXI LITE SIGNAL--------------------------------------------------------------
    reg arready_r;
    reg rvalid_r;
    wire arready_w;
    wire rvalid_w;
    wire rready_w;
    reg rready_r;
    wire [pADDR_WIDTH-1:0] read_addr_w;
    reg [pADDR_WIDTH-1:0] read_addr_r;
//==============================================================ENGINE STATE TRANSITION============================================================================
    // assign in_stall = (stream_in_data_cnt_r < TAP_NUM) ? ((dataRAM_read_cnt_r == stream_in_data_cnt_r) ? 1'b1 : 1'b0)
    //                                                 :((dataRAM_read_cnt_r == TAP_NUM) ? 1'b1 : 1'b0);
    assign in_stall = ~input_buf_valid |  ~(ss_tvalid & ss_tready);
    // assign out_stall = (stream_out_data_cnt_r < TAP_NUM) ? ((accum_in_cnt_r == (stream_out_data_cnt_r)) ? 1'b1 : 1'b0)
    //                      : ((accum_in_cnt_r == TAP_NUM) ? 1'b1 : 1'b0);
    assign out_stall = output_buf_valid &  ~(sm_tvalid & sm_tready) & (last_of_each_y_r1);
    always@(*)begin // State transition
      case(ap_state)
        CONF: begin
          if(ap_ctrl[0]) ap_state_nxt = STREAM;
          else ap_state_nxt = CONF;
        end 
        STREAM: begin
          if(sm_tvalid & sm_tready & (stream_out_data_cnt_r == (DATA_LENGTH-1))) ap_state_nxt = DONE;
        //   else if(~input_buf_valid & in_stall & ~(ss_tvalid & ss_tready)) ap_state_nxt = STALL_I;
          else if(in_stall) ap_state_nxt = STALL_I;
          else if(out_stall) ap_state_nxt = STALL_O;
          else ap_state_nxt = STREAM;
        end
        STALL_I: begin // 可能會跳到STALL_O
          if(input_buf_valid) begin
            if(out_stall) ap_state_nxt = STALL_O;
            else ap_state_nxt = STREAM;
          end
          else ap_state_nxt = STALL_I;
        end
        STALL_O: begin
          if(sm_tvalid & sm_tready) begin
            if(in_stall) ap_state_nxt = STALL_I;
            else ap_state_nxt = STREAM;
          end
          else ap_state_nxt = STALL_O;
        end
        DONE: ap_state_nxt = CONF;
        default: ap_state_nxt = CONF; 
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) ap_state <= CONF;
      else ap_state <= ap_state_nxt;
    end
 
//---------------------------------------------------------------Configuration Register--------------------------------------------------------------------

    assign DATA_LENGTH = config_reg_r[12'h10>>2];
    assign TAP_NUM = config_reg_r[12'h14>>2];

    assign config_end = `Param_Start + (TAP_NUM << 2);// address of the last tap 
    assign ap_ctrl = config_reg_r[12'h00][2:0];
    // assign config_en = (wready & awready & awvalid & wvalid)&(awaddr_r <= config_end);
    assign config_en = (awaddr_r <= config_end) & (&lite_sel_r);
    assign next_reg_val = (awaddr_r>>2 == 0) ? 
                          (wdata_r[0] ? {wdata_r[pDATA_WIDTH-1:3], 3'b001}
                          : {wdata_r[pDATA_WIDTH-1:3], ap_ctrl[2:0]}) : wdata_r;// ap_idle, ap_done can not be written, and ap_start will be written
    always@(*)begin // Writing configure register 
      case(ap_state)
        CONF: begin
          for(i=0;i<(`Config_Size>>2);i=i+1)begin
            if((i == (awaddr_r>>2)) & config_en) config_reg_w[i] = next_reg_val;
            else config_reg_w[i] = config_reg_r[i];
          end
        end
        STREAM: begin
          if(sm_tready & sm_tvalid & (stream_out_data_cnt_r == DATA_LENGTH))begin// last data transmitted
            for(i=1;i<config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
            config_reg_w[12'h00] = {config_reg_r[12'h00][pDATA_WIDTH-1:2]
                                              , 1'b1, ap_ctrl[0]};// pull ap_done since last data transmitted
          end
          else begin
            for(i=0;i<config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
          end
        end
        DONE: begin
            for(i=1;i<config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
            config_reg_w[12'h00] = {config_reg_r[12'h00][pDATA_WIDTH-1:3], 3'b100};// reset ap_ctrl by engine
        end
        default: begin
          for(i=0;i<config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
        end
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        for(i=1;i<(`Config_Size>>2);i=i+1) config_reg_r[i] <= 0;
        config_reg_r[12'h00] <= 3'b100;
      end
      else begin
        for(i=0;i<(`Config_Size>>2);i=i+1) config_reg_r[i] <= config_reg_w[i];
      end
    end
//-----------------------------------------------------------------DATA CONDITION FSM--------------------------------------------------------------------------

    always@(*)begin // Data condition state machine
      case(dc_state)
        DC_Idle: begin
          if(ss_tlast) dc_state_nxt = DC_Done;
          else begin
            if((ss_tvalid & ss_tready) | input_buf_valid) dc_state_nxt = DC_Write;
            else dc_state_nxt = DC_Idle;
          end
        end 
        DC_Write: dc_state_nxt = DC_Reuse;
        DC_Reuse: begin
          if(dc_written_cnt_r == 1'b1) begin
            if(in_stall) dc_state_nxt = DC_Stall;
            else dc_state_nxt = DC_Write;
          end
          else dc_state_nxt = DC_Read;
        end
        DC_Read: begin
          if(in_stall) dc_state_nxt = DC_Stall;
          else if(out_stall) dc_state_nxt = DC_Stall;
          else if
          else dc_state_nxt = DC_Write;
        end
        DC_Stall: begin
          if(ap_state == STREAM) dc_state_nxt = DC_Write;
          else dc_state_nxt = DC_Stall;
        end
        DC_Done: begin
          if(ap_state == CONF) dc_state_nxt = DC_Idle;
          else dc_state_nxt = DC_Done;
        end
        default: dc_state_nxt = DC_Idle;
      endcase
    end         
    always@(*)begin // determine how many cycles to read in read state
      case(dc_state)
        DC_Idle: begin
          if(sm_tlast) dc_read_cycle_w = 0; // reset when last data transmitted
          else dc_read_cycle_w = dc_read_cycle_r;
        end
        DC_Read: begin
          if(dc_state_nxt != DC_Read)begin
            if(dc_read_cycle_r < (TAP_NUM - 2'd3)) dc_read_cycle_w = dc_read_cycle_r + 1'b1;
            else dc_read_cycle_w = dc_read_cycle_r;
          end
          else dc_read_cycle_w = dc_read_cycle_r;
        end
        default: dc_read_cycle_w = dc_read_cycle_r;
      endcase
    end

    always@(*)begin //count cycles in read state
      case(dc_state)
        DC_Read: dc_read_cnt_w = dc_read_cnt_r + 1'b1;
        default: dc_read_cnt_w = 0;
      endcase
    end

    always@(*)begin // count enter times in write state
      case(dc_state)
        DC_Write: dc_written_cnt_w = dc_written_cnt_r + 1'b1;
        default: dc_written_cnt_w = dc_written_cnt_r;
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        dc_state <= DC_Idle;
        dc_read_cycle_r <= 0;
        dc_read_cnt_r <= 0;
        dc_written_cnt_r <= 0;
      end
      else begin
        dc_state <= dc_state_nxt;
        dc_read_cycle_r <= dc_read_cycle_w;
        dc_read_cnt_r <= dc_read_cnt_w;
        dc_written_cnt_r <= dc_written_cnt_w;
      end
    end
    //-----------------------------------------------------------data address generator---------------------------------------------------------------------
    // pointer 用減的， reading counter用加的
    assign ptr_dataRAM_write = 5'd31 - m;

    always@(*)begin // determine the address of dataRAM
      case(dc_state)
        DC_Write: data_addr = ptr_dataRAM_write << 2;
        DC_Reuse: data_addr = ((ptr_dataRAM_write + 2'd2) % 32) << 2;
        DC_Read:  data_addr = ((ptr_dataRAM_write + 2'd2 + dataRAM_read_cnt_r)%32) << 2;
        default: data_addr = 0;
      endcase
    end

    always@(*)begin // count read in read state
      case(dc_state)
        DC_Reuse: dataRAM_read_cnt_w = dataRAM_read_cnt_r + 1'b1;
        DC_Read: begin
          if(dc_state_nxt == DC_Read) dataRAM_read_cnt_w = dataRAM_read_cnt_r + 1'b1;
          else dataRAM_read_cnt_w = 0;
        end
        default: dataRAM_read_cnt_w = 0;
      endcase
    end

    always@(*)begin // Control of write pointer of dataRAM (automatically return to 0 when overflow)
      case(dc_state)
        DC_Idle: begin
          if(sm_tlast) m_temp = 0; // reset when last data transmitted
          else m_temp = m;
        end
        DC_Write: m_temp = m + 1'b1;
        // DC_Reuse: m_temp = m + 1'b1;
        default: m_temp = m;
      endcase
    end

    // always@(*)begin // data reuse buffer
    //   case(dc_state)
    //     DC_Write: data_reuse_buf_w = input_buf_r;
    //     DC_Reuse: data_reuse_buf_w = data_reuse_buf_r;
    //     default:  data_reuse_buf_w = 0;
    //   endcase
    // end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        m <= 0;
        dataRAM_read_cnt_r <= 0;
        // data_reuse_buf_r <= 0;
      end else begin
        m <= m_temp;
        dataRAM_read_cnt_r <= dataRAM_read_cnt_w;
        // data_reuse_buf_r <= data_reuse_buf_w;
      end
    end
    //------------------------tap address generator-----------------------------
    // wire tapRAM_rst;
    // assign tapRAM_rst = (stream_in_data_cnt_r < TAP_NUM) ? ((k == stream_in_data_cnt_r) ? 1'b1 : 1'b0)
    //                                                :(k == TAP_NUM) ? 1'b1 : 1'b0;
    // assign tap_addr = k << 2;// 跳下個word
    

    always@(*)begin
      case(ap_state)
        CONF: tap_addr = k << 2;
        STREAM: begin
          if(dc_state_nxt == DC_Write) tap_addr = (5'd0) << 2;
          else if(dc_state_nxt == DC_Reuse) tap_addr = 5'd1 << 2;
          else if(dc_state_nxt == DC_Read) tap_addr = ((k + 1'd1)%32) << 2;
          else tap_addr = 0;
        end
        default: begin
          if(dc_state_nxt == DC_Write) tap_addr = (5'd0) << 2;
          else tap_addr = 0;
        end
      endcase
    end

        
    always@(*)begin // 要給的是下一個cycle的address!!!!!!!!!!!!!!!!!!
      case(ap_state)
        CONF: begin
          if(tap_we_r) k_temp = k + 1'd1;
          else k_temp = k;
        end
        STREAM: begin
          if(dc_state == DC_Read) begin
            if((dc_state_nxt == DC_Read) | (stream_out_data_cnt_r >= 6'd31)) k_temp = k + 1'd1;
            else k_temp = 0;
          end
          else if(dc_state == DC_Reuse) k_temp = 5'd2;
          else if(dc_state == DC_Write) k_temp = 5'd1;
          else k_temp = 0;
        end
        default:   k_temp = 0;
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) k <= 0;
      else k <= k_temp;
    end
    //---------------------------INPUT DATA BUFFER----------------------------------
    assign input_buf_valid = input_buf_r[pDATA_WIDTH];
    
    always@(*)begin
      case(ap_state)
        DONE: stream_in_data_cnt_w = 0;
        default: begin
          if(ss_tvalid & ss_tready) stream_in_data_cnt_w = stream_in_data_cnt_r + 1'b1;
          else stream_in_data_cnt_w = stream_in_data_cnt_r;
        end
      endcase
    end

    always@(*)begin// 與有沒有STALL無關，buffer空了就可以進資料
      case(ap_state)
        CONF: input_buf_w = 0;
        DONE: input_buf_w = 0;
        default: begin
          if(input_buf_r[pDATA_WIDTH]) begin
            if(data_we_w) begin
              if(ss_tvalid & ss_tready) input_buf_w = {1'b1, ss_tdata};
              else input_buf_w = 0;
            end
            else input_buf_w = input_buf_r;
          end
          else begin
            if(ss_tvalid & ss_tready) input_buf_w = {1'b1, ss_tdata};
            else input_buf_w = 0;
          end
        end
      endcase
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        input_buf_r <= 0;
        stream_in_data_cnt_r <= 0;
      end
      else begin
        input_buf_r <= input_buf_w;
        stream_in_data_cnt_r <= stream_in_data_cnt_w;
      end
    end
    //------------------------------tap RAM(take from config reg)--------------------------------
    // assign tap_we_w = (awready & wready) & (ap_ctrl[2]) & (awaddr >= `Param_Start) ? 1'b1 : 1'b0;
    assign tap_we_w = &lite_sel_r & (awaddr_r >= `Param_Start) ;
    assign tap_WE = {4{tap_we_r}};// write enable has 4 bits
    assign tap_EN = tap_en;
    assign tap_Di = config_reg_r[awaddr_r>>2];
    assign tap_A = tap_addr;

    always@(*)begin
      case(ap_state)
        STALL_I: tap_en = 1'b0;
        STALL_O: tap_en = 1'b0;
        DONE: tap_en = 1'b0;
        default: tap_en = 1'b1;
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        awaddr_r <= 0;
        tap_we_r <= 0;
      end
      else begin
        awaddr_r <= awaddr;
        tap_we_r <= tap_we_w;
      end
    end
    //-----------------------------Data RAM--------------------------------
    // assign sel_dram_out = ap_ctrl[0] & (dataRAM_read_cnt_r == 0) & (input_buf_valid);// Only last 32 data can be written
    // assign data_we_w = sel_dram_out ? 1'b1 : 1'b0;// start writing when left 32 data(save last 32 data into RAM) 
    assign data_WE = {4{data_we_w}}; // write enable has 4 bits
    assign data_Di = input_buf_r[pDATA_WIDTH-1:0];
    assign data_A = data_addr;
    assign data_EN = data_en;
    
    always@(*)begin
      case(dc_state)
        DC_Idle: data_en = 1'b0;
        default: data_en = 1'b1;
      endcase
    end
    always@(*)begin
      case(dc_state)
        DC_Write: data_we_w = 1'b1;
        default: data_we_w = 1'b0;
      endcase
    end
//====================================================================END OF DC CONTROLL==============================================================

    
    reg [2:0] cal_state;
    reg [2:0] cal_state_nxt;

    always@(*)begin
      case(cal_state)
        Cal_Idle: begin
          if(ap_ctrl[0])begin
            if(dc_state_nxt == DC_Write) cal_state_nxt = Cal_MUL;
            else cal_state_nxt = Cal_Start;
          end
          else cal_state_nxt = Cal_Idle;
        end
        Cal_Start: begin
          if(dc_state_nxt == DC_Write) cal_state_nxt = Cal_MUL;
          else cal_state_nxt = Cal_Start;
        end
        Cal_MUL: cal_state_nxt = Cal_Accum;
        Cal_Accum: cal_state_nxt = Cal_Buf_Out;
        Cal_Buf_Out: begin
          if(ap_state_nxt == DONE) cal_state_nxt = Cal_Done;
          // else if((ap_state_nxt == STALL_I) | (ap_state_nxt == STALL_O)) cal_state_nxt = Cal_Stall;
          // else if(~((ap_state == STALL_I) | (ap_state == STALL_O)) & (accum_in_cnt_r == (accum_out_cnt_r)))  cal_state_nxt = Cal_Stall;
          // else cal_state_nxt = Cal_Buf_Out;
          else if((ap_state == STALL_I)) cal_state_nxt = Cal_Stall;
          else if((ap_state == STALL_O)) begin            
            if(accum_in_cnt_r == (accum_out_cnt_r - 1'b1)) cal_state_nxt = Cal_Stall;
            else cal_state_nxt = Cal_Buf_Out;
          end
          else cal_state_nxt = Cal_Buf_Out;
        end
        Cal_Stall: begin
          if(dc_state_nxt == DC_Write) cal_state_nxt = Cal_MUL;
          else cal_state_nxt = Cal_Stall;
        end
        Cal_Done: cal_state_nxt = Cal_Idle;
        default: cal_state_nxt = Cal_Idle;
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) cal_state <= Cal_Idle;
      else cal_state <= cal_state_nxt;
    end
    //----------------------------MULTIPLICATION-------------------------------------
    // assign data_mul = $signed(tap_Do) * $signed(data_Do);
    assign last_of_each_y = (dc_state == DC_Write);

    always@(*)begin // Control the input of pipeline register
      case(dc_state)
        DC_Write: data_mul = $signed(tap_Do) * $signed(input_buf_r[pDATA_WIDTH-1:0]);
        // DC_Reuse: data_mul = $signed(tap_Do) * $signed(data_reuse_buf_r[pDATA_WIDTH-1:0]);
        DC_Reuse: data_mul = $signed(tap_Do) * $signed(data_Do);
        DC_Read:  data_mul = $signed(tap_Do) * $signed(data_Do);
        default: data_mul = 0;
      endcase
    end
    always@(*)begin
      pipelined_mul_w1 = data_mul;
    end

    // always@(*)begin
    //   case(cal_state)
    //     // Cal_Idle: pipelined_mul_w1 = 0;
    //     // Cal_Start: pipelined_mul_w1 = 0;
    //     // Cal_MUL: pipelined_mul_w1 = data_mul;
    //     // Cal_Accum: pipelined_mul_w1 = data_mul;
    //     // Cal_Buf_Out: pipelined_mul_w1 = data_mul;
    //     // Cal_Stall: pipelined_mul_w1 = pipelined_mul_r1;
    //     DONE: pipelined_mul_w1 = 0;
    //     default: pipelined_mul_w1 = data_mul;
    //   endcase
    // end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) pipelined_mul_r1 <= 0;
      else pipelined_mul_r1 <= pipelined_mul_w1;
    end
    //-----------------------------ACCUMULATOR----------------------------------------------------  
    // assign data_add = add_input_sel ? pipelined_mul_r1 : (pipelined_mul_r1 + pipelined_add_r1);

    // always@(*)begin
    //   case(cal_state)
    //     Cal_Buf_Out: begin
    //       if(accum_in_cnt_r == accum_out_cnt_r) accum_out_cnt_w = accum_out_cnt_r + 1'b1;
    //       else accum_out_cnt_w = accum_out_cnt_r;
    //     end
    //     Cal_Stall: accum_out_cnt_w = accum_out_cnt_r;
    //     default: accum_out_cnt_w = accum_out_cnt_r;
    //   endcase
    // end      
    
    // always@(*)begin
    //   case(cal_state)
    //     Cal_Buf_Out: begin
    //       if(accum_out_cnt_r < TAP_NUM)begin
    //         if((accum_in_cnt_r == (accum_out_cnt_r))) add_input_sel = 1'b1; // ex: output 0筆資料共需進 accumulator 1次，所以加完1次才須重置
    //         else add_input_sel = 1'b0;
    //       end else begin
    //             if(accum_in_cnt_r == (TAP_NUM-1'b1)) add_input_sel = 1'b1;
    //             else add_input_sel = 1'b0;
    //       end
    //     end
    //     default: add_input_sel = 1'b0;
    //   endcase
    // end

    // always@(*)begin // Control the input of pipeline register
    //   case(cal_state)
    //     Cal_Idle: pipelined_add_w1 = 0;
    //     Cal_Start: pipelined_add_w1 = 0;
    //     Cal_MUL: pipelined_add_w1 = 0;
    //     Cal_Accum: pipelined_add_w1 = data_add;
    //     Cal_Buf_Out: pipelined_add_w1 = data_add;
    //     Cal_Stall: pipelined_add_w1 = pipelined_add_r1;
    //     DONE: pipelined_add_w1 = 0;
    //     default: pipelined_add_w1 = data_add;
    //   endcase
    // end

    // always@(*)begin // reset every first addition of y[n]
    //   case(cal_state)
    //     Cal_Idle: accum_in_cnt_w = 0;
    //     // Cal_Accum: accum_in_cnt_w = accum_in_cnt_r + 1'b1;// first multiplication doesn't have to be add
    //     Cal_Buf_Out: begin
    //       if(add_input_sel) accum_in_cnt_w = 0;
    //       // else if(dc_state == DC_Idle) accum_in_cnt_w = accum_in_cnt_r;
    //       else if((accum_in_cnt_r == (accum_out_cnt_r - 1'b1)) & (output_buf_valid & ~(sm_tvalid & sm_tready))) accum_in_cnt_w = accum_in_cnt_r;// 沒輸出就不要再累加
    //       else accum_in_cnt_w = accum_in_cnt_r + 1'b1;
    //     end
    //     Cal_Stall: accum_in_cnt_w = accum_in_cnt_r;
    //     Cal_Done: accum_in_cnt_w = 0;
    //     default: accum_in_cnt_w = accum_in_cnt_r;
    //   endcase
    // end
    
    // always@(posedge axis_clk or negedge axis_rst_n)begin
    //   if(~axis_rst_n)begin
    //     pipelined_add_r1 <= 0;
    //     accum_in_cnt_r <= 6'b0;
    //     accum_out_cnt_r <= 0;
    //   end
    //   else begin
    //     pipelined_add_r1 <= pipelined_add_w1;
    //     accum_in_cnt_r <= accum_in_cnt_w;
    //     accum_out_cnt_r <= accum_out_cnt_w;
    //   end
    // end
    reg [2:0] dc_state_r1;
    wire accum_buf_valid; // OK to push to output buffer
    reg accum_buf_valid_r;

    always@(*)begin // Control the input of pipeline register of accumulator
      case({(dc_state == DC_Idle), (dc_state_r1 == DC_Reuse)})
        2'b00: pipelined_add_w1 = pipelined_mul_r1 + pipelined_add_r1;
        2'b01: pipelined_add_w1 = pipelined_mul_r1;
        2'b10: pipelined_add_w1 = pipelined_add_r1;
        2'b11: pipelined_add_w1 = 0;
        default: pipelined_add_w1 = 0;
      endcase
    end
    assign accum_buf_valid = dc_state == DC_Reuse;
    always@(posedge axis_clk or negedge axis_rst_n)begin
       if(~axis_rst_n)begin
          dc_state_r1 <= DC_Idle;
          accum_buf_valid_r <= 0;
          pipelined_add_r1 <= 0;
          last_of_each_y_r1 <= 0;
        end
        else begin
          dc_state_r1 <= dc_state;
          accum_buf_valid_r <= accum_buf_valid;
          pipelined_add_r1 <= pipelined_add_w1;
          last_of_each_y_r1 <= last_of_each_y;
        end
    end
//---------------------------Output Buffer(Controlled by CALCULATOR FSM)-----------------------------------
    assign output_buf_valid = out_buffer_r[(pDATA_WIDTH<<1)+1];
    
    // always@(*)begin
    //   case(cal_state)
    //     Cal_Buf_Out: begin //main state
    //       if(output_buf_valid)begin // data valid
    //         if(sm_tvalid & sm_tready)begin // output y
    //           if(add_input_sel) out_buffer_w = {1'b1, pipelined_add_r1}; // output y and next data is also valid
    //           else out_buffer_w = 0;
    //         end
    //         else out_buffer_w = out_buffer_r;
    //       end
          
    //       else begin // data not valid
    //         if(add_input_sel) out_buffer_w = {1'b1, pipelined_add_r1};
    //         else out_buffer_w = 0;
    //       end
    //     end
    //     Cal_Stall: begin
    //       if(output_buf_valid)begin // data valid
    //         if(sm_tvalid & sm_tready)begin // output y
    //           if(add_input_sel) out_buffer_w = {1'b1, pipelined_add_r1}; // output y and next data is also valid
    //           else out_buffer_w = 0;
    //         end
    //         else out_buffer_w = out_buffer_r;
    //       end
          
    //       else begin // data not valid
    //         if(add_input_sel) out_buffer_w = {1'b1, pipelined_add_r1};
    //         else out_buffer_w = 0;
    //       end
    //     end
    //     // STALL_O: begin
    //     //   if(ap_state_nxt == STREAM) out_buffer_w = {1'b1, pipelined_add_r1};
    //     //   else out_buffer_w = out_buffer_r;
    //     // end
    //     // STALL_I: out_buffer_w = out_buffer_r;
    //     default: out_buffer_w = out_buffer_r;
    //   endcase
    // end
    
    // always@(*)begin
    //   case(cal_state)
    //     Cal_Done: stream_out_data_cnt_w = 0;
    //     default: begin
    //       if(sm_tvalid & sm_tready) stream_out_data_cnt_w = stream_out_data_cnt_r + 1'b1;
    //       else stream_out_data_cnt_w = stream_out_data_cnt_r;
    //     end
    //   endcase
    // end
    assign out_buffer_w = (output_buf_valid) ? ((sm_tvalid & sm_tready) ? (accum_buf_valid ? {1'b1, pipelined_add_r1} : 0) : out_buffer_r) : (accum_buf_valid ? {1'b1, pipelined_add_r1} : 0);
    assign stream_out_data_cnt_w = ap_ctrl[1] ? 0 : ((sm_tvalid & sm_tready) ? stream_out_data_cnt_r + 1'b1 : stream_out_data_cnt_r);

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        out_buffer_r <= 0;
        stream_out_data_cnt_r <= 0;
        last_of_each_y_r2 <= 0;
      end
      else begin
        out_buffer_r <= out_buffer_w;
        stream_out_data_cnt_r <= stream_out_data_cnt_w;
        last_of_each_y_r2 <= last_of_each_y_r1;
      end
    end
//---------------AXI Lite interface----------------
// rvalid要等arvalid和arready才能拉
    assign rready_w = rready;

    assign awready = awready_r;
    assign wready = wready_r;

    assign arready = arready_r;
    assign rvalid = rvalid_r;

    assign rdata = config_reg_r[read_addr_r>>2]; 
    assign arready_w = (arvalid & arready_r) ? (rvalid_r & rready_r) : 1'b1;
    assign rvalid_w = rvalid_r ? (rready_r ? (arvalid & arready_r) : 1'b1) : (arvalid & arready_r);
    assign read_addr_w = (arvalid & arready_r) ? araddr : read_addr_r;// refresh when address is valid

    assign lite_sel_w = (lite_sel_r == 2'b11) ? 2'b00 : {(awvalid & awready) | lite_sel_r[1], (wvalid & wready) | lite_sel_r[0]};
    assign awready_w = (lite_sel_r != 2'b10) ? 1'b1 : 1'b0;
    assign wready_w = (lite_sel_r != 2'b01) ? 1'b1 : 1'b0;
    assign awaddr_w = (awvalid & awready) ? awaddr : awaddr_r;
    assign wdata_w = (wvalid & wready) ? wdata : wdata_r;

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        awready_r <= 1'b0;
        wready_r <= 1'b0;
        arready_r <= 1'b0;
        rvalid_r <= 1'b0;
        rready_r <= 1'b0;
        read_addr_r <= 0;
        lite_sel_r <= 2'b00;
        awaddr_r <= 0;
        wdata_r <= 0;
      end
      else begin
        awready_r <= awready_w;
        wready_r  <= wready_w;
        arready_r <= arready_w;
        rvalid_r  <= rvalid_w;
        rready_r  <= rready_w;
        read_addr_r <= read_addr_w;
        lite_sel_r <= lite_sel_w;
        awaddr_r <= awaddr_w;
        wdata_r <= wdata_w;
      end
    end
//--------------------AXI Stream interface---------------------
    //---------------AXI Stream interface(x[n])--------------
    reg ss_tready_w;

    assign ss_tready = ss_tready_w;
    always@(*)begin
      case(ap_state)
        CONF: ss_tready_w = 1'b0;
        STREAM: begin
          if(input_buf_valid) ss_tready_w = 1'b0;
          else ss_tready_w = 1'b1;
        end
        STALL_I: ss_tready_w = 1'b1;
        STALL_O: begin 
          if(input_buf_valid) ss_tready_w = 1'b0;
          else ss_tready_w = 1'b1;
        end
        DONE: ss_tready_w = 1'b0;
        default: ss_tready_w = 1'b0;
      endcase
    end
        
    //--------------AXI Stream interface(y[n])---------------
    assign sm_tvalid = output_buf_valid;
    assign sm_tlast = sm_tready & sm_tvalid & (stream_out_data_cnt_w == DATA_LENGTH); // buffer clear and last data transferred
    assign sm_tdata = out_buffer_r[pDATA_WIDTH-1:0]; // output last 32 bit of data
endmodule