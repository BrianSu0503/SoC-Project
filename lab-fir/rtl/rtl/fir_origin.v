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
// 
// ------------------------------------------------------------------------
// 若Tap不足32，data RAM剩餘的空間可用來存下次FIR的資料
// 把data in latch住，先回tready，可從12cycle(11讀1寫) 降到11cycle
// Global FSM控制內部FSM迴圈，裡面TAP COUNT外面DATA COUNT
// STATE MACHINE可反映PIPELINE 有幾個STAGE(EX: MUL有3T，則STATE loop可跑三次)
// 控制訊號集中，用FSM訊號控制整體邏輯
// tap_A = 32(arvalid)&araddr | 32(~arvalid&awvalid)&awaddr
// ------------------------------------------------------------------------

`define AP_IDLE  3'd2 //改成parameter
`define AP_DONE  3'd1
`define AP_PROC  3'd0

`define Data_Num 600
`define Param_Start  12'h80
`define Config_Size  12'h180
`define RAM_Size     32

module fir #(  
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
//---------------Configuration REG-----------------
// 
    //---------------FSM for config--------------------------------
   
    reg [2:0] ap_state;
    reg [2:0] ap_state_nxt;
    wire [2:0] ap_ctrl; // {ap_idle, ap_done, ap_start}

    always@(*)begin
      case(ap_state)
        `AP_IDLE: begin
          if(ap_ctrl[0]) ap_state_nxt = `AP_PROC;
          else ap_state_nxt = `AP_IDLE;
        end 
        `AP_PROC: begin
          if(ap_ctrl[1]) ap_state_nxt = `AP_DONE;
          else ap_state_nxt = `AP_PROC;
        end
        `AP_DONE: ap_state_nxt = `AP_IDLE;
        default: ap_state_nxt = `AP_IDLE; 
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) ap_state <= `AP_IDLE;
      else ap_state <= ap_state_nxt;
    end

    
    //---------------config REG control signal-----------------
    reg [pDATA_WIDTH-1:0] config_reg_r[0:(`Config_Size>>2)];// address/4 is the the number of reg
    reg [pDATA_WIDTH-1:0] config_reg_w[0:(`Config_Size>>2)];
    wire [pDATA_WIDTH-1:0] next_reg_val;
    wire config_en;
    wire [pADDR_WIDTH-1:0] config_end;

    integer i;
    assign config_end = `Param_Start + (Tape_Num << 2);// address of the last tap 
    assign ap_ctrl = config_reg_r[12'h00][2:0];
    assign config_en = (awvalid & wvalid)&(awaddr <= config_end);
    assign next_reg_val = (awaddr == 12'h00) ? {{wdata[pADDR_WIDTH-1:3]
                          , ap_ctrl[2:1], wdata[0]}} : wdata;// ap_idle, ap_done can not be written
                                                             //, and ap_start will be written
    always@(*)begin // Writing configure register 
      case(ap_state)
        `AP_IDLE: begin
          if(config_en) config_reg_w[awaddr>>2] = next_reg_val; // next_reg_val already takes care of the Read-only parts
          else config_reg_w[awaddr>>2] = config_reg_r[awaddr>>2];
          for(i=0;i<awaddr>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
          for(i=(awaddr>>2)+1;i<(config_end)>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
        end
        `AP_PROC: begin
          if(sm_tlast)begin
            for(i=1;i<config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
            config_reg_w[12'h00] = {config_reg_r[12'h00][pDATA_WIDTH-1:2]
                                              , 1'b1, ap_ctrl[0]};// pull ap_done since last data transmitted
          end
          else begin
            for(i=1;i<config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
          end
        end
        `AP_DONE: begin
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
//---------------------------Address Generator-----------------------------
    
    //------------------------data address generator-----------------------------
    reg [$clog2(`Data_Num)-1:0] m;// address累加器
    reg [$clog2(`Data_Num)-1:0] data_count_r;
    wire [$clog2(`Data_Num)-1:0] data_count_w;
    wire [$clog2(`Data_Num)-1:0] m_temp;
    wire [pADDR_WIDTH-1:0] data_addr;

    //assign m_temp = (m == (`Data_Num-1)) ? 0 : (m + 1'd1);// 超過Data_Num歸零
    assign m_temp = (data_count_r >= (`Data_Num - `RAM_Size)) ? ((m == `RAM_Size) ? 0 : (m + 1'd1)) : m; // 數到倒數RAM_Size(32)個就開始加
    assign data_addr = (`RAM_Size-m_temp) << 2;// 跳下個word
    assign data_count_w = (~ap_ctrl[0] | (data_count_r == `Data_Num)) ? data_count_r : data_count_r + 1'b1;// start就開始加，加到600不動，以判斷是否寫入

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) m <= 0;
      else m <= m_temp;
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) data_count_r <= 0;
      else data_count_r <= data_count_w;
    end
    //------------------------tap address generator-----------------------------
    wire [$clog2(Tape_Num)-1:0] k_temp;
    reg [$clog2(Tape_Num)-1:0] k;      // address累加器
    wire [pADDR_WIDTH-1:0] tap_addr;

    assign k_temp = (k == (Tape_Num)) ? 0 : (((awaddr >= `Param_Start) & (awready & wready)) | ap_ctrl[0] ? (k + 1'd1) : k);// 超過tap_num歸零
    //assign tap_addr = (~ap_ctrl[2]) ? (k<<2) : araddr;
    assign tap_addr = k<<2;

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n) k <= 0;
      else k <= k_temp;
    end
//---------------AXI Lite interface----------------
// rvalid要等arvalid和arready才能拉
    reg awready_r;
    reg wready_r;
    wire awready_w;
    wire wready_w;

    reg arready_r;
    reg rvalid_r;
    wire arready_w;
    wire rvalid_w;

    assign awready = awready_r;
    assign wready = wready_r;

    assign arready = arready_r;
    assign rvalid = rvalid_r;

    assign rdata = config_reg_r[araddr>>2];
    assign arready_w = arvalid & rready;// 有問題?
    assign rvalid_w = arvalid & rready;// if address is reasonable, then data is valid

    assign awready_w = awvalid & wvalid;
    assign wready_w = awvalid & wvalid;

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        awready_r <= 1'b0;
        wready_r <= 1'b0;
        arready_r <= 1'b0;
        rvalid_r <= 1'b0;
      end
      else begin
        awready_r <= awready_w;
        wready_r  <= wready_w;
        arready_r <= arready_w;
        rvalid_r  <= rvalid_w;
      end
    end
//----------------Data RAM------------------------------------
    // assign data_EN = ss_tvalid;
    // assign data_WE = (data_count_r < `Data_Num);
    // assign data_Di = ss_tdata;
    // assign data_A = data_addr;
    

//------------------------------tap RAM(take from config reg)--------------------------------
    bram32 tap_ram(
        .CLK(axis_clk),
        .WE(tap_WE),
        .EN(tap_EN),
        .Di(tap_Di),
        .Do(tap_Do),
        .A(tap_addr)
    );
    wire tap_we_w;
    //assign tap_we_w = (ap_ctrl[2] & (awaddr >= `Param_Start) & awvalid & wvalid) ? 1'b1 : 1'b0;
    assign tap_we_w = (awready & wready) & (ap_ctrl[2]) & (awaddr >= `Param_Start) ? 1'b1 : 1'b0;
    assign tap_WE = {4{tap_we_w}};// write enable has 4 bits
    //assign tap_EN = arvalid & rready;
    assign tap_EN = tap_we_w | (data_count_r == `Data_Num);// 如果Tap RAM寫入較慢就會有問題
    assign tap_Di = config_reg_r[awaddr>>2];
    //assign tap_A = (ap_ctrl[2]) ? awaddr : tap_addr;// idle: awaddr, start: increment
    assign tap_A = tap_addr;
//-----------------------------Data RAM--------------------------------
    bram32 data_ram(
        .CLK(axis_clk),
        .WE(data_WE),
        .EN(data_EN),
        .Di(data_Di),
        .Do(data_Do),
        .A(data_A)
    );
    wire sel_dram_out;
    wire data_we_w;
    assign sel_dram_out = (data_count_r >= (`Data_Num - `RAM_Size)) & (data_count_r < `Data_Num);// Only last 32 data can be written
    //assign sel_dram_out = ((data_count_r - Tape_Num) & ap_ctrl[0]) ? 1'b1 : 1'b0;
    assign data_we_w = sel_dram_out ? 1'b1 : 1'b0;// start writing when left 32 data(save last 32 data into RAM) 
    assign data_WE = {4{data_we_w}}; // write enable has 4 bits
    //assign data_EN = ss_tvalid;
    assign data_EN = sel_dram_out | ((data_count_r == `Data_Num) & ap_ctrl[0]);// specify the read time and write time
    assign data_Di = ss_tdata;
    assign data_A = data_addr;

//---------------------------Calculation part(including output buffer)-----------------------------------
    reg signed [pDATA_WIDTH:0] pipelined_D_Do_r1; // pipeline after data out 
    wire signed[pDATA_WIDTH:0] pipelined_D_Do_W1;// extra bit for tlast flag
    reg signed [pDATA_WIDTH:0] pipelined_T_Do_r1;// pipeline after tap out
    wire signed[pDATA_WIDTH:0] pipelined_T_Do_W1;// extra bit for tlast flag
    reg signed [pDATA_WIDTH<<1:0] pipelined_mul_r1; // pipeline after multiplication
    wire signed [pDATA_WIDTH<<1:0] pipelined_mul_w1; // extra bit for tlast flag
    reg signed [pDATA_WIDTH<<1:0] pipelined_add_r1; // pipeline after addition
    wire signed [pDATA_WIDTH<<1:0] pipelined_add_w1;// extra bit for tlast flag

    //for output buffer(4 modes: R&W, R, W, IDLE)
    wire [pDATA_WIDTH<<1:0] yn;
    wire empty_flag;
    wire buf_in_flag;
    wire buf_out_flag;
    wire [2:0]buf_ctrl;
    reg [pADDR_WIDTH-1:0] ptr_out_buffer_r;
    reg [pADDR_WIDTH-1:0] ptr_out_buffer_w;
    reg signed [pDATA_WIDTH<<1:0] out_buffer_r[0:pDATA_WIDTH<<1-1];// extra bit for tlast flag
    reg signed [pDATA_WIDTH<<1:0] out_buffer_w[0:pDATA_WIDTH<<1-1];
    
    //combination logic
    assign pipelined_D_Do_W1 = {ss_tlast, data_Do};
    assign pipelined_T_Do_W1 = {ss_tlast, tap_Do};
    assign pipelined_mul_w1 = {pipelined_D_Do_r1[pDATA_WIDTH]
                              , $signed(pipelined_T_Do_r1[0+:pDATA_WIDTH]) * $signed(pipelined_D_Do_r1[0+:pDATA_WIDTH])};
    assign pipelined_add_w1 = {pipelined_mul_r1[pDATA_WIDTH<<1], $signed(pipelined_mul_r1[0+:(pDATA_WIDTH<<1)] 
                              + $signed(pipelined_add_r1[0+:(pDATA_WIDTH<<1)]))};

    assign buf_ctrl = {empty_flag, buf_in_flag, buf_out_flag};
    assign empty_flag = ptr_out_buffer_r == 1'b0;
    //assign buf_in_flag = (ap_ctrl[0]) & ((arvalid & rready) & ss_tvalid); // tap_EN & data_EN & ap_start
    assign buf_in_flag = ((data_count_r == `Data_Num) & ap_ctrl[0]);
    assign buf_out_flag = sm_tready & ~empty_flag;
    assign yn = (&buf_ctrl[2:1]) ? pipelined_add_r1 : 
                      ((empty_flag) ? (pDATA_WIDTH<<1)'h0 : out_buffer_r[ptr_out_buffer_r-1]);// FIFO data control
    
    always@(*)begin // pointer moving
      case(buf_ctrl)
        3'b010: ptr_out_buffer_w = ptr_out_buffer_r + 1'b1;
        3'b001: ptr_out_buffer_w = ptr_out_buffer_r - 1'b1;
        default: ptr_out_buffer_w = ptr_out_buffer_r;
      endcase
    end

    always@(*)begin // buffer writing
      case(buf_ctrl)
        3'b001: begin// only read out
          for(i=0;i<(pDATA_WIDTH<<1)-1;i=i+1) out_buffer_w[i] = out_buffer_r[i+1];
        end
        3'b010: begin// only write in
          out_buffer_w[ptr_out_buffer_r] = pipelined_add_r1;
        end
        3'b011: begin// read out and write in
          for(i=0;i<(ptr_out_buffer_r - 1);i=i+1) out_buffer_w[i] = out_buffer_r[i+1];
          for(i=ptr_out_buffer_r;i<(pDATA_WIDTH<<1)-1;i=i+1) out_buffer_w[i] = out_buffer_r[i+1];
          out_buffer_w[(ptr_out_buffer_r - 1)] = pipelined_add_r1;
        end
        3'b110: begin// empty and write in
          out_buffer_w[ptr_out_buffer_r] = pipelined_add_r1;
        end
        default: begin
          for(i=0;i<(pDATA_WIDTH<<1);i=i+1) out_buffer_w[i] = out_buffer_r[i];
        end
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        ptr_out_buffer_r <= 0;
        for(i=0;i<(pDATA_WIDTH<<1);i=i+1) out_buffer_r[i] <= 0;
      end else begin
        ptr_out_buffer_r <= ptr_out_buffer_w;
        for(i=0;i<(pDATA_WIDTH<<1);i=i+1) out_buffer_r[i] <= out_buffer_w[i];
      end
    end
    //pipeline
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin 
        pipelined_D_Do_r1 <= 0;
        pipelined_T_Do_r1 <= 0;
        pipelined_mul_r1 <= 0;
        pipelined_add_r1 <= 0;
      end
      else begin
        pipelined_D_Do_r1 <= pipelined_D_Do_W1;
        pipelined_T_Do_r1 <= pipelined_T_Do_W1;
        pipelined_mul_r1 <= pipelined_mul_w1;
        pipelined_add_r1 <= pipelined_add_w1;
      end
    end
//--------------------AXI Stream interface---------------------
    //---------------AXI Stream interface(x[n])--------------
    assign ss_tready = ap_ctrl[0] & (data_count_r < `Data_Num);
    //--------------AXI Stream interface(y[n])---------------
    assign sm_tvalid = ~empty_flag;
    assign sm_tlast = empty_flag & (yn[pDATA_WIDTH<<1]); // buffer clear and last data transferred
    assign sm_tdata = yn[0+:pDATA_WIDTH];

endmodule