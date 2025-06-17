module FIR #(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Param_Start = 12'h80,
    parameter Config_end = 12'hff
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
    localparam IDLE = 3'd0;
    localparam CONF = 3'd1;
    localparam WRITE = 3'd2;
    localparam READ = 3'd3;
    localparam STALL_I = 3'd4;
    localparam STALL_O = 3'd5;
    localparam DONE = 3'd6;

    //------------------------------------config register-----------------------------------
    wire ap_idle, ap_start, ap_done;
    wire [$clog2(pDATA_WIDTH)-1:0] TAP_NUM;
    wire [$clog2(600):0] DATA_LENGTH;
    reg [pDATA_WIDTH-1:0] config_reg_r[0:2];
    reg [pDATA_WIDTH-1:0] config_reg_w[0:2];

    //------------------------------------fsm reg--------------------------------------------
    reg [2:0] state;
    reg [2:0] state_nxt;
    wire gated_clk, clk_enable;

    //-------------------------------------data ram signal-------------------------------------
    wire [pDATA_WIDTH:0] mul_x_in;

    //-------------------------------------address generater signal---------------------------
    reg [$clog2(pDATA_WIDTH)-1:0] data_write_ptr_r;
    reg [$clog2(pDATA_WIDTH)-1:0] data_write_ptr_w;
    reg [$clog2(pDATA_WIDTH)-1:0] data_read_ptr_r;
    reg [$clog2(pDATA_WIDTH)-1:0] data_read_ptr_w;
    reg [$clog2(pDATA_WIDTH)-1:0] data_read_cnt_r;
    wire [$clog2(pDATA_WIDTH)-1:0] data_read_cnt_w;
    reg [pADDR_WIDTH-1:0] awaddr_r;

    wire tap_we_w;
    reg [pADDR_WIDTH-1:0] tap_addr;
    reg [$clog2(pDATA_WIDTH)-1:0] k_temp;
    reg [$clog2(pDATA_WIDTH)-1:0] k;

    //------------------------------------tap ram signal----------------------------------------
    reg [pDATA_WIDTH:0] lite_buf_r;
    wire [pDATA_WIDTH:0] lite_buf_w;

    //------------------------------------multiplier signal---------------------------------------
    wire signed[(pDATA_WIDTH << 1)-1:0] mul_w;

    //------------------------------------accumulator signal-------------------------------------
    wire signed[pDATA_WIDTH << 1:0] accum_w;
    wire signed[(pDATA_WIDTH<<1)-1:0] accum_in;

    //--------------------------------------AXI-stream signal----------------------------------------
    reg [$clog2(600):0] y_cnt_r;
    reg [$clog2(600):0] y_cnt_w;
    reg [pDATA_WIDTH:0] stream_in_buf_w;
    reg [pDATA_WIDTH:0] stream_in_buf_r;
    reg [pDATA_WIDTH:0] stream_out_buf_w;
    reg [pDATA_WIDTH:0] stream_out_buf_r;
    wire x_buf_v;
    wire y_buf_v;

    //--------------------------------------AXI-Lite signal-------------------------------------------
    wire request_w; // detect arvalid and arready and save
    reg request_r;
    reg awready_r;
    reg wready_r;
    wire awready_w;
    wire wready_w;
    reg[1:0] lite_sel_r; // to check the transfer
    wire[1:0] lite_sel_w;
    wire [pADDR_WIDTH-1:0] awaddr_w;
    reg [pDATA_WIDTH-1:0] wdata_r;
    wire [pDATA_WIDTH-1:0] wdata_w;
    
    reg arready_r;
    reg rvalid_r;
    wire arready_w;
    wire rvalid_w;
    wire rready_w;
    reg rready_r;
    wire [pADDR_WIDTH-1:0] read_addr_w;
    reg [pADDR_WIDTH-1:0] read_addr_r;
    wire lite_buf_v;

    //------------------------------------pipeline signal----------------------------------------------
    reg [$clog2(pDATA_WIDTH):0] pipeline_data_cnt_0_r; // Ram stage
    reg [$clog2(pDATA_WIDTH):0] pipeline_data_cnt_1_r; // multiplier stage
    reg [$clog2(pDATA_WIDTH):0] pipeline_data_cnt_2_r; // accumulation stage
    reg [$clog2(pDATA_WIDTH):0] pipeline_data_cnt_0_w; // Ram stage
    wire [$clog2(pDATA_WIDTH):0] pipeline_data_cnt_1_w; // multiplier stage
    wire [$clog2(pDATA_WIDTH):0] pipeline_data_cnt_2_w; // accumulation stage

    reg signed[pDATA_WIDTH-1:0] tap_Do_r; // pipeline stage 1
    reg signed[pDATA_WIDTH-1:0] data_Do_r;

    reg signed[(pDATA_WIDTH << 1)-1:0] mul_r;// pipeline stage 2

    reg signed[pDATA_WIDTH:0] accum_r;// pipeline stage 3

    //================================================================== FSM =======================================================================
    assign clk_enable = ~((state == STALL_I)|(state == STALL_O));
    assign gated_clk = clk_enable & axis_clk;
    
    always@(*)begin
      case(state)
        IDLE: state_nxt = CONF;
        CONF: begin
          if(ap_start & ~(|data_write_ptr_r)) state_nxt = WRITE;
          else state_nxt = CONF;
        end
        WRITE: begin
          if(x_buf_v) state_nxt = READ;
          else state_nxt = STALL_I;
        end
        READ: begin
          if(y_cnt_r == DATA_LENGTH) state_nxt = DONE;
          else if(data_read_cnt_r == (TAP_NUM-1'b1)) state_nxt = WRITE;
          else if(y_buf_v & pipeline_data_cnt_2_r == (TAP_NUM)) state_nxt = STALL_O;
          else state_nxt = READ;
        end
        STALL_I: begin
          if(x_buf_v) state_nxt = WRITE;
          else state_nxt = STALL_I;
        end
        STALL_O: begin
          if(!y_buf_v) state_nxt = READ;
          else state_nxt = STALL_O;
        end
        DONE: begin
          if(ap_done & tap_addr == 0) state_nxt = IDLE;
          else state_nxt = DONE;
        end
      endcase
    end
    
    always@(posedge axis_clk or negedge axis_rst_n) begin
      if(~axis_rst_n) state <= IDLE;
      else state <= state_nxt;
    end

    //================================================================== Configuration register =====================================================
    integer i;
    assign ap_idle = config_reg_r[0][2];
    assign ap_start = config_reg_r[0][1];
    assign ap_done = config_reg_r[0][0];

    always@(*)begin // [0]: ap_ctrl, [1]: data length, [2]: tap number
      case(state)
        CONF: begin
          if(request_r) begin 
            if(tap_addr == 8'h00)begin
              config_reg_w[0] = {config_reg_r[0][pDATA_WIDTH-1:2], rdata[1], config_reg_r[0][0]};
              config_reg_w[1] = config_reg_r[1];
              config_reg_w[2] = config_reg_r[2];
            end else if(tap_addr == 8'h10)begin
              config_reg_w[0] = config_reg_r[0];
              config_reg_w[1] = rdata;
              config_reg_w[2] = config_reg_r[2];
            end else if(tap_addr == 8'h14)begin
              config_reg_w[0] = config_reg_r[0];
              config_reg_w[1] = config_reg_r[1];
              config_reg_w[2] = rdata;
            end else begin
              for(i = 0; i < 2; i = i + 1) config_reg_w[i] = config_reg_r[i];
            end
          end
          else begin
            for(i = 0; i < 2; i = i + 1) config_reg_w[i] = config_reg_r[i];
          end
        end
        READ: begin
          if(y_cnt_r == DATA_LENGTH) begin
            config_reg_w[0] = {config_reg_r[0][pDATA_WIDTH-1:3], 3'b101};
            config_reg_w[1] = config_reg_r[1];
            config_reg_w[2] = config_reg_r[2];
          end else begin
            for(i = 0; i < 2; i = i + 1) config_reg_w[i] = config_reg_r[i];
          end
        end
        default: begin 
          for(i = 0; i < 2; i = i + 1)begin
            config_reg_w[i] = config_reg_r[i];
          end
        end
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        for(i = 0; i < 2; i = i + 1) config_reg_r[i] <= 0;
      end else begin
        for(i = 0; i < 2; i = i + 1) config_reg_r[i] <= config_reg_w[i];
      end
    end

    //================================================================= Address generater(Tap) ======================================================
    always@(*)begin
      case(state)
        WRITE: k_temp = k + 1'b1;
        READ: begin
          if(k == TAP_NUM) k_temp = 0;
          else k_temp = k + 1'b1;
        end
        STALL_I: k_temp = k;
        STALL_O: k_temp = k;
        default: k_temp = 0;
      endcase
    end

    always@(*)begin
      case(state)
        CONF: begin
          if(tap_we_w) tap_addr = awaddr_r[0+:5];
          else if(request_r) tap_addr = read_addr_r[0+:5];
          else tap_addr = k;
        end
        default: tap_addr = k;
      endcase
    end
    //================================================================= Address generater(data RAM) ======================================================
    assign data_read_cnt_w = ((state == WRITE) | (state == READ)) ? 
                             ((data_read_cnt_r == (TAP_NUM - 1'b1)) ? 0 : data_read_cnt_r + 1'b1) : data_read_cnt_r;
    assign data_addr = ((state_nxt == WRITE)) ? 
                       ((state == CONF) ? 0 : data_write_ptr_r) :  data_read_ptr_r;
    
    always@(*)begin
      case(state)
        CONF: begin
          if(state_nxt == WRITE) data_write_ptr_w = 0;
          else data_write_ptr_w = data_write_ptr_r + 1'b1; // To preset every word of data RAM
        end
        WRITE: data_write_ptr_w = data_write_ptr_r + 1'b1;
        READ: data_write_ptr_w = data_write_ptr_r;
        STALL_I: data_write_ptr_w = data_write_ptr_r;
        STALL_O: data_write_ptr_w = data_write_ptr_r;
        default: data_write_ptr_w = 0;
      endcase
    end

    always@(*)begin
      case(state)
        IDLE: data_read_ptr_w = 0;
        CONF: data_read_ptr_w = 0;
        WRITE: data_read_ptr_w = data_read_ptr_r + 1'b1;
        READ: begin
          if(data_read_cnt_r == (TAP_NUM - 2'd2)) data_read_ptr_w = data_write_ptr_r - 1'b1;
          else data_read_ptr_w = data_read_ptr_r + 1'b1;
        end
        STALL_I: data_read_ptr_w = data_read_ptr_r;
        STALL_O: data_read_ptr_w = data_read_ptr_r;
        default: data_read_ptr_w = 0;
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        data_read_ptr_r <= 0;
        data_write_ptr_r <= 0;
        data_read_cnt_r <= 0;
      end else begin
        data_read_ptr_r <= data_read_ptr_w;
        data_write_ptr_r <= data_write_ptr_w;
        data_read_cnt_r <= data_read_cnt_w;
      end
    end

    //================================================================= AXI-Stream in(x[n]) ====================================================
    assign x_buf_v = stream_in_buf_r[pDATA_WIDTH];
    assign ss_tready = !x_buf_v | (state == WRITE);

    always@(*)begin
      case(state)
        IDLE: stream_in_buf_w = 0;
        DONE: stream_in_buf_w = 0;
        WRITE: stream_in_buf_w = {ss_tvalid, wdata};
        default: begin
          if(!x_buf_v)begin
            if(ss_tvalid & ss_tready) stream_in_buf_w = {1'b1, wdata};
            else stream_in_buf_w = stream_in_buf_r;
          end 
          else stream_in_buf_w = stream_in_buf_r;
        end
      endcase
    end
    
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        stream_in_buf_r <= 0;
      end else begin
        stream_in_buf_r <= stream_in_buf_w;
      end
    end

    //================================================================= AXI-stream out(y[n]) ==================================================
    assign y_buf_v = stream_out_buf_r[pDATA_WIDTH];
    assign sm_tvalid = y_buf_v;
    assign sm_tlast = (y_cnt_r == DATA_LENGTH);

    always@(*)begin
      case(state)
        READ: begin
          if(y_buf_v) stream_out_buf_w = stream_out_buf_r;
          else begin
            if(pipeline_data_cnt_2_r == TAP_NUM) stream_out_buf_w = {1'b1, accum_r[0+:pDATA_WIDTH]};
            else stream_out_buf_w = stream_out_buf_r;
          end
        end
        DONE: stream_out_buf_w = 0;
        default: begin
          if(sm_tvalid & sm_tready) stream_out_buf_w = 0;
          else stream_out_buf_w = stream_out_buf_r;
        end
      endcase
    end
    
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        stream_out_buf_r <= 0;
      end else begin
        stream_out_buf_r <= stream_out_buf_w;
      end
    end
    
    //================================================================= Tap RAM ===============================================================
    assign tap_we_w = &lite_sel_r & (awaddr_r >= Param_Start) ;
    assign tap_WE = {4{tap_we_w}};
    assign tap_EN = 1;
    assign tap_Di = wdata_r;
    assign tap_A = tap_addr;
    
    assign lite_buf_w = ((read_addr_r >= Param_Start) & request_r) ? {1'b1, tap_Do} 
                        : ((rvalid & rready) ? 0 : lite_buf_r);
    assign lite_buf_v = lite_buf_r[pDATA_WIDTH];

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        lite_buf_r <= 0;
      end else begin
        lite_buf_r <= lite_buf_w;
      end
    end
    
    //================================================================= Data RAM =============================================================
    assign mul_x_in = (state == WRITE) ? stream_in_buf_r[0+:pDATA_WIDTH] : data_Do;

    assign data_we_w = (state_nxt == WRITE);
    assign data_WE = {4{data_we_w}}; // write enable has 4 bits
    assign data_Di = stream_in_buf_r[0+:pDATA_WIDTH];
    assign data_A = data_addr;
    assign data_EN = 1;

    //=============================================================== Pipeline ================================================================
    //assign pipeline_data_cnt_0_w = (state_nxt == WRITE) ? 1 : (((state == WRITE)|(state == READ)) ? pipeline_data_cnt_0_r + 1'b1 : pipeline_data_cnt_0_r);
    assign pipeline_data_cnt_1_w = pipeline_data_cnt_0_r;
    assign pipeline_data_cnt_2_w = pipeline_data_cnt_1_r;

    always@(*)begin
      if(state_nxt == WRITE) pipeline_data_cnt_0_w = 1; // 進WRITE state時有第一筆資料
      else begin
        if((state == WRITE)|(state == READ)) pipeline_data_cnt_0_w = pipeline_data_cnt_0_r + 1'b1;
        else pipeline_data_cnt_0_w = pipeline_data_cnt_0_r;
      end
    end

    always@(posedge gated_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        tap_Do_r <= 0;
        data_Do_r <= 0;
        mul_r <= 0;
        accum_r <= 0;
        pipeline_data_cnt_0_r <= 0;
        pipeline_data_cnt_1_r <= 0;
        pipeline_data_cnt_2_r <= 0;
      end else begin
        tap_Do_r <= tap_Do;
        data_Do_r <= data_Do;
        mul_r <= mul_w;
        accum_r <= accum_w;
        pipeline_data_cnt_0_r <= pipeline_data_cnt_0_w;
        pipeline_data_cnt_1_r <= pipeline_data_cnt_1_w;
        pipeline_data_cnt_2_r <= pipeline_data_cnt_2_w;
      end
    end

    //================================================================= Multiplier =======================================================
    assign mul_w = tap_Do_r * data_Do_r;
    
    //================================================================= Accumulator =======================================================
    assign accum_in = (pipeline_data_cnt_2_r == 1) ? 0 : accum_r; 
    assign accum_w =  accum_in + mul_r;

    //================================================================= AXI-Lite =======================================================
    // rvalid要等arvalid和arready才能拉
    assign rready_w = rready;

    assign awready = awready_r;
    assign wready = wready_r;

    assign arready = arready_r;
    assign rvalid = rvalid_r;

    assign rdata = (read_addr_r >= Param_Start) ? ((~ap_start ? lite_buf_r[0+:pDATA_WIDTH] : {pDATA_WIDTH{1'b1}})): 
                                                       config_reg_r[read_addr_r>>2]; 
    assign arready_w = (arvalid & arready_r) ? (rvalid_r & rready_r) : 1'b1;
    // assign rvalid_w = rvalid_r ? (rready_r ? (arvalid & arready_r) : 1'b1) : (arvalid & arready_r);
    assign rvalid = lite_buf_v | ((read_addr_r < Param_Start) & request_r); // valid when buffer has data
    assign read_addr_w = (arvalid & arready_r) ? araddr : read_addr_r;// refresh when address is valid

    assign request_w = (rvalid & rready) ? (arvalid & arready) : (request_r | (arvalid & arready));

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
        request_r <= 0;
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
        request_r <= request_w;
      end
    end

endmodule