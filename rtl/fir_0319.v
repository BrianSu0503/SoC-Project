module FIR #(  
    parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 32,
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
    //-------------------------------------------------FSM Parameters----------------------------------------------
    localparam CONF = 3'd0;
    localparam READ = 3'd1;    
    localparam WRITE = 3'd2;
    localparam WRITE_RE = 3'd3;
    localparam DONE = 3'd4;

    integer i;
    //----------------------------------FSM signal-----------------------------
    reg [2:0] ap_state;
    reg [2:0] ap_state_nxt;
    //---------------------------------Data Ram signal---------------------------
    // reg [5:0] ptr_dataRAM_read_r;
    // reg [5:0] ptr_dataRAM_read_w;
    reg [4:0] dataRAM_read_cnt_r;
    reg [4:0] dataRAM_read_cnt_w;
    reg [4:0] ptr_dataRAM_write_r;
    reg [4:0] ptr_dataRAM_write_w;
    reg [pADDR_WIDTH-1:0] data_addr_w;
    wire data_read_rst;
    //---------------------------------Tap signal----------------------------------
    reg [pADDR_WIDTH-1:0] tap_addr_w;
    wire tap_we_w;
    reg tap_we_r;
    reg [4:0] ptr_tapRAM_r;
    reg [4:0] ptr_tapRAM_w;
    //---------------------------------Configure Register signal---------------------------
    reg [pDATA_WIDTH-1:0] config_reg_r[0:(Config_end>>2)-1];
    reg [pDATA_WIDTH-1:0] config_reg_w[0:(Config_end>>2)-1];
    wire [pDATA_WIDTH-1:0] TAP_NUM;
    wire [pDATA_WIDTH-1:0] DATA_LENGTH;
    wire done_read;
    wire config_in_sel;
    wire [2:0] ap_ctrl;
    wire ap_start;
    wire ap_idle;
    wire ap_done;
    //---------------------------------multiplier signal-----------------------------------
    wire signed [(pDATA_WIDTH<<1)-1:0] data_mul;
    //---------------------------------accumulator signal---------------------------------
    reg signed [(pDATA_WIDTH<<1):0] accum_reg_r;
    reg signed [(pDATA_WIDTH<<1):0] accum_reg_w;
    reg [pDATA_WIDTH-1:0] stream_out_data_cnt_r;
    wire [pDATA_WIDTH-1:0] stream_out_data_cnt_w;
    //--------------------------------------------------------------AXI LITE SIGNAL--------------------------------------------------------------
    reg arready_r;
    reg rvalid_r;
    wire arready_w;
    wire rvalid_w;
    wire rready_w;
    reg rready_r;
    wire [pADDR_WIDTH-1:0] read_addr_w;
    reg [pADDR_WIDTH-1:0] read_addr_r;
    reg [pADDR_WIDTH-1:0] awaddr_r;
    reg awready_r;
    reg wready_r;
    wire awready_w;
    wire wready_w;
    reg[1:0] lite_sel_r;
    wire[1:0] lite_sel_w;
    reg [1:0] data_transmitted_r;
    wire [1:0] data_transmitted_w;
    wire [pADDR_WIDTH-1:0] awaddr_w;
    reg [pDATA_WIDTH-1:0] wdata_r;
    wire [pDATA_WIDTH-1:0] wdata_w;
//=================================================================================ENGINE FSM=====================================================================
    always@(*)begin 
      case(ap_state)
        CONF: begin
          if(ap_start) ap_state_nxt = READ;
          else ap_state_nxt = CONF;
        end
        READ: begin
          if(stream_out_data_cnt_r == DATA_LENGTH) ap_state_nxt = DONE;
          else if(dataRAM_read_cnt_r == (TAP_NUM-2'd2)) ap_state_nxt = WRITE;
          else ap_state_nxt = READ;
        end
        WRITE: begin
          if(ss_tvalid & ss_tready) ap_state_nxt = WRITE_RE;
          else ap_state_nxt = WRITE;
        end
        WRITE_RE: begin
          if(sm_tvalid & sm_tready) ap_state_nxt = READ;
          else ap_state_nxt = WRITE_RE;
        end
        DONE: begin
          if(ap_idle) ap_state_nxt = CONF;
          else ap_state_nxt = DONE;
        end
      endcase
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        ap_state <= CONF;
      end else begin
        ap_state <= ap_state_nxt;
      end
    end
//==========================================================================Configure Register==========================================================
    assign DATA_LENGTH = config_reg_r[12'h10>>2];
    assign TAP_NUM = config_reg_r[12'h14>>2];
    assign ap_ctrl = config_reg_r[0][2:0];
    assign ap_idle = config_reg_r[0][2];
    assign ap_done = config_reg_r[0][1];
    assign ap_start = config_reg_r[0][0];
    assign done_read = ap_done & (read_addr_r == 0) & (rvalid & rready);
    wire a, b;
    assign a = (awaddr_r>>2) == 0;
    // assign b = (i == (awaddr_r >> 2));

    assign config_en = (awaddr_r <= Config_end) & (&lite_sel_r);
    
    always@(*)begin // Writing configure register 
      case(ap_state)
        CONF: begin
        //   for(i=0;i<((Config_end>>2));i=i+1)begin
        //     if(config_en)begin
        //       if(a) config_reg_w[0] = {wdata_r[pDATA_WIDTH-1:3], 3'b001};// Be careful with ap_ctrl signal
        //       else if(b) config_reg_w[i] = wdata_r;
        //       else config_reg_w[i] = config_reg_r[i];
        //     end
          if(config_en)begin
            if(a)begin
              config_reg_w[0] = wdata_r;
              for(i=1;i<(Config_end>>2);i=i+1) config_reg_w[i] = config_reg_r[i];
            end else begin
              config_reg_w[(awaddr_r>>2)] = wdata_r;
              for(i=0;i<awaddr_r>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
              for(i=(awaddr_r>>2)+1;i<(Config_end)>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
            end
          end else begin
            for(i=0;i<Config_end>>2;i=i+1) config_reg_w[i] = config_reg_r[i];
          end
        end
        READ: begin
          if(sm_tlast)begin
            config_reg_w[0] = {config_reg_r[0][pDATA_WIDTH-1:3], 3'b010};
            for(i=1;i<(Config_end>>2);i=i+1) config_reg_w[i] = config_reg_r[i];
          end else begin
            for(i=0;i<(Config_end>>2);i=i+1) config_reg_w[i] = config_reg_r[i];
          end
        end
        DONE: begin
          if(done_read)begin
            config_reg_w[0] = {config_reg_r[0][pDATA_WIDTH-1:3], 3'b100};
            for(i=1;i<(Config_end>>2);i=i+1) config_reg_w[i] = config_reg_r[i];
          end else begin
            for(i=0;i<(Config_end>>2);i=i+1) config_reg_w[i] = config_reg_r[i];
          end
        end
        default: begin
          for(i=0;i<(Config_end>>2);i=i+1) config_reg_w[i] = config_reg_r[i];
        end
      endcase
    end

    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        for(i=0;i<((Config_end>>2));i=i+1) begin
          if(i > 0) config_reg_r[i] <= 0;
          else config_reg_r[12'h00] <= 3'b100;
        end
      end
      else begin
        for(i=0;i<((Config_end>>2));i=i+1) config_reg_r[i] <= config_reg_w[i];
      end
    end
//========================================================================Tap Address===========================================================================
    always@(*)begin // pointer of tap RAM
      case(ap_state)
        CONF: begin
          if(ap_start) ptr_tapRAM_w = 5'd3;
          else if((awaddr_r >= 12'h80) & config_en) ptr_tapRAM_w = ptr_tapRAM_r + 5'b1;
          else ptr_tapRAM_w = ptr_tapRAM_r;
        end
        READ: begin
          if(dataRAM_read_cnt_r == (TAP_NUM-2)) ptr_tapRAM_w = 5'd0;
          else ptr_tapRAM_w = ptr_tapRAM_r + 5'b1;
        end
        WRITE: begin
          if(ss_tvalid & ss_tready) ptr_tapRAM_w = 5'd1;
          else ptr_tapRAM_w = ptr_tapRAM_r;
        end
        WRITE_RE: begin
          if(sm_tvalid & sm_tready) ptr_tapRAM_w = 5'd3;
          else ptr_tapRAM_w = ptr_tapRAM_r;
        end
        DONE: ptr_tapRAM_w = 0;
        default: ptr_tapRAM_w = ptr_tapRAM_r + 5'b1;
      endcase
    end
    always@(*)begin // tap address for next use
      case(ap_state)
        CONF: begin
          if(ap_start) tap_addr_w = 12'h2 << 2;
          else tap_addr_w = ptr_tapRAM_r << 2;
        end
        READ: begin
          if(dataRAM_read_cnt_r == (TAP_NUM-2)) tap_addr_w = 0;
          else tap_addr_w = ptr_tapRAM_r << 2;
        end
        WRITE: begin
          if(ss_tvalid & ss_tready) tap_addr_w = 5'h1 << 2;
          else tap_addr_w = ptr_tapRAM_r << 2;
        end
        WRITE_RE: begin
          if(sm_tvalid & sm_tready) tap_addr_w = 5'h2 << 2;
          else tap_addr_w = ptr_tapRAM_r << 2;
        end
        default: tap_addr_w = ptr_tapRAM_r << 2;
      endcase
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        ptr_tapRAM_r <= 0;
      end else begin
        ptr_tapRAM_r <= ptr_tapRAM_w;
      end
    end
//=======================================================================Tap RAM===========================================================================
    assign tap_we_w = &lite_sel_r & (awaddr_r >= Param_Start);
    assign tap_WE = {4{tap_we_w}};// write enable has 4 bits
    assign tap_EN = 1;
    assign tap_Di = wdata_r;
    assign tap_A = tap_addr_w;
    
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        tap_we_r <= 0;
      end else begin
        tap_we_r <= tap_we_w;
      end
    end
//======================================================================Data address=======================================================================
    wire [4:0] data_read_addr;
    wire [4:0] data_read_addr_2;
    assign data_read_addr = (ptr_dataRAM_write_r + dataRAM_read_cnt_r + 5'd2); // to make overflow work
    assign data_read_addr_2 = (sm_tvalid & sm_tready) ? ptr_dataRAM_write_r + 5'd2 : ptr_dataRAM_write_r + 5'd1; // has to be the same as write state to prevent stall

    always@(*)begin
      case(ap_state)
        CONF: begin
          if(ap_start) ptr_dataRAM_write_w = 5'd31;
          else ptr_dataRAM_write_w = ptr_dataRAM_write_r + 5'b1;
        end
        WRITE: begin
          if(ss_tvalid & ss_tready) ptr_dataRAM_write_w = ptr_dataRAM_write_r - 5'b1;
          else ptr_dataRAM_write_w = ptr_dataRAM_write_r;
        end
        DONE: ptr_dataRAM_write_w = 0;
        default: ptr_dataRAM_write_w = ptr_dataRAM_write_r;
      endcase
    end
    always@(*)begin
      case(ap_state)
        READ: dataRAM_read_cnt_w = dataRAM_read_cnt_r + 5'b1;
        default: dataRAM_read_cnt_w = 5'b1;
      endcase
    end
    always@(*)begin
      case(ap_state)
        CONF: begin
          if(ap_start) data_addr_w = 5'd31 << 2;
          else data_addr_w = ptr_dataRAM_write_r << 2;
        end
        READ: begin
          if(dataRAM_read_cnt_r == (TAP_NUM-2)) data_addr_w = ptr_dataRAM_write_r << 2;
          else data_addr_w = data_read_addr << 2;
        end
        WRITE: data_addr_w = ptr_dataRAM_write_r << 2;
        WRITE_RE: data_addr_w = data_read_addr_2 << 2;
        default: data_addr_w = ptr_dataRAM_write_r << 2;
      endcase
    end
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        ptr_dataRAM_write_r <= 0;
        dataRAM_read_cnt_r <= 0;
      end else begin
        ptr_dataRAM_write_r <= ptr_dataRAM_write_w;
        dataRAM_read_cnt_r <= dataRAM_read_cnt_w;
      end
    end
//=====================================================================Data RAM=============================================================================
    assign data_WE = {4{((ap_state == WRITE) & (ss_tvalid & ss_tready)) | (ap_state == CONF)}};
    assign data_EN = 1;
    assign data_A = data_addr_w;
    assign data_Di = (ap_state == CONF) ? 0 : ss_tdata; // Configure state initialize
//=====================================================================Multiplier============================================================================
    assign data_mul = (ap_state == WRITE) ? $signed(tap_Do * ss_tdata) : $signed(tap_Do * data_Do);
//=====================================================================Accumulator===========================================================================
    assign stream_out_data_cnt_w = (ap_done) ? 0 : stream_out_data_cnt_r + (sm_tvalid & sm_tready);

    always@(*)begin
      case(ap_state)
        CONF: accum_reg_w = 0;
        READ: accum_reg_w = accum_reg_r + data_mul; 
        WRITE: begin
          if(ss_tvalid & ss_tready) accum_reg_w = accum_reg_r + data_mul;
          else accum_reg_w = accum_reg_r;
        end
        WRITE_RE: begin
          if(sm_tvalid & sm_tready) accum_reg_w = data_mul;
          else accum_reg_w = accum_reg_r;
        end
        DONE: accum_reg_w = 0;
        default: accum_reg_w = accum_reg_r + data_mul;
      endcase
    end
    
    always@(posedge axis_clk or negedge axis_rst_n)begin
      if(~axis_rst_n)begin
        accum_reg_r <= 0;
        stream_out_data_cnt_r <= 0;
      end else begin
        accum_reg_r <= accum_reg_w;
        stream_out_data_cnt_r <= stream_out_data_cnt_w;
      end
    end
//====================================================================AXI Lite================================================================================
    assign rready_w = rready;

    assign awready = awready_r;
    assign wready = wready_r;

    assign arready = arready_r;
    assign rvalid = rvalid_r;

    assign rdata = config_reg_r[read_addr_r>>2]; 
    assign arready_w = ((ap_state == CONF)|(ap_state == DONE)|((data_transmitted_r == 2'b10)|(data_transmitted_r == 2'b01))) ? ((arvalid & arready_r) ? (rvalid_r & rready_r) : 1'b1) : 0;// Only accept read when configuring or Done
    assign rvalid_w = ((ap_state == CONF)|(ap_state == DONE)|((data_transmitted_r == 2'b10)|(data_transmitted_r == 2'b01))) ? (rvalid_r ? (rready_r ? (arvalid & arready_r) : 1'b1) : (arvalid & arready_r)) : 0; // Confirm a data successfully transmitted before controll with state
    assign data_transmitted_w = (data_transmitted_r == 2'b11) ? 2'b00 : {(arvalid & arready) | data_transmitted_r[1], (rvalid & rready) | data_transmitted_r[0]};

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
        data_transmitted_r <= 0;
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
        data_transmitted_r <= data_transmitted_w;
      end
    end
//--------------------AXI Stream interface---------------------
    //---------------AXI Stream interface(x[n])--------------
    assign ss_tready = (ap_state == WRITE);
        
    //--------------AXI Stream interface(y[n])---------------
    assign sm_tvalid = (ap_state == WRITE_RE); // Since one cycle latency, the next state of WRITE state
    assign sm_tlast = (stream_out_data_cnt_r == DATA_LENGTH); // buffer clear and last data transferred
    assign sm_tdata = accum_reg_r[pDATA_WIDTH-1:0]; // output last 32 bit of data  


endmodule