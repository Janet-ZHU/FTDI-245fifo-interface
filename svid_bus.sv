`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/03/23 10:15:11
// Design Name: 
// Module Name: svid_module
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module svid_module(
    input logic rst_n, //system reset
    input logic svid_clk,
    input logic svid_dat, //In actually, this is inout type; but, we just to get the data in the svid bus, just using it as input port.
    input logic svid_alert_n,
    output logic itready, //aways is 1, can be verify this bit FIFO is not full
    input logic oclk,
    input logic otready, // extern read request 
    output logic otvalid, // has data in FIFO,if this bit==0. the FIFO is empty
    output logic[7:0] otdata
    //debug signal
    ,
    output logic start,
    output logic[15:0] cnt,
    output logic[15:0] cnt1,
    output logic tran_vrm_flag,
    output logic  end_transaction
    
    );
//logic state;
logic [2:0]  cur_state,nxt_state;
localparam  Idle  = 3'b000;
localparam  S0    = 3'b001;
localparam  S01   = 3'b010;
localparam  S010  = 3'b100;

logic[7:0] fill_data; //usb-data loss the first and second data, should fill two data in beigin.
logic[2:0] fill_data_en;
//logic start;

//logic[15:0] cnt,cnt1;
//logic tran_vrm_flag;
//below is sampled at posedge of svid_clk,driving by processor(master)
logic[3:0] addr;
logic[4:0] cmd;
logic[7:0] cmd_data;
logic      proc_parity_bit;
logic[2:0] end_pattern;//011
logic[1:0] trun_around1;//11
//below is sampled at negedge of svid_clk, driving by slave
logic[1:0] ack;
logic[7:0] vrm_data;
logic      slave_parity_bit;
logic [1:0] turn_around2;//11
logic ready_to_end;
//logic end_transaction;
logic itvalid;
logic[7:0] itdata;


initial  begin cnt<=16'b0;cnt1<=16'b0; itvalid<=0; start<=0;addr=4'b0;cmd<=5'b0;cmd_data<=8'b0;end_pattern<=0;ack<=2'b0;end    //simulation will using this sentence

stream_async_fifo #( // svid bus just 25MHz is slower than FT2232 60MHZ
    .DSIZE        ( 1  ),
    .ASIZE        ( 7  )
)svid_fifo(
        .rst_n       (rst_n),
        .iclk        (svid_clk),
        .itvalid     (itvalid), //input, if svid want wirte data into FIFO, itvalid=1, before this time, data shoule be ready.
        .itready     (itready), //output,if itready==0,FIFO full. if itready==1,FIFO can be write
        .itdata      (itdata),
    
        .oclk        (oclk),
        .otready     (otready), 
        .otvalid     (otvalid),
        .otdata      (otdata)
);

//状态转移
always @(negedge svid_clk or posedge rst_n) begin
   if(~rst_n)
       begin cur_state <= Idle;end
   else begin
        cur_state <= nxt_state; 
   end
end

//次态生成
always@(*)
begin
   if(~rst_n)
        begin
            nxt_state = Idle;
            start = 0;
        end
   else begin   
   if( (start==1) && (end_transaction == 1))begin  // 
        start = 0;
   end
   if(start==0)
   begin
        case(cur_state)
             Idle   : nxt_state = svid_dat ? Idle : S0;
             S0     : nxt_state = svid_dat ? S01  : S0;
             S01    : nxt_state = svid_dat ? Idle : S010;
             S010   : begin nxt_state= Idle; start=1;end
             default: nxt_state <= Idle;
        endcase
        end
   end
end
//send addr,cmd,cmd_data, ack, vmr_data, 5byte to FIFO, because FIFO outclk is 60MHz, is faster than svid_clk, so FIFO always is available.
always @(negedge svid_clk or posedge rst_n) begin
   if(~rst_n)
       begin cnt<=16'b1; tran_vrm_flag <=1'b0; itvalid<=1'b0;addr=4'b0;cmd<=5'b0;cmd_data<=8'b0;end_pattern<=3'b0;ready_to_end<=0;fill_data<=8'b0;
            fill_data_en<=1'b1;
       end
   else begin
        if((start==1)&& (tran_vrm_flag==0)) begin //start is set previous clk, so the data will go out at this clk
            cnt<=cnt+1; //cnt will be transfer cnt+1 after this clk, so this begin end will excute current value.
            if(cnt>=1 && cnt<=4) begin
                addr <= addr + (svid_dat<<(4-cnt));
                if(fill_data_en) begin //fill two 0 can make data not loss
                    if(cnt == 1) begin
                        itdata <= fill_data;
                        itvalid <= 1;
                    end  
                    if(cnt == 3) begin
                        itdata <= fill_data;
                        itvalid <= 0;
                        fill_data_en <= 0;
                    end   
                 end
            end
            if(cnt>=5 && cnt<=9) begin
                cmd <= cmd + (svid_dat<<(9-cnt));
                if(cnt==5) begin
                    itdata <= {4'b0000,addr};//fifo is write in posedge of svid_clk, this module is in negedge of svid_clk
                    itvalid<=1;//  when next posedge svid_clk is arrive, the it data is ready too.
                end
                if(cnt==6) begin
                    itvalid<=0;
                end
            end
            if(cnt>=10 && cnt<=17) begin
                cmd_data <= cmd_data + (svid_dat<<(17-cnt));
                if(cnt==10)begin
                    itdata={3'b000,cmd};//send addr to FIFO
                    itvalid <= 1;
                end
                if(cnt==11)begin
                    itvalid<=0;//cmd wirte finish
                end
            end 
            if(cnt==18) begin
                proc_parity_bit = svid_dat; 
                itdata=cmd_data;//send cmd_data to FIFO
                itvalid <= 1;
            end 
            if(cnt>=19 && cnt<=21) begin
                end_pattern <=end_pattern + (svid_dat<<(21-cnt));
                if(cnt==19)begin
                    itvalid<=0;//cmd_data wirte finish
                end 
            end 
            if(cnt>=22 && cnt<=23) begin  
                trun_around1  <=trun_around1 + (svid_dat<<(23-cnt));
                if(cnt==23)begin
                    tran_vrm_flag <=1; // at this clk, cnt==24
                    cnt<=16'b1; 
                    addr=4'b0;cmd<=5'b0;cmd_data<=8'b0;end_pattern<=3'b0; //must clear to 0, because addr =addr +.., intial value must to be 0                              
                end
            end 
        end
        if(tran_vrm_flag==1) begin
       // In case of  mutil-drive, the data in posedge  should write in here.
//           if(cnt1==1)begin
//                itdata<={4'b0000,addr};//send addr to FIFO
//                itvalid <= 1;
//           end
//           if(cnt1==2) begin
//                itvalid<=0;
//            end
                
            if(cnt1==3)begin
                itdata<={6'b000000,ack};//send ack[1:0] to FIFO
                itvalid <= 1; //this posedge is changed the data, and set the itvalid, and next posedge is write, 
            end
            if(cnt1==4)begin
                itvalid <= 0; //cnt1==4, this posedge clk itvalid is 
            end 
            if(cnt1==11) begin
                itdata<=vrm_data;
                itvalid <= 1; //send vrm_data to FIFO
            end  
            if(cnt1==12)begin
                itvalid <= 0;
                itdata  <= 0;
            end
            if(cnt1==13)begin // need delay 1T, clear  tran_vrm_flag 
                ready_to_end<=1; 
            end
            if(ready_to_end==1) begin
                tran_vrm_flag <=0;
                ready_to_end<=0;
            end
         
        end
   end
end
////posedge sampling
always @(posedge svid_clk or posedge rst_n) 
begin
    if(~rst_n) begin
        cnt1<=1;vrm_data<=8'b0;ack<=2'b0; end_transaction <= 0;
    end
    else begin
         if((start == 0)&&(tran_vrm_flag ==0)) begin end_transaction<=0; end
         if((start == 1)&&(tran_vrm_flag ==1)) begin
             if(cnt1<=13)begin //when cnt1==13, stop +1
                cnt1<=cnt1+1;
             end   
             if(cnt1>=1 && cnt1<=2) begin
                ack <= ack + (svid_dat<<(2-cnt1)); 
             end
             if(cnt1>=3 && cnt1<=10) begin
                vrm_data <= vrm_data + (svid_dat<<(10-cnt1)); 
             end
            if(cnt1==11) begin
                slave_parity_bit =svid_dat; 
             end
            if(cnt1>=12 && cnt1<=13) begin
               turn_around2 = turn_around2 + (svid_dat<<(13-cnt1)); 
            end
            if(cnt1 == 13) begin 
                cnt1<=1;
                end_transaction <= 1;
                vrm_data<=8'b0;ack<=2'b0;
            end
         end
    end    
end

endmodule
