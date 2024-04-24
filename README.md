# Routing-algorithm-for-NOC 
The integration of over one hundred IP cores on a single chip has made the modern System of Chips (SoC) more complex. To separate communication and computing, the silicon industries are promoting the notion of ”Network on Chip.” For Network-on-Chip, Router is the fundamental component. The goal is to create and deploy an adaptable Network-on-Chip router that includes multiple architectural stages, supports virtual channels, and is thoroughly verified using testbenches.The aim is to provide and evaluate adaptive routing algorithm, which must be able dynamically modify its routing decisions in response to changes in network traffic and topology i.e., effective, scalable, and can accommodate a large number of processing parts while also reducing latency and energy.A network-on-chip (NoC) that permits effective communication between the various processor units (PEs) in the MPSoC is the explicit goal of the hardware implementation of the RISC-V based NoC-MPSoC architecture with an adaptive routing algorithm. 
The primary objective is to design, develop, and deploy an adaptable Network-on-Chip router capable of efficiently managing communication between multiple processing elements within an MPSoC. This router will include multiple architectural stages and support features such as virtual channels to enhance communication efficiency.
The motivation behind this endeavor is to overcome the limitations of traditional bus-based communication architectures in MPSoCs. By adopting a NoC approach, the goal is to improve scalability, reduce latency, and conserve energy by separating communication and computing functionalities. This separation allows for more flexible and efficient communication paths, leading to improved overall system performance.

//Verilog Code 

package noc_params;

localparam MESH_SIZE_X = 5;
localparam MESH_SIZE_Y = 5;

localparam DEST_ADDR_SIZE_X = $clog2(MESH_SIZE_X);
localparam DEST_ADDR_SIZE_Y = $clog2(MESH_SIZE_Y);

localparam VC_NUM = 2;
localparam VC_SIZE = $clog2(VC_NUM);

localparam HEAD_PAYLOAD_SIZE = 16;

localparam FLIT_DATA_SIZE = DEST_ADDR_SIZE_X + DEST_ADDR_SIZE_Y + HEAD_PAYLOAD_SIZE;

typedef enum logic[2:0] {LOCAL, NORTH, SOUTH, WEST, EAST} port_t;
localparam PORT_NUM = 5;
localparam PORT_SIZE = $clog2(PORT_NUM);

typedef enum logic[1:0] {HEAD, BODY, TAIL, HEADTAIL} flit_label_t;

typedef struct packed {
    logic [DEST_ADDR_SIZE_X-1:0] x_dest;
    logic [DEST_ADDR_SIZE_Y-1:0] y_dest;
    logic [HEAD_PAYLOAD_SIZE-1:0] headpl;
} headdatat;

typedef struct packed {
    flit_label_t flit_label;
    logic [VC_SIZE-1:0] vcid;
    union packed {
        headdatat headdat;
        logic [FLIT_DATA_SIZE-1:0] btpl;
    } data;
} flitt;

typedef struct packed {
    flit_label_t flit_label;
    union packed {
        headdatat headdata;
        logic [FLIT_DATA_SIZE-1:0] btpl;
    } data;
} flitnovct;

endpackage

// nodelink
import noc_params::*;

module nodelink (
    router2router.upstream_router_if up,
    router2router.downstream_router_if down,
    // upstream connections
    input flitt data_i,
    input is_valid_i,
    output logic[VC_NUM-1:0] is_on_off_o,
    output logic[VC_NUM-1:0] is_allocatable_o,
    // downstream connections
    output flitt data_o,
    output logic is_valid_o,
    input [VC_NUM-1:0] is_on_off_i,
    input [VC_NUM-1:0] is_allocatable_i
);

always_comb begin
    up.data = data_i;
    up.is_valid = is_valid_i;
    is_on_off_o = up.is_on_off;
    is_allocatable_o = up.is_allocatable;
    data_o = down.data;
    is_valid_o = down.is_valid;
    down.is_on_off = is_on_off_i;
    down.is_allocatable = is_allocatable_i;
end

endmodule

// routerlink
module routerlink (
    router2router.upstream_router_if up,
    router2router.downstream_router_if down
);

always_comb begin
    up.data = down.data;
    up.is_valid = down.is_valid;
    down.is_on_off = up.is_on_off;
    down.is_allocatable = up.is_allocatable;
endmodule

// mesh
import nocparams::*;

module mesh #(
    parameter BUFFER_SIZE = 8,
    parameter MESH_SIZE_X = 2,
    parameter MESH_SIZE_Y = 3
) (
    input clk,
    input rst,
    output logic [VC_NUM-1:0] error_o[MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0][PORT_NUM-1:0],
    // connections to all local Router interfaces
    output flit_t[MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0] data_o,
    output logic [MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0] is_valid_o,
    input [MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0][VC_NUM-1:0] is_onoff_i,
    input [MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0][VC_NUM-1:0] is_allocatable_i,
    input flit_t[MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0] data_i,
    input [MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0] is_valid_i,
    output logic [MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0][VC_NUM-1:0] is_onoff_o,
    output logic [MESH_SIZE_X-1:0][MESH_SIZE_Y-1:0][VC_NUM-1:0] is_allocatable_o
);

genvar row, col;
generate
    for (row = 0; row < MESH_SIZE_Y; row++)
    begin : mesh_row
        for (col = 0; col < MESH_SIZE_X; col++)
        begin : mesh_col
            // interfaces instantiation
            router2router_localup();
            router2router_northup();
            router2router_southup();
            router2router_westup();
            router2router_eastup();
            router2router_localdown();
            router2router_northdown();
            router2router_southdown();
            router2router_westdown();
            router2router_eastdown();
            // router instantiation
            router #(
                .BUFFER_SIZE(BUFFER_SIZE),
                .X_CURRENT(col),
                .Y_CURRENT(row)
            )
            router_inst (
                .clk(clk),
                .rst(rst),
                // upstream interfaces connections
                .router_if_localup(localup),
router_if_northup(northup),
    .router_if_southup(southup),
    .router_if_westup(westup),
    .router_if_eastup(eastup),
    // downstream interfaces connections
    .router_if_localdown(localdown),
    .router_if_northdown(northdown),
    .router_if_southdown(southdown),
    .router_if_westdown(westdown),
    .router_if_eastdown(eastdown),
    .error_o(error_o[col][row])
);

end
end

for (row = 0; row < MESH_SIZE_Y - 1; row++)
begin : vertical_links_row
    for (col = 0; col < MESH_SIZE_X; col++)
    begin : vertical_links_col
        router_link link_one (
            .router_if_up(mesh_row[row].mesh_col[col].southdown),
            .router_if_down(mesh_row[row + 1].mesh_col[col].northup)
        );
        router_link link_two (
            .router_if_up(mesh_row[row + 1].mesh_col[col].northdown),
            .router_if_down(mesh_row[row].mesh_col[col].southup)
        );
    end
end

for (row = 0; row < MESH_SIZE_Y; row++)
begin : horizontal_links_row
    for (col = 0; col < MESH_SIZE_X - 1; col++)
    begin : horizontal_links_col
        router_link link_one (
            .router_if_up(mesh_row[row].mesh_col[col].eastdown),
            .router_if_down(mesh_row[row].mesh_col[col + 1].westup)
        );
        router_link link_two (
            .router_if_up(mesh_row[row].mesh_col[col + 1].westdown),
            .router_if_down(mesh_row[row].mesh_col[col].eastup)
        );
    end
end
for ( row =0; row<MESH SIZE_Y ; row ++)
begin : nodeconnection_row
    for (col = 0; col < MESH_SIZE_X; col++)
    begin : nodeconnection_col
        node_link node_link (
            .router_if_up(mesh_row[row].mesh_col[col].localdown),
            .router_if_down(mesh_row[row].mesh_col[col].localup),
            .data_i(data_i[col][row]),
            .is_valid_i(is_valid_i[col][row]),
            .is_onoff_o(is_onoff_o[col][row]),
            .is_allocatable_o(is_allocatable_o[col][row]),
            .data_o(data_o[col][row]),
            .is_valid_o(is_valid_o[col][row]),
            .is_onoff_i(is_onoff_i[col][row]),
            .is_allocatable_i(is_allocatable_i[col][row])
        );
    end
end

endgenerate

endmodule
// Router
import nocparams::*;

module router #(
    parameter BUFFER_SIZE = 8,
    parameter X_CURRENT = MESH_SIZE_X / 2,
    parameter Y_CURRENT = MESH_SIZE_Y / 2
) (
    input clk,
    input rst,
    router2router.upstream_router_if_localup,
    router2router.upstream_router_if_northup,
    router2router.upstream_router_if_southup,
    router2router.upstream_router_if_westup,
    router2router.upstream_router_if_eastup,
    router2router.downstream_router_if_localdown,
    router2router.downstream_router_if_northdown,
    router2router.downstream_router_if_southdown,
    router2router.downstream_router_if_westdown,
    router2router.downstream_router_if_eastdown,
    output logic [VC_NUM-1:0] error_o[PORT_NUM-1:0]
);

// connections from upstream
flit_t data_out[PORT_NUM-1:0];
logic [PORT_NUM-1:0] is_valid_out;
logic [PORT_NUM-1:0][VC_NUM-1:0] is_onoff_in;
logic [PORT_NUM-1:0][VC_NUM-1:0] is_allocatable_in;

// connections from downstream
flit_t data_in[PORT_NUM-1:0];
logic is_valid_in[PORT_NUM-1:0];
logic [VC_NUM-1:0] is_onoff_out[PORT_NUM-1:0];
logic [VC_NUM-1:0] is_allocatable_out[PORT_NUM-1:0];

always_comb begin
    router_if_localup.data = data_out[LOCAL];
    router_if_northup.data = data_out[NORTH];
    router_if_southup.data = data_out[SOUTH];
    router_if_westup.data = data_out[WEST];
    router_if_eastup.data = data_out[EAST];
    router_if_localup.is_valid = is_valid_out[LOCAL];
    router_if_northup.is_valid = is_valid_out[NORTH];
    router_if_southup.is_valid = is_valid_out[SOUTH];
    router_if_westup.is_valid = is_valid_out[WEST];
    router_if_eastup.is_valid = is_valid_out[EAST];
    is_onoff_in[LOCAL] = router_if_localup.is_onoff;
    is_onoff_in[NORTH] = router_if_northup.is_onoff;
    is_onoff_in[SOUTH] = router_if_southup.is_onoff;
    is_onoff_in[WEST] = router_if_westup.is_onoff;
    is_onoff_in[EAST] = router_if_eastup.is_onoff;
    is_allocatable_in[LOCAL] = router_if_localup.is_allocatable;
    is_allocatable_in[NORTH] = router_if_northup.is_allocatable;
    is_allocatable_in[SOUTH] = router_if_southup.is_allocatable;
    is_allocatable_in[WEST] = router_if_westup.is_allocatable;
    is_allocatable_in[EAST] = router_if_eastup.is_allocatable;
    data_in[LOCAL] = router_if_localdown.data;
    data_in[NORTH] = router_if_northdown.data;
    data_in[SOUTH] = router_if_southdown.data;
    data_in[WEST] = router_if_westdown.data;
    data_in[EAST] = router_if_eastdown.data;
    is_valid_in[LOCAL] = router_if_localdown.is_valid;
    is_valid_in[NORTH] = router_if_northdown.is_valid;
    is_valid_in[SOUTH] = router_if_southdown.is_valid;
    is_valid_in[WEST] = router_if_westdown.is_valid;
    is_valid_in[EAST] = router_if_eastdown.is_valid;
    router_if_localdown.is_onoff = is_onoff_out[LOCAL];
    router_if_northdown.is_onoff = is_onoff_out[NORTH];
    router_if_southdown.is_onoff = is_onoff_out[SOUTH];
    router_if_westdown.is_onoff = is_onoff_out[WEST];
    router_if_eastdown.is_onoff = is_onoff_out[EAST];
    router_if_localdown.is_allocatable = is_allocatable_out[LOCAL];
    router_if_northdown.is_allocatable = is_allocatable_out[NORTH];
    router_if_southdown.is_allocatable = is_allocatable_out[SOUTH];
    router_if_westdown.is_allocatable = is_allocatable_out[WEST];
    router_if_eastdown.is_allocatable = is_allocatable_out[EAST];
end
input block2crossbarib2xbarif ();
input block2switchallocatorib2saif ();
input block2vcallibratorib2vaif ();
switchallocator2crossbarsa2xbarif ();
input block #(
    .BUFFER_SIZE(BUFFER_SIZE),
    .X_CURRENT(X_CURRENT),
    .Y_CURRENT(Y_CURRENT)
)
input_block (
    .rst(rst),
    .clk(clk),
    .data_in(data_in),
    .valid_flit_in(is_valid_in),
    .crossbar_if(ib2xbarif),
    .sa_if(ib2saif),
    .va_if(ib2vaif),
    .onoff_out(is_onoff_out),
    .vc_allocatable_out(is_allocatable_out),
    .error_out(error_o)
);

crossbar #() crossbar (
    .ib_if(ib2xbarif),
    .sa_if(sa2xbarif),
    .data_out(data_out)
);

switchallocator #() switchallocator (
    .rst(rst),
    .clk(clk),
    .onoff_in(is_onoff_in),
    .ib_if(ib2saif),
    .xbar_if(sa2xbarif),
    .valid_flit_out(is_valid_out)
);

vcallibrator #() vcallibrator (
    .rst(rst),
    .clk(clk),
    .idle_downstream_vci(is_allocatable_in),
    .ib_if(ib2vaif)
);
 endmodule



// circular buffer
import nocparams::*;

module circularbuffer #(
    parameter BUFFER_SIZE = 8
) (
    input flit novct data_i,
    input read_i,
    input write_i,
    input rst,
    input clk,
    output flit novct data_o,
    output logic is_full_o,
    output logic is_empty_o,
    output logic onoff_o
);

localparam ONOFF_LATENCY = 2;
localparam [31:0] POINTER_SIZE = $clog2(BUFFER_SIZE);

flit novct memory[BUFFER_SIZE - 1:0];

logic [POINTER_SIZE - 1:0] read_ptr;
logic [POINTER_SIZE - 1:0] write_ptr;

logic [POINTER_SIZE - 1:0] read_ptr_next;
logic [POINTER_SIZE - 1:0] write_ptr_next;
logic is_full_next;
logic is_empty_next;
logic onoff_next;

logic [POINTER_SIZE:0] num_flits;
logic [POINTER_SIZE:0] num_flits_next;

always_ff @(posedge clk or posedge rst)
begin
    if (rst)
    begin
        read_ptr <= 0;
        write_ptr <= 0;
        num_flits <= 0;
        is_full_o <= 0;
        is_empty_o <= 1;
        onoff_o <= 1;
    end
    else
    begin
        read_ptr <= read_ptr_next;
        write_ptr <= write_ptr_next;
        num_flits <= num_flits_next;
        is_full_o <= is_full_next;
        is_empty_o <= is_empty_next;
        onoff_o <= onoff_next;
        if ((~read_i & write_i & ~is_full_o) | (read_i & write_i))
            memory[write_ptr] <= data_i;
end

always_comb
begin
    data_o = memory[read_ptr];
    unique if (read_i & ~write_i & ~is_empty_o)
    begin : read_not_empty
        read_ptr_next = increase_ptr(read_ptr);
        write_ptr_next = write_ptr;
        is_full_next = 0;
        update_empty_on_read();
        num_flits_next = num_flits - 1;
    end
    else if (~read_i & write_i & ~is_full_o)
    begin : write_not_full
        read_ptr_next = read_ptr;
        write_ptr_next = increase_ptr(write_ptr);
        update_full_on_write();
        is_empty_next = 0;
        num_flits_next = num_flits + 1;
    end
    else if (read_i & write_i & ~is_empty_o)
    begin : read_write_not_empty
        read_ptr_next = increase_ptr(read_ptr);
        write_ptr_next = increase_ptr(write_ptr);
        is_full_next = is_full_o;
        is_empty_next = is_empty_o;
        num_flits_next = num_flits;
    end
    else
    begin : do_nothing
        read_ptr_next = read_ptr;
        write_ptr_next = write_ptr;
        is_full_next = is_full_o;
        is_empty_next = is_empty_o;
        num_flits_next = num_flits;
    end
    begin : update_onoff_flag
        unique if (num_flits > num_flits_next && num_flits_next < ONOFF_LATENCY)
            onoff_next = 1;
        else if (num_flits < num_flits_next && num_flits_next > BUFFER_SIZE - ONOFF_LATENCY)
            onoff_next = 0;
        else
            onoff_next = onoff_o;
    end
end

function logic[POINTER_SIZE - 1:0] increase_ptr(input logic[POINTER_SIZE - 1:0] ptr);
    if (ptr == BUFFER_SIZE - 1)
        increase_ptr = 0;
    else
        increase_ptr = ptr + 1;
endfunction

function void update_empty_on_read();
    if (read_ptr_next == write_ptr)
        is_empty_next = 1;
    else
        is_empty_next = 0;
endfunction

function void update_full_on_write();
    if (write_ptr_next == read_ptr)
        is_full_next = 1;
    else
        is_full_next = 0;
endfunction

endmodule

// input block
import noc_params::*;

module input_block #(
    parameter PORT_NUM = 5,
    parameter BUFFER_SIZE = 8,
    parameter X_CURRENT = MESH_SIZE_X / 2,
    parameter Y_CURRENT = MESH_SIZE_Y / 2
) (
    input flit_t data_i[PORT_NUM - 1:0],
    input valid_flit_i[PORT_NUM - 1:0],
    input rst,
    input clk,
    input block2crossbar.input_block_crossbar_if,
    input block2switchallocator.input_block_sa_if,
    input block2vcallocator.input_block_va_if,
    output logic[VC_NUM - 1:0] onoff_o[PORT_NUM - 1:0],
    output logic[VC_NUM - 1:0] vcallable_o[PORT_NUM - 1:0],
    output logic[VC_NUM - 1:0] error_o[PORT_NUM - 1:0]
);

logic[VC_NUM - 1:0] is_full[PORT_NUM - 1:0];
logic[VC_NUM - 1:0] is_empty[PORT_NUM - 1:0];
port_t[VC_NUM - 1:0] out_port[PORT_NUM - 1:0];

assign vcallocator.out_port = out_port;
assign switchallocator.out_port = out_port;

genvar ip;
generate
    for (ip = 0; ip < PORT_NUM; ip++)
    begin : generate_input_ports
        input_port #(
            .BUFFER_SIZE(BUFFER_SIZE),
            .X_CURRENT(X_CURRENT),
            .Y_CURRENT(Y_CURRENT)
        )
        input_port (
            .data_i(data_i[ip]),
            .valid_flit_i(valid_flit_i[ip]),
            .rst(rst),
                .clk(clk),
                .sa_sel_vci(sa_if.vc_sel[ip]),
                .va_new_vci(va_if.vc_new[ip]),
                .va_valid_i(va_if.vc_valid[ip]),
                .sa_valid_i(sa_if.valid_sel[ip]),
                .xb_flit_o(crossbar_if.flit[ip]),
                .is_onoff_o(onoff_o[ip]),
                .is_allocatable_vco(vc_allocatable_o[ip]),
                .va_request_o(va_if.vc_request[ip]),
                .sa_request_o(sa_if.switch_request[ip]),
                .sa_downstream_vco(sa_if.downstream_vc[ip]),
                .out_port_o(out_port[ip]),
                .is_full_o(is_full[ip]),
                .is_empty_o(is_empty[ip]),
                .error_o(error_o[ip])
            );
        end
    endgenerate
endmodule

// input buffer
import noc_params::*;

module input_buffer #(
    parameter BUFFER_SIZE = 8
) (
    input flit_novc data_i,
    input read_i,
    input write_i,
    input [VC_SIZE - 1:0] vc_new_i,
    input vc_valid_i,
    input port_t out_port_i,
    input rst,
    input clk,
    output flit_novc data_o,
    output logic is_full_o,
    output logic is_empty_o,
    output logic is_onoff_o,
    output port_t out_port_o,
    output logic vc_request_o,
    output logic switch_request_o,
    output logic vc_allocatable_o,
    output logic [VC_SIZE - 1:0] downstream_vc_o,
    output logic error_o
);

enum logic [1:0] {IDLE, VA, SA} ss, ss_next;

logic [VC_SIZE - 1:0] downstream_vc_next;

logic read_cmd, write_cmd;
logic end_packet, end_packet_next;
logic vc_allocatable_next;
logic error_next;

flit_novc read_flit;
port_to_out_port_next;

circular_buffer #(
    .BUFFER_SIZE(BUFFER_SIZE)
)
circular_buffer (
    .data_i(data_i),
    .read_i(read_cmd),
    .write_i(write_cmd),
    .rst(rst),
    .clk(clk),
    .data_o(read_flit),
    .is_full_o(is_full_o),
    .is_empty_o(is_empty_o),
    .onoff_o(onoff_o)
);

always_ff @(posedge clk, posedge rst)
begin
    if (rst)
    begin
        ss <= IDLE;
        out_port_o <= LOCAL;
        downstream_vc_o <= 0;
        end_packet <= 0;
        vc_allocatable_o <= 0;
        error_o <= 0;
    end
    else
    begin
        ss <= ss_next;
        out_port_o <= out_port_next;
        downstream_vc_o <= downstream_vc_next;
        end_packet <= end_packet_next;
        vc_allocatable_o <= vc_allocatable_next;
        error_o <= error_next;
    end
end

always_comb
begin
    data_o.flit_label = read_flit.flit_label;
    data_o.vcid = downstream_vc_o;
    data_o.data = read_flit.data;
    ss_next = ss;
    out_port_next = out_port_o;
    downstream_vc_next = downstream_vc_o;
    read_cmd = 0;
    write_cmd = 0;
    end_packet_next = end_packet;
    error_next = 0;
    vc_request_o = 0;
    switch_request_o = 0;
    vc_allocatable_next = 0;
unique case(ss)
IDLE:
begin
    if ((data_i.flit_label == HEAD | data_i.flit_label == HEADTAIL) & write_i & is_empty_o)
    begin
        ss_next = VA;
        out_port_next = out_port_i;
        write_cmd = 1;
    end
    if (vc_valid_i | read_i | ((data_i.flit_label == BODY | data_i.flit_label == TAIL) & write_i) | ~is_empty_o)
    begin
        error_next = 1;
    end
    if (write_i & data_i.flit_label == HEADTAIL)
    begin
        end_packet_next = 1;
    end
end

VA:
begin
    if (vc_valid_i)
    begin
        ss_next = SA;
        downstream_vc_next = vc_new_i;
    end
    vc_request_o = 1;
    if (write_i & (data_i.flit_label == BODY | data_i.flit_label == TAIL) & ~end_packet)
    begin
        write_cmd = 1;
    end
    if ((write_i & (end_packet | data_i.flit_label == HEAD | data_i.flit_label == HEADTAIL)) | read_i)
    begin
        error_next = 1;
    end
    if (write_i & data_i.flit_label == TAIL)
    begin
        end_packet_next = 1;
    end
end

SA:
begin
    if (read_i & (data_o.flit_label == TAIL | data_o.flit_label == HEADTAIL))
    begin
        ss_next = IDLE;
        vc_allocatable_next = 1;
        end_packet_next = 0;
    end
if (~is_empty_o)
begin
    switch_request_o = 1;
end

read_cmd = read_i;
if (write_i & (data_i.flit_label == BODY | data_i.flit_label == TAIL) & ~end_packet)
begin
    write_cmd = 1;
end

if ((write_i & (end_packet | data_i.flit_label == HEAD | data_i.flit_label == HEADTAIL)) | vc_valid_i)
begin
    error_next = 1;
end

if (write_i & data_i.flit_label == TAIL)
begin
    end_packet_next = 1;
end

default:
begin
    ss_next = IDLE;
    vc_allocatable_next = 1;
    error_next = 1;
    end_packet_next = 0;
end
endcase
end
endmodule
//input port
import noc_params::*;

module input_buffer #(
    parameter BUFFER_SIZE = 8
) (
    input flit_no_vc_t data_i,
    input read_i,
    input write_i,
    input [VC_SIZE - 1 : 0] vc_new_i,
    input vc_valid_i,
    input port_t port_to_out_port_i,
    input rst,
    input clk,
    output flit_t data_o,
    output logic is_full_o,
    output logic is_empty_o,
    output logic on_off_o,
    output port_t port_to_out_port_o,
    output logic vc_request_o,
    output logic switch_request_o,
output logic vc_allocatable_o,
output logic [VC_SIZE - 1 : 0] downstream_vc_o,
output logic error_o
);

enum logic [1:0] {IDLE, VA, SA} ss, ss_next;
logic [VC_SIZE - 1:0] downstream_vc_next;
logic read_cmd, write_cmd;
logic end_packet, end_packet_next;
logic vc_allocatable_next;
logic error_next;
flit_no_vc read_flit;
port_t out_port_next;

circular_buffer #(
    .BUFFER_SIZE(BUFFER_SIZE)
) circular_buffer (
    .data_i(data_i),
    .read_i(read_cmd),
    .write_i(write_cmd),
    .rst(rst),
    .clk(clk),
    .data_o(read_flit),
    .is_full_o(is_full_o),
    .is_empty_o(is_empty_o),
    .on_off_o(on_off_o)
);

always_ff @(posedge clk, posedge rst)
begin
    if (rst)
    begin
        ss <= IDLE;
        out_port_o <= LOCAL;
        downstream_vc_o <= 0;
        end_packet <= 0;
        vc_allocatable_o <= 0;
        error_o <= 0;
    end
    else
    begin
        ss <= ss_next;
        out_port_o <= out_port_next;
        downstream_vc_o <= downstream_vc_next;
        end_packet <= end_packet_next;
        vc_allocatable_o <= vc_allocatable_next;
        error_o <= error_next;
    end
end

always_comb
begin
    data_o.flit_label = read_flit.flit_label;
data_o.vcid = downstream_vc_o;
data_o.data = read_flit.data;

ss_next = ss;
out_port_next = out_port_o;
downstream_vc_next = downstream_vc_o;

read_cmd = 0;
write_cmd = 0;

end_packet_next = end_packet;
error_next = 0;

vc_request_o = 0;
switch_request_o = 0;
vc_allocatable_next = 0;

unique case(ss)
IDLE:
begin
    if ((data_i.flit_label == HEAD | data_i.flit_label == HEADTAIL) & write_i & is_empty_o)
    begin
        ss_next = VA;
        out_port_next = out_port_i;
        write_cmd = 1;
    end
    if (vc_valid_i | read_i | ((data_i.flit_label == BODY | data_i.flit_label == TAIL) & write_i) | ~is_empty_o)
        error_next = 1;
    if (write_i & data_i.flit_label == HEADTAIL)
        end_packet_next = 1;
end

VA:
begin
    if (vc_valid_i)
    begin
        ss_next = SA;
        downstream_vc_next = vc_new_i;
    end
    vc_request_o = 1;
    if (write_i & (data_i.flit_label == BODY | data_i.flit_label == TAIL) & ~end_packet)
        write_cmd = 1;
    if ((write_i & (end_packet | data_i.flit_label == HEAD | data_i.flit_label == HEADTAIL)) | read_i)
        error_next = 1;
    if (write_i & data_i.flit_label == TAIL)
        end_packet_next = 1;
end
if (write_i & data_i.flit_label == TAIL)
begin
    end_packet_next = 1;
end
end

SA:
begin
    if (read_i & (data_o.flit_label == TAIL | data_o.flit_label == HEADTAIL))
    begin
        ss_next = IDLE;
        vc_allocatable_next = 1;
        end_packet_next = 0;
    end
    if (~is_empty_o)
    begin
        switch_request_o = 1;
    end
    read_cmd = read_i;
    if (write_i & (data_i.flit_label == BODY | data_i.flit_label == TAIL) & ~end_packet)
    begin
        write_cmd = 1;
    end
    if ((write_i & (end_packet | data_i.flit_label == HEAD | data_i.flit_label == HEADTAIL)) | vc_valid_i)
    begin
        error_next = 1;
    end
    if (write_i & data_i.flit_label == TAIL)
    begin
        end_packet_next = 1;
    end
end

default:
begin
    ss_next = IDLE;
    vc_allocatable_next = 1;
    error_next = 1;
    end_packet_next = 0;
end

endcase
end

endmodule

// rcunit
import nocparams::*;
module rcunit #(
parameter X_CURRENT = 0,
parameter Y_CURRENT = 0 ,
parameter DEST_ADDR_SIZE_X = 4 ,
parameter DEST_ADDR_SIZE_Y = 4
) (
input logic [DEST_ADDR_SIZE_X-1:0] x_dest_i ,
input logic [DEST_ADDR_SIZE_Y-1:0] y_dest_i ,
output logic port_to_outport_o
) ;

wire signed [DEST_ADDR_SIZE_X-1:0] x_offset ;
wire signed [DEST_ADDR_SIZE_Y-1:0] y_offset ;

assign x_offset = x_dest_i - X_CURRENT ;
assign y_offset = y_dest_i - Y_CURRENT ;

always_comb
begin
    unique if (x_offset < 0)
    begin
        port_to_outport_o = WEST;
    end
    else if (x_offset > 0)
    begin
        port_to_outport_o = EAST ;
    end
    else if (x_offset == 0 & y_offset < 0)
    begin
        port_to_outport_o = NORTH;
    end
    else if (x_offset == 0 & y_offset > 0)
    begin
        port_to_outport_o = SOUTH;
    end
    else
    begin
        port_to_outport_o = LOCAL;
    end
end

endmodule
/ / r o u n d r o b i n a r b i t e r
module r o u n d r o b i n a r b i t e r #(
parameter AGENTS_NUM = 4
) (
input rst ,
input clk ,
input [AGENTS_NUM-1:0] requests_i ,
output logic [AGENTS_NUM-1:0] grants_o
) ;

localparam [31:0] AGENTS_PTR_SIZE = $clog2(AGENTS_NUM) ;

logic [AGENTS_PTR_SIZE-1:0] highest_priority, highest_priority_next ;

always_ff @(posedge clk, posedge rst)
begin
    if (rst)
    begin
        highest_priority <= 0 ;
    end
    else
    begin
        highest_priority <= highest_priority_next ;
    end
end

always_comb
begin
    grants_o = {AGENTS_NUM{1'b0}} ;
    highest_priority_next = highest_priority ;
    for (int i = 0; i < AGENTS_NUM; i = i + 1)
    begin
        if (requests_i[(highest_priority + i) % AGENTS_NUM])
        begin
            grants_o[(highest_priority + i) % AGENTS_NUM] = 1'b1 ;
            highest_priority_next = (highest_priority + i + 1) % AGENTS_NUM;
            break ;
        end
    end
end

endmodule

/ / s e p a r a b l e i n p u t f i r s t a l l o c a t o r
module separable_input_firstallocator #(
parameter VC_NUM = 2
) (
input rst ,
input clk ,
input [PORT_NUM-1:0][VC_NUM-1:0] request_i ,
input port_t[VC_NUM-1:0] outport_i[PORT_NUM-1:0] ,
output logic [PORT_NUM-1:0][VC_NUM-1:0] grant_o
) ;

logic [PORT_NUM-1:0][PORT_NUM-1:0] out_request ;
logic [PORT_NUM-1:0][PORT_NUM-1:0] ip_grant ;
logic [PORT_NUM-1:0][VC_NUM-1:0] vc_grant ;
genvar in_arb ;
generate
for (in_arb = 0; in_arb < PORT_NUM; in_arb++)
begin : generate_inputroundrobinarbiters
    roundrobinarbiter #(
    .AGENTS_NUM(VC_NUM)
    )
    roundrobinarbiter (
    .rst(rst),
    .clk(clk),
    .requests_i(request_i[in_arb]),
    .grants_o(vc_grant[in_arb])
    ) ;
end
endgenerate
genvar out_arb ;
generate
for (out_arb = 0; out_arb < PORT_NUM; out_arb++)
begin : generate_outputroundrobinarbiters
    roundrobinarbiter #(
    .AGENTS_NUM(PORT_NUM)
    )
    roundrobinarbiter (
    .rst(rst),
    .clk(clk),
    .requests_i(out_request[out_arb]),
    .grants_o(ip_grant[out_arb])
    ) ;
end
endgenerate

always_comb
begin
    out_request = {PORT_NUM*PORT_NUM{1'b0}} ;
    grant_o = {PORT_NUM*VC_NUM{1'b0}} ;
    for (int in_port = 0; in_port < PORT_NUM; in_port = in_port + 1)
    begin
        for (int in_vc = 0; in_vc < VC_NUM; in_vc = in_vc + 1)
        begin
            if (vc_grant[in_port][in_vc])
            begin
                out_request[outport_i[in_port][in_vc]][in_port] = 1'b1 ;
                break ;
            end
        end
    end
end

endmodule
/ / s w i t c h a l l o c a t o r
module switchallocator #(
) (
input rst ,
input clk ,
input [PORT NUM-1:0][VC NUM-1:0] onoff_i ,
input block2 switchallocator.switchallocator_ibif ,
switchallocator2crossbar.switchallocator_xbarif ,
output logic [PORT NUM-1:0] validflit_o
) ;

logic [PORT NUM-1:0][VC NUM-1:0] requestcmd ;
logic [PORT NUM-1:0][VC NUM-1:0] grant ;

separable_input_firstallocator #(
.VC NUM(VC NUM)
)
separable_input_firstallocator (
. rst ( rst ) ,
. clk ( clk ) ,
. requests_i ( requestcmd ) ,
. outport_i ( ibif.outport ) ,
. grants_o ( grant )
) ;

always_comb
begin
for ( int port = 0 ; port < PORT NUM ; port = port + 1)
begin
ibif.validsel[port] = 1'b0 ;
validflit_o[port] = 1'b0 ;
ibif.vcsel[port] = {VC SIZE{1'b0}} ;
switchallocator_xbarif.inputvcsel[port] = {PORT SIZE{1'b0}} ;
requestcmd[port] = {VC NUM{1'b0}} ;
end

for ( int upport = 0 ; upport < PORT NUM; upport = upport + 1)
begin
for ( int upvc = 0 ; upvc < VC NUM; upvc = upvc + 1)
begin
if ( ibif.switchrequest[upport][upvc] & onoff_i [ibif.outport[upport][upvc]] [ibif.downstreamvc[upport][upvc]] )
begin
requestcmd[upport][upvc] = 1'b1 ;
end
end
end

for ( int upport = 0 ; upport < PORT NUM; upport = upport + 1)
begin
for ( int upvc = 0 ; upvc < VC NUM; upvc = upvc + 1)
begin
if ( grant[upport][upvc] )
begin
ibif.vcsel[upport] = upvc ;
ibif.validsel[upport] = 1'b1 ;
validflit_o[ibif.outport[upport][upvc]] = 1'b1 ;
switchallocator_xbarif.inputvcsel[ibif.outport[upport][upvc]] = upport ;
end
end
end
end

endmodule

/ / V C a l l o c a t o r
module vcallocator #(
) (
input rst ,
input clk ,
input [PORT NUM-1:0][VC NUM-1:0] idledownstreamvc_i ,
input block2 vcallocator.vcallocator_ibif
) ;

logic [PORT NUM-1:0][VC NUM-1:0] requestcmd ;
logic [PORT NUM-1:0][VC NUM-1:0] grant ;

logic [PORT NUM-1:0][VC NUM-1:0] isavailablevc ,
isavailablevcnext ;

separable_input_firstallocator #(
.VC NUM(VC NUM)
)
separable_input_firstallocator (
. rst ( rst ) ,
. clk ( clk ) ,
. requests_i ( requestcmd ) ,
. outport_i ( ibif.outport ) ,
. grants_o ( grant )
) ;

always_ff @(posedge clk, posedge rst)
begin
if (rst)
begin
isavailablevc <= {PORT NUM*VC NUM{1'b1}} ;
end
else
begin
isavailablevc <= isavailablevcnext ;
end
end

always_comb
begin
isavailablevcnext = isavailablevc ;
for (int upport = 0; upport < PORT NUM; upport = upport + 1)
begin
for (int upvc = 0; upvc < VC NUM; upvc = upvc + 1)
begin
requestcmd[upport][upvc] = 1'b0 ;
ibif.vcvalid[upport][upvc] = 1'b0 ;
ibif.vcnew[upport][upvc] = {VC SIZE{1'bx}} ;
end
end

for (int upport = 0; upport < PORT NUM; upport = upport + 1)
begin
for (int upvc = 0; upvc < VC NUM; upvc = upvc + 1)
begin
if (ibif.vcrequest[upport][upvc] & isavailablevc [ibif.outport[upport][upvc]])
begin
requestcmd[upport][upvc] = 1'b1 ;
end
end
end

for (int upport = 0; upport < PORT NUM; upport = upport + 1)
begin
for (int upvc = 0; upvc < VC NUM; upvc = upvc + 1)
begin
if (grant[upport][upvc])
begin
ibif.vcnew[upport][upvc] = assign_downstreamvc(ibif.outport[upport][upvc]) ;
ibif.vcvalid[upport][upvc] = 1'b1 ;
isavailablevcnext[ibif.outport[upport][upvc]][ibif.vcnew[upport][upvc]] = 1'b0 ;
end
end
end

for (int downport = 0; downport < PORT NUM; downport = downport + 1)
begin
for (int downvc = 0; downvc < VC NUM; downvc = downvc + 1)
begin
if (~isavailablevc[downport][downvc] & idledownstreamvci[downport][downvc])
begin
isavailablevcnext[downport][downvc] = 1'b1 ;
end
end
end

end

function logic[VC SIZE-1:0] assign_downstreamvc(input portt port);
assign_downstreamvc = {VC SIZE{1'bx}} ;
for (int vc = 0; vc < VC NUM; vc = vc + 1)
begin
if (isavailablevc[port][vc])
begin
assign_downstreamvc = vc ;
break ;
end
end
endfunction

endmodule
