// E9.3 FAT32 READ-ONLY FOUNDATION (integration candidate, not a drop-in top).
// One SD reader, one open file at a time; same clk as sd_card_top.
// Supports FAT32 in a superfloppy VBR or MBR primary partition (types 0B/0C/1B/1C).
// Exact 8.3 upper-case directory names only; optional one-level 8.3 folder.
// Supports fragmented directory and file FAT chains. No FAT RAM, no hidden writes.
// read pulse is held until sd_sec_read_end; consumer MUST take every data_valid byte.
// Does not alter HDMI, PCM cache, or SDRAM arbitration.
module fat32_file_stream #(
    parameter [31:0] TIMEOUT_CYCLES = 32'd100_000_000,
    parameter [15:0] MAX_DIR_CLUSTERS = 16'd4096
)(
    input  wire        clk,
    input  wire        rst,
    input  wire        sd_init_done,
    input  wire        start,                     // one clock, accepted when !busy
    input  wire [87:0] file_name_83,              // exactly 11 bytes e.g. "PIANO   BIN"
    input  wire        use_folder,
    input  wire [87:0] folder_name_83,            // e.g. "IMAGES     "
    input  wire        lookup_only,               // return file_size/found/done without data
    input  wire [31:0] read_limit_bytes,          // 0=all; otherwise up to N file bytes
    output reg         busy,
    output reg         done,                      // one clock on complete file
    output reg         found,                     // one clock after matching directory entry
    output reg  [3:0]  error,                     // sticky until next start, 0 success
    output reg  [31:0] file_size,
    output reg  [7:0]  data_byte,
    output reg         data_valid,                // no backpressure; full 512 byte SD flow
    output reg         data_last,                 // asserted with last valid byte
    output reg  [31:0] data_offset,               // byte offset inside file
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end
);
    localparam [3:0] ST_IDLE      = 4'd0,
                     ST_REQUEST   = 4'd1,
                     ST_WAIT      = 4'd2,
                     ST_ADVANCE   = 4'd3,
                     ST_SETUP     = 4'd4,
                     ST_DIR_INIT  = 4'd5,
                     ST_FILE_INIT = 4'd6,
                     // Split cluster -> LBA into three register-to-register
                     // operations. Fixes candidate_cluster -> read_lba
                     // long combinational carry/barrel-shift path.
                     ST_LBA_SUB   = 4'd7,
                     ST_LBA_SHIFT = 4'd8,
                     ST_LBA_ADD   = 4'd9;
    localparam [2:0] PH_LBA0     = 3'd0,
                     PH_BPB      = 3'd1,
                     PH_DIR      = 3'd2,
                     PH_DIR_FAT  = 3'd3,
                     PH_DATA     = 3'd4,
                     PH_DATA_FAT = 3'd5;
    localparam [3:0] ERR_OK      = 4'd0,
                     ERR_MBR     = 4'd1,
                     ERR_BPB     = 4'd2,
                     ERR_NOFILE  = 4'd3,
                     ERR_CLUSTER = 4'd4,
                     ERR_TIMEOUT = 4'd5,
                     ERR_SECTOR  = 4'd6,
                     ERR_SHORT   = 4'd7,
                     ERR_DIRLOOP = 4'd8;
    reg [3:0] state;
    reg [2:0] phase;
    reg       mounted;
    reg [31:0] volume_lba, fat_lba, data_lba, root_cluster;
    reg [7:0]  spc;
    reg [2:0]  spc_shift;
    reg [31:0] read_lba;
    reg [9:0]  byte_count;
    reg [31:0] timer;

    // BPB parsed directly from sector bytes; no 512-byte RAM is used.
    reg [15:0] bpb_bps, bpb_reserved, bpb_fat16;
    reg [7:0]  bpb_spc, bpb_num_fats;
    reg [31:0] bpb_fat32, bpb_root;
    reg [7:0]  sig510, sig511;
    reg [31:0] mbr_part_lba, mbr_tmp_lba;
    reg        mbr_partition_found, mbr_type_ok;

    // Directory lookup: streaming 32-byte directory entry parser.
    reg [31:0] dir_cluster, candidate_cluster, candidate_size;
    reg [31:0] dir_lo_cluster, dir_hi_cluster, dir_tmp_size;
    reg [7:0]  dir_attribute;
    reg [87:0] name_shift;
    reg        name_match, entry_found, end_of_directory, seek_folder;
    reg [7:0]  cluster_sector_index;
    reg [15:0] dir_cluster_hops;

    // Current file and FAT chain pointer.
    reg [31:0] file_cluster, bytes_remaining, file_position;
    // Registered cluster address pipeline. All cluster references (root,
    // directory match, FAT next link) must enter ST_LBA_SUB through this reg.
    reg [31:0] addr_cluster, addr_delta, addr_scaled;
    reg [31:0] fat_entry;
    reg [8:0]  fat_byte_offset;

    wire bpb_valid = (sig510 == 8'h55) && (sig511 == 8'haa) &&
                     (bpb_bps == 16'd512) &&
                     (bpb_spc != 8'd0) && (bpb_spc <= 8'd128) &&
                     ((bpb_spc & (bpb_spc-8'd1)) == 8'd0) &&
                     (bpb_reserved != 16'd0) &&
                     ((bpb_num_fats == 8'd1) || (bpb_num_fats == 8'd2)) &&
                     (bpb_fat16 == 16'd0) && (bpb_fat32 != 32'd0) &&
                     ((bpb_root & 32'h0fff_ffff) >= 32'd2);
    wire [27:0] next_fat_cluster = fat_entry[27:0];
    wire fat_eoc = (next_fat_cluster >= 28'hffffff8);
    wire fat_valid = (next_fat_cluster >= 28'd2) &&
                     (next_fat_cluster < 28'hffffff0);
    function [2:0] spc_to_shift;
        input [7:0] sectors;
        begin
            case (sectors)
                8'd1: spc_to_shift=3'd0;
                8'd2: spc_to_shift=3'd1;
                8'd4: spc_to_shift=3'd2;
                8'd8: spc_to_shift=3'd3;
                8'd16: spc_to_shift=3'd4;
                8'd32: spc_to_shift=3'd5;
                8'd64: spc_to_shift=3'd6;
                8'd128: spc_to_shift=3'd7;
                default: spc_to_shift=3'd0;
            endcase
        end
    endfunction
    // All SD and metadata operations are serialized. The low-level SD core
    // already returns a one-clock sd_sec_read_end; a full idle gap precedes
    // each next CMD17 to satisfy its S_WAIT_READ_WRITE handshake.
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= ST_IDLE;
            phase <= PH_LBA0;
            mounted <= 1'b0;
            busy <= 1'b0;
            done <= 1'b0;
            found <= 1'b0;
            error <= ERR_OK;
            file_size <= 32'd0;
            data_byte <= 8'd0;
            data_valid <= 1'b0;
            data_last <= 1'b0;
            data_offset <= 32'd0;
            file_position <= 32'd0;
            sd_sec_read <= 1'b0;
            sd_sec_read_addr <= 32'd0;
            read_lba <= 32'd0;
            byte_count <= 10'd0;
            timer <= 32'd0;
            volume_lba <= 32'd0;
            fat_lba <= 32'd0;
            data_lba <= 32'd0;
            root_cluster <= 32'd0;
            spc <= 8'd0;
            spc_shift <= 3'd0;
            bpb_bps <= 16'd0;
            bpb_reserved <= 16'd0;
            bpb_fat16 <= 16'd0;
            bpb_spc <= 8'd0;
            bpb_num_fats <= 8'd0;
            bpb_fat32 <= 32'd0;
            bpb_root <= 32'd0;
            sig510 <= 8'd0;
            sig511 <= 8'd0;
            mbr_part_lba <= 32'd0;
            mbr_tmp_lba <= 32'd0;
            mbr_partition_found <= 1'b0;
            mbr_type_ok <= 1'b0;
            dir_cluster <= 32'd0;
            candidate_cluster <= 32'd0;
            candidate_size <= 32'd0;
            dir_lo_cluster <= 32'd0;
            dir_hi_cluster <= 32'd0;
            dir_tmp_size <= 32'd0;
            dir_attribute <= 8'd0;
            name_shift <= 88'd0;
            name_match <= 1'b0;
            entry_found <= 1'b0;
            end_of_directory <= 1'b0;
            seek_folder <= 1'b0;
            cluster_sector_index <= 8'd0;
            dir_cluster_hops <= 16'd0;
            file_cluster <= 32'd0;
            bytes_remaining <= 32'd0;
            addr_cluster <= 32'd0;
            addr_delta <= 32'd0;
            addr_scaled <= 32'd0;
            fat_entry <= 32'd0;
            fat_byte_offset <= 9'd0;
        end else begin
            done <= 1'b0;
            found <= 1'b0;
            data_valid <= 1'b0;
            data_last <= 1'b0;

            if (!sd_init_done) begin
                // Do not trust mount parameters if the SD core reinitializes.
                mounted <= 1'b0;
                busy <= 1'b0;
                sd_sec_read <= 1'b0;
                state <= ST_IDLE;
            end else begin
                case (state)
                    ST_IDLE: begin
                        sd_sec_read <= 1'b0;
                        if (start) begin
                            busy <= 1'b1;
                            error <= ERR_OK;
                            file_size <= 32'd0;
                            bytes_remaining <= 32'd0;
                            data_offset <= 32'd0;
                            file_position <= 32'd0;
                            entry_found <= 1'b0;
                            end_of_directory <= 1'b0;
                            seek_folder <= use_folder;
                            dir_cluster_hops <= 16'd0;
                            if (mounted) begin
                                state <= ST_DIR_INIT;
                            end else begin
                                mbr_partition_found <= 1'b0;
                                mbr_type_ok <= 1'b0;
                                bpb_bps <= 16'd0;
                                bpb_spc <= 8'd0;
                                bpb_reserved <= 16'd0;
                                bpb_num_fats <= 8'd0;
                                bpb_fat16 <= 16'd0;
                                bpb_fat32 <= 32'd0;
                                bpb_root <= 32'd0;
                                sig510 <= 8'd0;
                                sig511 <= 8'd0;
                                read_lba <= 32'd0;
                                phase <= PH_LBA0;
                                state <= ST_REQUEST;
                            end
                        end
                    end
                    ST_REQUEST: begin
                        sd_sec_read_addr <= read_lba;
                        sd_sec_read <= 1'b1;
                        byte_count <= 10'd0;
                        timer <= 32'd0;
                        if (phase == PH_DIR) begin
                            // Reset per-sector match; sector can contain 16 entries.
                            entry_found <= 1'b0;
                            end_of_directory <= 1'b0;
                        end
                        if ((phase == PH_DIR_FAT) || (phase == PH_DATA_FAT))
                            fat_entry <= 32'd0;
                        state <= ST_WAIT;
                    end
                    ST_WAIT: begin
                        if (sd_sec_read_data_valid) begin
                            byte_count <= byte_count + 10'd1;
                            if ((phase == PH_LBA0) || (phase == PH_BPB)) begin
                                case (byte_count)
                                    10'd11: bpb_bps[7:0] <= sd_sec_read_data;
                                    10'd12: bpb_bps[15:8] <= sd_sec_read_data;
                                    10'd13: bpb_spc <= sd_sec_read_data;
                                    10'd14: bpb_reserved[7:0] <= sd_sec_read_data;
                                    10'd15: bpb_reserved[15:8] <= sd_sec_read_data;
                                    10'd16: bpb_num_fats <= sd_sec_read_data;
                                    10'd22: bpb_fat16[7:0] <= sd_sec_read_data;
                                    10'd23: bpb_fat16[15:8] <= sd_sec_read_data;
                                    10'd36: bpb_fat32[7:0] <= sd_sec_read_data;
                                    10'd37: bpb_fat32[15:8] <= sd_sec_read_data;
                                    10'd38: bpb_fat32[23:16] <= sd_sec_read_data;
                                    10'd39: bpb_fat32[31:24] <= sd_sec_read_data;
                                    10'd44: bpb_root[7:0] <= sd_sec_read_data;
                                    10'd45: bpb_root[15:8] <= sd_sec_read_data;
                                    10'd46: bpb_root[23:16] <= sd_sec_read_data;
                                    10'd47: bpb_root[31:24] <= sd_sec_read_data;
                                    10'd510: sig510 <= sd_sec_read_data;
                                    10'd511: sig511 <= sd_sec_read_data;
                                    default: ;
                                endcase
                                if (phase == PH_LBA0) begin
                                    case (byte_count)
                                        10'd450,10'd466,10'd482,10'd498:
                                            mbr_type_ok <= (sd_sec_read_data == 8'h0b) ||
                                                           (sd_sec_read_data == 8'h0c) ||
                                                           (sd_sec_read_data == 8'h1b) ||
                                                           (sd_sec_read_data == 8'h1c);
                                        10'd454,10'd470,10'd486,10'd502:
                                            mbr_tmp_lba[7:0] <= sd_sec_read_data;
                                        10'd455,10'd471,10'd487,10'd503:
                                            mbr_tmp_lba[15:8] <= sd_sec_read_data;
                                        10'd456,10'd472,10'd488,10'd504:
                                            mbr_tmp_lba[23:16] <= sd_sec_read_data;
                                        10'd457,10'd473,10'd489,10'd505:
                                            begin
                                                mbr_tmp_lba[31:24] <= sd_sec_read_data;
                                                if (mbr_type_ok && !mbr_partition_found) begin
                                                    mbr_part_lba <= {sd_sec_read_data,mbr_tmp_lba[23:0]};
                                                    mbr_partition_found <= 1'b1;
                                                end
                                            end
                                        default: ;
                                    endcase
                                end
                            end
                            if (phase == PH_DIR) begin
                                case (byte_count[4:0])
                                    5'd0: begin
                                        name_shift <= {(seek_folder ? folder_name_83[79:0] : file_name_83[79:0]),8'd0};
                                        name_match <= (sd_sec_read_data == (seek_folder ? folder_name_83[87:80] : file_name_83[87:80])) &&
                                                      (sd_sec_read_data != 8'he5) && (sd_sec_read_data != 8'd0);
                                        if (sd_sec_read_data == 8'd0)
                                            end_of_directory <= 1'b1;
                                    end
                                    5'd1,5'd2,5'd3,5'd4,5'd5,5'd6,5'd7,5'd8,5'd9,5'd10: begin
                                        name_match <= name_match && (sd_sec_read_data == name_shift[87:80]);
                                        name_shift <= {name_shift[79:0],8'd0};
                                    end
                                    5'd11: dir_attribute <= sd_sec_read_data;
                                    5'd20: dir_hi_cluster[7:0] <= sd_sec_read_data;
                                    5'd21: dir_hi_cluster[15:8] <= sd_sec_read_data;
                                    5'd26: dir_lo_cluster[7:0] <= sd_sec_read_data;
                                    5'd27: dir_lo_cluster[15:8] <= sd_sec_read_data;
                                    5'd28: dir_tmp_size[7:0] <= sd_sec_read_data;
                                    5'd29: dir_tmp_size[15:8] <= sd_sec_read_data;
                                    5'd30: dir_tmp_size[23:16] <= sd_sec_read_data;
                                    5'd31: begin
                                        if (!entry_found && !end_of_directory && name_match &&
                                            (dir_attribute != 8'h0f) && !dir_attribute[3] &&
                                            (dir_attribute[4] == seek_folder)) begin
                                            entry_found <= 1'b1;
                                            candidate_cluster <= {dir_hi_cluster[11:0],dir_lo_cluster[15:0]};
                                            candidate_size <= {sd_sec_read_data,dir_tmp_size[23:0]};
                                        end
                                    end
                                    default: ;
                                endcase
                            end
                            if ((phase == PH_DIR_FAT) || (phase == PH_DATA_FAT)) begin
                                if (byte_count == {1'b0,fat_byte_offset}) fat_entry[7:0] <= sd_sec_read_data;
                                if (byte_count == ({1'b0,fat_byte_offset} + 10'd1)) fat_entry[15:8] <= sd_sec_read_data;
                                if (byte_count == ({1'b0,fat_byte_offset} + 10'd2)) fat_entry[23:16] <= sd_sec_read_data;
                                if (byte_count == ({1'b0,fat_byte_offset} + 10'd3)) fat_entry[31:24] <= sd_sec_read_data;
                            end
                            if (phase == PH_DATA && bytes_remaining != 32'd0) begin
                                data_byte <= sd_sec_read_data;
                                data_valid <= 1'b1;
                                data_last <= (bytes_remaining == 32'd1);
                                data_offset <= file_position;
                                file_position <= file_position + 32'd1;
                                bytes_remaining <= bytes_remaining - 32'd1;
                            end
                        end
                        if (sd_sec_read_end) begin
                            sd_sec_read <= 1'b0;
                            if (byte_count != 10'd512) begin
                                error <= ERR_SECTOR;
                                busy <= 1'b0;
                                state <= ST_IDLE;
                            end else begin
                                state <= ST_ADVANCE;
                            end
                        end else if (timer >= TIMEOUT_CYCLES) begin
                            error <= ERR_TIMEOUT;
                            sd_sec_read <= 1'b0;
                            busy <= 1'b0;
                            state <= ST_IDLE;
                        end else begin
                            timer <= timer + 32'd1;
                        end
                    end
                    ST_ADVANCE: begin
                        case (phase)
                            PH_LBA0: begin
                                if (bpb_valid) begin
                                    volume_lba <= 32'd0;
                                    state <= ST_SETUP;
                                end else if ((sig510 == 8'h55) && (sig511 == 8'haa) &&
                                             mbr_partition_found && (mbr_part_lba != 32'd0)) begin
                                    volume_lba <= mbr_part_lba;
                                    read_lba <= mbr_part_lba;
                                    bpb_bps <= 16'd0;
                                    bpb_spc <= 8'd0;
                                    bpb_reserved <= 16'd0;
                                    bpb_num_fats <= 8'd0;
                                    bpb_fat16 <= 16'd0;
                                    bpb_fat32 <= 32'd0;
                                    bpb_root <= 32'd0;
                                    sig510 <= 8'd0;
                                    sig511 <= 8'd0;
                                    phase <= PH_BPB;
                                    state <= ST_REQUEST;
                                end else begin
                                    error <= ERR_MBR;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end
                            end
                            PH_BPB: begin
                                if (bpb_valid) state <= ST_SETUP;
                                else begin
                                    error <= ERR_BPB;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end
                            end
                            PH_DIR: begin
                                if (entry_found) begin
                                    if (seek_folder) begin
                                        if (candidate_cluster < 32'd2) begin
                                            error <= ERR_CLUSTER;
                                            busy <= 1'b0;
                                            state <= ST_IDLE;
                                        end else begin
                                            dir_cluster <= candidate_cluster;
                                            seek_folder <= 1'b0;
                                            dir_cluster_hops <= 16'd0;
                                            cluster_sector_index <= 8'd0;
                                            addr_cluster <= candidate_cluster;
                                            state <= ST_LBA_SUB;
                                        end
                                    end else begin
                                        file_size <= candidate_size;
                                        file_cluster <= candidate_cluster;
                                        bytes_remaining <= (read_limit_bytes == 32'd0 ||
                                            read_limit_bytes >= candidate_size) ? candidate_size : read_limit_bytes;
                                        found <= 1'b1;
                                        state <= ST_FILE_INIT;
                                    end
                                end else if (end_of_directory) begin
                                    error <= ERR_NOFILE;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end else if (cluster_sector_index + 8'd1 < spc) begin
                                    cluster_sector_index <= cluster_sector_index + 8'd1;
                                    read_lba <= read_lba + 32'd1;
                                    state <= ST_REQUEST;
                                end else begin
                                    // After the last sector of this cluster, look up FAT chain.
                                    fat_byte_offset <= {dir_cluster[6:0],2'b00};
                                    read_lba <= fat_lba + (dir_cluster >> 7);
                                    phase <= PH_DIR_FAT;
                                    state <= ST_REQUEST;
                                end
                            end
                            PH_DIR_FAT: begin
                                if (fat_eoc) begin
                                    error <= ERR_NOFILE;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end else if (!fat_valid) begin
                                    error <= ERR_CLUSTER;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end else if (dir_cluster_hops >= MAX_DIR_CLUSTERS) begin
                                    error <= ERR_DIRLOOP;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end else begin
                                    dir_cluster_hops <= dir_cluster_hops + 16'd1;
                                    dir_cluster <= {4'd0,next_fat_cluster};
                                    cluster_sector_index <= 8'd0;
                                    addr_cluster <= {4'd0,next_fat_cluster};
                                    phase <= PH_DIR;
                                    state <= ST_LBA_SUB;
                                end
                            end
                            PH_DATA: begin
                                if (bytes_remaining == 32'd0) begin
                                    busy <= 1'b0;
                                    done <= 1'b1;
                                    state <= ST_IDLE;
                                end else if (cluster_sector_index + 8'd1 < spc) begin
                                    cluster_sector_index <= cluster_sector_index + 8'd1;
                                    read_lba <= read_lba + 32'd1;
                                    state <= ST_REQUEST;
                                end else begin
                                    fat_byte_offset <= {file_cluster[6:0],2'b00};
                                    read_lba <= fat_lba + (file_cluster >> 7);
                                    phase <= PH_DATA_FAT;
                                    state <= ST_REQUEST;
                                end
                            end
                            PH_DATA_FAT: begin
                                if (fat_eoc) begin
                                    error <= ERR_SHORT;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end else if (!fat_valid) begin
                                    error <= ERR_CLUSTER;
                                    busy <= 1'b0;
                                    state <= ST_IDLE;
                                end else begin
                                    file_cluster <= {4'd0,next_fat_cluster};
                                    cluster_sector_index <= 8'd0;
                                    addr_cluster <= {4'd0,next_fat_cluster};
                                    phase <= PH_DATA;
                                    state <= ST_LBA_SUB;
                                end
                            end
                            default: begin
                                error <= ERR_BPB;
                                busy <= 1'b0;
                                state <= ST_IDLE;
                            end
                        endcase
                    end
                    ST_SETUP: begin
                        fat_lba <= volume_lba + {16'd0,bpb_reserved};
                        data_lba <= volume_lba + {16'd0,bpb_reserved} +
                                    ((bpb_num_fats == 8'd2) ? (bpb_fat32 << 1) : bpb_fat32);
                        root_cluster <= (bpb_root & 32'h0fff_ffff);
                        spc <= bpb_spc;
                        spc_shift <= spc_to_shift(bpb_spc);
                        mounted <= 1'b1;
                        state <= ST_DIR_INIT;
                    end
                    ST_DIR_INIT: begin
                        dir_cluster <= root_cluster;
                        cluster_sector_index <= 8'd0;
                        dir_cluster_hops <= 16'd0;
                        addr_cluster <= root_cluster;
                        phase <= PH_DIR;
                        state <= ST_LBA_SUB;
                    end
                    ST_FILE_INIT: begin
                        if (lookup_only || bytes_remaining == 32'd0) begin
                            busy <= 1'b0;
                            done <= 1'b1;
                            state <= ST_IDLE;
                        end else if (candidate_cluster < 32'd2) begin
                            error <= ERR_CLUSTER;
                            busy <= 1'b0;
                            state <= ST_IDLE;
                        end else begin
                            cluster_sector_index <= 8'd0;
                            addr_cluster <= candidate_cluster;
                            phase <= PH_DATA;
                            state <= ST_LBA_SUB;
                        end
                    end
                    ST_LBA_SUB: begin
                        addr_delta <= addr_cluster - 32'd2;
                        state <= ST_LBA_SHIFT;
                    end
                    ST_LBA_SHIFT: begin
                        addr_scaled <= addr_delta << spc_shift;
                        state <= ST_LBA_ADD;
                    end
                    ST_LBA_ADD: begin
                        read_lba <= data_lba + addr_scaled;
                        state <= ST_REQUEST;
                    end
                    default: begin
                        error <= ERR_BPB;
                        busy <= 1'b0;
                        sd_sec_read <= 1'b0;
                        state <= ST_IDLE;
                    end
                endcase
            end
        end
    end
endmodule
