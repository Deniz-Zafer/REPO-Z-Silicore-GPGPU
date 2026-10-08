`timescale 1ns / 1ps

package gpu_cfg_pkg;
  localparam int NUM_WARPS = 4;
  localparam int LANES     = 8;
  localparam int VREGS     = 8;
  localparam int DATA_W    = 32;
  localparam int PC_W      = 32;

  // Bank paramları (şimdilik RF için)
  localparam int NUM_BANKS = 4;
endpackage


package lsu_types_pkg;
  import gpu_cfg_pkg::*;

  typedef struct packed {
    logic [$clog2(NUM_WARPS)-1:0] warp_id;

    logic                         is_load;
    logic                         is_store;

    logic [$clog2(VREGS)-1:0]     rd;        // load dest (store için ignore)

    logic [LANES-1:0]             mask;

    logic [LANES-1:0][DATA_W-1:0] addr_vec;  // lane-lane adresler
    logic [LANES-1:0][DATA_W-1:0] store_vec; // lane-lane store data
  } lsq_entry_t;
endpackage


package ex_types_pkg;
  import gpu_cfg_pkg::*;

  typedef struct packed {
    logic [$clog2(NUM_WARPS)-1:0]         warp_id;
    logic [5:0]                           op_code;
    logic [$clog2(VREGS)-1:0]             rd;
    logic [LANES-1:0]                     mask;

    logic [LANES-1:0][DATA_W-1:0]         V1_vec;
    logic [LANES-1:0][DATA_W-1:0]         V2_vec;      // burada V2_eff olacak
    logic [LANES-1:0][DATA_W-1:0]         Vold_vec;    // VFMA accumulate
  } ex_entry_t;
endpackage


import gpu_cfg_pkg::*;

// ============================================================
// GPU Core V0.3
// - Mask'li ALL-reduce BEQ
// - Unconditional JUMP
// - PC update logic in PC_control_unit
// - WARP hazard optimized via sel_use_mask (3-bit)
// ============================================================

module GPU_Core (
    input  logic clk,
    input  logic rst_n
);

    // -------------------------
    // Scheduler <-> WARP
    // -------------------------
    logic                         sched_valid;
    logic [$clog2(NUM_WARPS)-1:0]  sel_warp_id;
    logic [NUM_WARPS-1:0]          can_issue;

    logic [PC_W-1:0]               pc_sel;
    logic [LANES-1:0]              mask_sel;

    // -------------------------
    // Fetch / Decode
    // -------------------------
    logic [31:0]                   instruction_wire;

    logic [5:0]                    opcode_wire;
    logic [$clog2(VREGS)-1:0]      dest_addr, src1_addr, src2_addr;
    logic [LANES-1:0]              mask_wire;

    logic                          SV_sel_wire;
    logic                          Imm_sel_wire;
    logic                          is_jump_wire, is_beq_wire;

    logic                          reg_we_wire, mem_we_wire, wb_sel_wire;
    logic                          branch_sel_wire; // (jump veya beq)

    logic [7:0]                    imm_field_wire;
    logic [31:0]                   imm_value_wire;
    logic [31:0]                   branch_offset_wire;

    // -------------------------
    // Issue / EX handshake
    // -------------------------
    logic                          id_ready;
    logic                          ex_ready;
    logic                          ex_busy;
    logic                          do_issue_id;

    // Issue_reg -> EX payload
    logic                          ex_valid;
    logic                          ex_stall;

    logic [$clog2(NUM_WARPS)-1:0]  ex_warp_id;
    logic [5:0]                    ex_op_code;
    logic [$clog2(VREGS)-1:0]      ex_rd;
    logic [LANES-1:0]              ex_mask;

    logic [LANES-1:0][DATA_W-1:0]  ex_V1_vec;
    logic [LANES-1:0][DATA_W-1:0]  ex_V2_eff;
    logic [LANES-1:0][DATA_W-1:0]  ex_old_vec;
    logic [LANES-1:0][DATA_W-1:0]  ex_store_vec;

    logic                          ex_reg_we;
    logic                          ex_mem_we;
    logic                          ex_wb_sel;
    logic                          ex_is_load;
    logic                          ex_is_store;
    logic                          ex_is_branch;

    // -------------------------
    // WARP operand outputs
    // -------------------------
    logic [LANES-1:0][DATA_W-1:0]  V1_vec, V2_vec, Vrd_old_vec, store_vec;
    logic [NUM_WARPS-1:0][VREGS-1:0] busy;
    logic                          issue_ok_sel;
    
    logic pc_taken_wire;
    logic flush_out;
    logic [$clog2(NUM_WARPS)-1:0] flush_id_out;

    // rd_old: VFMA accumulate için dest'in eski hali (şimdilik dest)
    wire [$clog2(VREGS)-1:0] rd_old_addr = dest_addr;

    // -------------------------
    // V2 effective (imm broadcast)
    // -------------------------
    logic [LANES-1:0][DATA_W-1:0]  V2_eff;

    generate
        for (genvar i = 0; i < LANES; i++) begin : GEN_V2
            assign V2_eff[i] = (Imm_sel_wire) ? imm_value_wire : V2_vec[i];
        end
    endgenerate

    // -------------------------
    // PC control wires
    // -------------------------
    logic                         pc_update_valid_wire;
    logic [PC_W-1:0]              pc_next_wire;

    // -------------------------
    // WB mux wires to WARP
    // -------------------------
    logic                          wb_valid_mux;
    logic [$clog2(NUM_WARPS)-1:0]  wb_warp_id_mux;
    logic [$clog2(VREGS)-1:0]      wb_rd_mux;
    logic [LANES-1:0]              wb_mask_mux;
    logic [LANES-1:0][DATA_W-1:0]  wb_data_mux;

    // -------------------------
    // ALU / DMEM
    // -------------------------
    logic                         alu_valid_out;
    logic [LANES-1:0][DATA_W-1:0]  alu_vout;
    logic [$clog2(VREGS)-1:0]      alu_rd_out;

    logic [LANES-1:0][DATA_W-1:0]  mem_rdata_wire;

    // -------------------------
    // Handshake policy (şimdilik stall yok)
    // -------------------------
    assign ex_ready = ~ex_busy;


    // scheduler seçti + hazard ok + issue_reg alabiliyor
    assign do_issue_id = sched_valid && issue_ok_sel && id_ready;

    // ------------------------------------------------------------
    // Operand usage mask for hazard check (min LUT)
    // [0]=rs1, [1]=rs2, [2]=rd_old
    // ------------------------------------------------------------
    logic [2:0] sel_use_mask;

    // Opcodes (decoder ile uyumlu)
    localparam [5:0] OPC_VADD   = 6'b10_0000;
    localparam [5:0] OPC_VADDI  = 6'b11_0000;
    localparam [5:0] OPC_VLOAD  = 6'b10_1000;
    localparam [5:0] OPC_VSTORE = 6'b10_1001;
    localparam [5:0] OPC_VFMA   = 6'b10_0011;

    localparam [5:0] OPC_JUMP   = 6'b01_1110;
    localparam [5:0] OPC_BEQ    = 6'b01_1111;

    always_comb begin
        // default: rs1+rs2, rd_old yok
        sel_use_mask = 3'b011;

        if (is_jump_wire) begin
            sel_use_mask = 3'b000;
        end else if (is_beq_wire) begin
            sel_use_mask = 3'b011;
        end else if (opcode_wire == OPC_VADDI) begin
            sel_use_mask = 3'b001; // rs1 only
        end else if (opcode_wire == OPC_VLOAD) begin
            sel_use_mask = 3'b001; // addr from rs1 only
        end else if (opcode_wire == OPC_VSTORE) begin
            sel_use_mask = 3'b011; // addr rs1 + data rs2
        end else if (opcode_wire == OPC_VFMA) begin
            sel_use_mask = 3'b111; // rs1 rs2 rd_old
        end
    end

    // ============================================================
    // Scheduler
    // ============================================================
    Scheduler #(.NUM_WARPS(NUM_WARPS)) sched_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .can_issue  (can_issue),
        .sel_warp_id(sel_warp_id),
        .sel_valid  (sched_valid)
    );

    // ============================================================
    // IMEM
    // ============================================================
    IMEM imem_inst (
        .clk        (clk),
        .pc_addr    (pc_sel),
        .instruction(instruction_wire)
    );

    // ============================================================
    // Decoder
    // ============================================================
    decoder dec_inst (
        .instr      (instruction_wire),

        .opcode     (opcode_wire),
        .addr_dest  (dest_addr),
        .addr_src1  (src1_addr),
        .addr_src2  (src2_addr),

        .SV_sel     (SV_sel_wire),
        .Imm_sel    (Imm_sel_wire),

        .is_jump    (is_jump_wire),
        .is_beq     (is_beq_wire),

        .write_mask (mask_wire),
        .imm_field  (imm_field_wire),

        .o_reg_we   (reg_we_wire),
        .o_mem_we   (mem_we_wire),
        .o_wb_sel   (wb_sel_wire),

        .branch_sel (branch_sel_wire) // jump veya beq
    );

    // ============================================================
    // Imm Unit
    // ============================================================
    Imm_Unit imm_inst (
        .imm_field     (imm_field_wire),
        .imm_value     (imm_value_wire),
        .branch_offset (branch_offset_wire)
    );

    // ============================================================
    // PC / Branch control (ALL-reduce BEQ, unconditional JUMP)
    // ============================================================
    PC_control_unit #(
      .PC_W   (PC_W),
      .LANES  (LANES),
      .DATA_W (DATA_W)
    ) pc_ctrl (
      .do_issue       (do_issue_id),
      .is_jump        (is_jump_wire),
      .is_beq         (is_beq_wire),
      .pc_cur         (pc_sel),
      .branch_offset  (branch_offset_wire),
      .mask           (mask_sel),
      .V1_vec         (V1_vec),
      .V2_vec         (V2_vec),
      .pc_update_valid(pc_update_valid_wire),
      .pc_next        (pc_next_wire),
      .taken          (pc_taken_wire) 
    );

    // ============================================================
    // WARP bank
    // ============================================================
    WARP #(
        .NUM_WARPS(NUM_WARPS),
        .LANES    (LANES),
        .VREGS    (VREGS),
        .DATA_W   (DATA_W),
        .PC_W     (PC_W)
    ) warp_inst (
        .clk        (clk),
        .rst_n      (rst_n),

        .sel_warp_id(sel_warp_id),
        .pc_sel     (pc_sel),
        .mask_sel   (mask_sel),
        .can_issue  (can_issue),

        .op_warp_id (sel_warp_id),
        .rs1        (src1_addr),
        .rs2        (src2_addr),
        .rd_old     (rd_old_addr),

        .V1_vec     (V1_vec),
        .V2_vec     (V2_vec),
        .Vrd_old_vec(Vrd_old_vec),
        .store_vec  (store_vec),

        // WB
        .wb_valid   (wb_valid_mux),
        .wb_warp_id (wb_warp_id_mux),
        .wb_rd      (wb_rd_mux),
        .wb_mask    (wb_mask_mux),
        .wb_data    (wb_data_mux),

        // Issue -> busy set
        .issue_valid     (do_issue_id),
        .issue_warp_id   (sel_warp_id),
        .issue_writes_rd (reg_we_wire && !branch_sel_wire),
        .issue_rd        (dest_addr),

        .busy            (busy),
        .sel_use_mask    (sel_use_mask),
        .issue_ok_sel    (issue_ok_sel),

        // stall yok
        .stall_set_valid   (1'b0),
        .stall_clr_valid   (1'b0),
        .stall_set_warp_id ('0),
        .stall_clr_warp_id ('0),

        // PC update
        .pc_update_valid   (pc_update_valid_wire),
        .pc_update_warp_id (sel_warp_id),
        .pc_next           (pc_next_wire)
    );

    // ============================================================
    // Issue_reg (ID -> EX)
    // ============================================================
    Issue_reg #(
        .NUM_WARPS(NUM_WARPS),
        .LANES    (LANES),
        .VREGS    (VREGS),
        .DATA_W   (DATA_W),
        .PC_W     (PC_W),
        .OPC_W    (6)
    ) issue_reg_inst (
        .clk        (clk),
        .rst_n      (rst_n),

        .do_issue   (do_issue_id),
        .flush      (1'b0),

        .id_warp_id (sel_warp_id),
        .id_op_code (opcode_wire),
        .id_rd      (dest_addr),
        .id_mask    (mask_wire),

        .id_V1_vec   (V1_vec),
        .id_V2_eff   (V2_eff),
        .id_old_vec  (Vrd_old_vec),
        .id_store_vec(store_vec),

        .id_reg_we   (reg_we_wire),
        .id_mem_we   (mem_we_wire),
        .id_wb_sel   (wb_sel_wire),
        .id_is_load  (wb_sel_wire && reg_we_wire),
        .id_is_store (mem_we_wire),
        .id_is_branch(branch_sel_wire),

        .ex_ready   (ex_ready),
        .id_ready   (id_ready),

        .ex_valid   (ex_valid),
        .ex_stall   (ex_stall),

        .ex_warp_id (ex_warp_id),
        .ex_op_code (ex_op_code),
        .ex_rd      (ex_rd),
        .ex_mask    (ex_mask),

        .ex_V1_vec   (ex_V1_vec),
        .ex_V2_eff   (ex_V2_eff),
        .ex_old_vec  (ex_old_vec),
        .ex_store_vec(ex_store_vec),

        .ex_reg_we  (ex_reg_we),
        .ex_mem_we  (ex_mem_we),
        .ex_wb_sel  (ex_wb_sel),
        .ex_is_load (ex_is_load),
        .ex_is_store(ex_is_store),
        .ex_is_branch(ex_is_branch)
    );

    // ============================================================
    // ALU
    // ============================================================
    ALU #(
        .LANES(LANES),
        .WIDTH(DATA_W)
    ) alu_inst (
        .clk           (clk),
        .rst_n         (rst_n),

        .valid_in      (ex_valid),
        .V1            (ex_V1_vec),
        .V2            (ex_V2_eff),
        .Vacc          (ex_old_vec),
        .op_code       (ex_op_code),
        .MASK          (ex_mask),

        .dest_addr_in  (ex_rd[$bits(alu_rd_out)-1:0]),
        .valid_out     (alu_valid_out),
        .Vout          (alu_vout),
        .dest_addr_out (alu_rd_out)
    );

    // ============================================================
    // DMEM (basit shared)
    // addr: lane0 pointer
    // ============================================================
    DMEM dmem_inst (
        .clk    (clk),
        .mem_we (ex_valid && ex_mem_we),
        .addr   (ex_V1_vec[0]),
        .wdata  (ex_store_vec),
        .rdata  (mem_rdata_wire)
    );

    // ============================================================
    // WB mux (EX stage based)
    // ============================================================
    wire wb_fire = ex_valid && ex_reg_we && !ex_is_branch && !ex_is_store;

    assign wb_valid_mux   = wb_fire;
    assign wb_warp_id_mux = ex_warp_id;
    assign wb_rd_mux      = ex_rd;
    assign wb_mask_mux    = ex_mask;
    assign wb_data_mux    = (ex_wb_sel) ? mem_rdata_wire : alu_vout;
    
    // ============================================================
    // Branch taken = Flush (EX_Q selective, LSQ-LSU-ALU Complete flush)
    // ============================================================
    assign flush_out    = do_issue_id && branch_sel_wire && pc_taken_wire;
    assign flush_id_out = sel_warp_id;
    
endmodule


// ============================================================
// IMEM
// ============================================================
module IMEM (
    input  logic        clk,
    input  logic [31:0]  pc_addr,
    output logic [31:0]  instruction
);
    logic [31:0] memory [0:63];
    assign instruction = memory[pc_addr[7:2]];
endmodule


// ============================================================
// Imm Unit
// ============================================================
module Imm_Unit (
    input  logic [7:0]  imm_field,
    output logic [31:0] imm_value,
    output logic [31:0] branch_offset
);
    logic [31:0] imm_sext;

    always_comb begin
        imm_sext       = {{24{imm_field[7]}}, imm_field};
        imm_value      = imm_sext;
        branch_offset  = imm_sext <<< 2;
    end
endmodule


// ============================================================
// Decoder (ISA: [31]=SV, [30]=Imm, [29:26]=op_field)
// ============================================================
module decoder (
    input  logic [31:0] instr,

    output logic [5:0]  opcode,
    output logic [2:0]  addr_dest,
    output logic [2:0]  addr_src1,
    output logic [2:0]  addr_src2,

    output logic        SV_sel,
    output logic        Imm_sel,

    output logic        is_jump,
    output logic        is_beq,

    output logic [7:0]  write_mask,
    output logic [7:0]  imm_field,

    output logic        o_reg_we,
    output logic        o_mem_we,
    output logic        o_wb_sel,

    output logic        branch_sel
);

    logic       sv_bit;
    logic       imm_bit;
    logic [3:0] op_field;

    assign sv_bit     = instr[31];
    assign imm_bit    = instr[30];
    assign op_field   = instr[29:26];

    assign opcode     = {sv_bit, imm_bit, op_field};

    assign addr_dest  = instr[25:23];
    assign addr_src1  = instr[22:20];
    assign addr_src2  = instr[19:17];

    assign imm_field  = instr[15:8];
    assign write_mask = instr[7:0];

    assign SV_sel     = sv_bit;
    assign Imm_sel    = imm_bit;

    // opcodes
    localparam [5:0] OPC_VADD   = 6'b10_0000;
    localparam [5:0] OPC_VADDI  = 6'b11_0000;
    localparam [5:0] OPC_VLOAD  = 6'b10_1000;
    localparam [5:0] OPC_VSTORE = 6'b10_1001;

    localparam [5:0] OPC_JUMP   = 6'b01_1110;
    localparam [5:0] OPC_BEQ    = 6'b01_1111;

    always_comb begin
        o_reg_we   = 1'b0;
        o_mem_we   = 1'b0;
        o_wb_sel   = 1'b0;

        is_jump    = 1'b0;
        is_beq     = 1'b0;
        branch_sel = 1'b0;

        unique case (opcode)
            OPC_VADD,
            OPC_VADDI: begin
                o_reg_we = 1'b1;
                o_wb_sel = 1'b0;
            end

            OPC_VLOAD: begin
                o_reg_we = 1'b1;
                o_wb_sel = 1'b1;
            end

            OPC_VSTORE: begin
                o_mem_we = 1'b1;
            end

            OPC_JUMP: begin
                is_jump    = 1'b1;
                branch_sel = 1'b1;
            end

            OPC_BEQ: begin
                is_beq     = 1'b1;
                branch_sel = 1'b1;
            end

            default: begin end
        endcase
    end

endmodule


// ============================================================
// PC_control_unit
// - JUMP: unconditional taken
// - BEQ : mask'li ALL-reduce compare
// ============================================================
module PC_control_unit #(
    parameter int PC_W   = 32,
    parameter int LANES  = 8,
    parameter int DATA_W = 32
)(
    input  logic                     do_issue,
    input  logic                     is_jump,
    input  logic                     is_beq,

    input  logic [PC_W-1:0]          pc_cur,
    input  logic [PC_W-1:0]          branch_offset,

    input  logic [LANES-1:0]         mask,
    input  logic [LANES-1:0][DATA_W-1:0] V1_vec,
    input  logic [LANES-1:0][DATA_W-1:0] V2_vec,

    output logic                     pc_update_valid,
    output logic [PC_W-1:0]          pc_next
);
    logic beq_taken_all;
    logic taken;

    always_comb begin
        pc_update_valid = 1'b0;
        pc_next         = pc_cur;

        // ALL-reduce with mask: AND_i( !mask[i] || (V1[i]==V2[i]) )
        beq_taken_all = 1'b1;
        for (int i = 0; i < LANES; i++) begin
            beq_taken_all &= ( (!mask[i]) || (V1_vec[i] == V2_vec[i]) );
        end

        taken = 1'b0;
        if (is_jump) begin
            taken = 1'b1;
        end else if (is_beq) begin
            taken = beq_taken_all;
        end

        if (do_issue) begin
            pc_update_valid = 1'b1;
            if (taken) pc_next = pc_cur + branch_offset;
            else       pc_next = pc_cur + 32'd4;
        end
    end
endmodule


// ============================================================
// Scheduler (Round-robin)
// ============================================================
module Scheduler #(
    parameter int NUM_WARPS = 4
)(
    input  logic clk,
    input  logic rst_n,

    input  logic [NUM_WARPS-1:0]         can_issue,

    output logic [$clog2(NUM_WARPS)-1:0] sel_warp_id,
    output logic                         sel_valid
);
    localparam int W = $clog2(NUM_WARPS);

    logic [W-1:0] last_grant;
    logic [W-1:0] pick_id;
    logic         pick_valid;

    always_comb begin
        pick_id    = last_grant;
        pick_valid = 1'b0;

        for (int k = 1; k <= NUM_WARPS; k++) begin
            int idx = (last_grant + k) % NUM_WARPS;
            if (!pick_valid && can_issue[idx]) begin
                pick_valid = 1'b1;
                pick_id    = idx[W-1:0];
            end
        end

        sel_valid   = pick_valid;
        sel_warp_id = pick_id;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) last_grant <= '0;
        else if (pick_valid) last_grant <= pick_id;
    end
endmodule


// ============================================================
// Issue_reg (1-entry buffer with ready/valid)
// ============================================================
module Issue_reg #(
    parameter int NUM_WARPS = 4,
    parameter int LANES     = 8,
    parameter int VREGS     = 8,
    parameter int DATA_W    = 32,
    parameter int PC_W      = 32,
    parameter int OPC_W     = 6
)(
    input  logic                         clk,
    input  logic                         rst_n,

    input  logic                         do_issue,
    input  logic                         flush,

    input  logic [$clog2(NUM_WARPS)-1:0]  id_warp_id,
    input  logic [OPC_W-1:0]              id_op_code,
    input  logic [$clog2(VREGS)-1:0]      id_rd,
    input  logic [LANES-1:0]              id_mask,

    input  logic [LANES-1:0][DATA_W-1:0]  id_V1_vec,
    input  logic [LANES-1:0][DATA_W-1:0]  id_V2_eff,
    input  logic [LANES-1:0][DATA_W-1:0]  id_old_vec,
    input  logic [LANES-1:0][DATA_W-1:0]  id_store_vec,

    input  logic                         id_reg_we,
    input  logic                         id_mem_we,
    input  logic                         id_wb_sel,
    input  logic                         id_is_load,
    input  logic                         id_is_store,
    input  logic                         id_is_branch,

    input  logic                         ex_ready,
    output logic                         id_ready,

    output logic                         ex_valid,
    output logic                         ex_stall,

    output logic [$clog2(NUM_WARPS)-1:0]  ex_warp_id,
    output logic [OPC_W-1:0]              ex_op_code,
    output logic [$clog2(VREGS)-1:0]      ex_rd,
    output logic [LANES-1:0]              ex_mask,

    output logic [LANES-1:0][DATA_W-1:0]  ex_V1_vec,
    output logic [LANES-1:0][DATA_W-1:0]  ex_V2_eff,
    output logic [LANES-1:0][DATA_W-1:0]  ex_old_vec,
    output logic [LANES-1:0][DATA_W-1:0]  ex_store_vec,

    output logic                         ex_reg_we,
    output logic                         ex_mem_we,
    output logic                         ex_wb_sel,
    output logic                         ex_is_load,
    output logic                         ex_is_store,
    output logic                         ex_is_branch
);

    assign ex_stall = ex_valid && !ex_ready;
    assign id_ready = (!ex_valid) || ex_ready;

    logic hold, load, bubble;

    always_comb begin
        hold   = ex_valid && !ex_ready;
        load   = (!hold) && do_issue;
        bubble = (!hold) && !do_issue;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ex_valid     <= 1'b0;

            ex_warp_id   <= '0;
            ex_op_code   <= '0;
            ex_rd        <= '0;
            ex_mask      <= '0;

            ex_V1_vec    <= '0;
            ex_V2_eff    <= '0;
            ex_old_vec   <= '0;
            ex_store_vec <= '0;

            ex_reg_we    <= 1'b0;
            ex_mem_we    <= 1'b0;
            ex_wb_sel    <= 1'b0;
            ex_is_load   <= 1'b0;
            ex_is_store  <= 1'b0;
            ex_is_branch <= 1'b0;

        end else begin
            if (flush) begin
                ex_valid <= 1'b0;
            end else if (load) begin
                ex_valid     <= 1'b1;

                ex_warp_id   <= id_warp_id;
                ex_op_code   <= id_op_code;
                ex_rd        <= id_rd;
                ex_mask      <= id_mask;

                ex_V1_vec    <= id_V1_vec;
                ex_V2_eff    <= id_V2_eff;
                ex_old_vec   <= id_old_vec;
                ex_store_vec <= id_store_vec;

                ex_reg_we    <= id_reg_we;
                ex_mem_we    <= id_mem_we;
                ex_wb_sel    <= id_wb_sel;
                ex_is_load   <= id_is_load;
                ex_is_store  <= id_is_store;
                ex_is_branch <= id_is_branch;
            end else if (bubble) begin
                ex_valid <= 1'b0;
            end
        end
    end

endmodule


// ============================================================
// ALU (combinational)
// ============================================================
module ALU #(
    parameter int LANES = 8,
    parameter int WIDTH = 32
)(
    input  logic                        clk,
    input  logic                        rst_n,

    input  logic                        valid_in,

    input  logic [LANES-1:0][WIDTH-1:0] V1,
    input  logic [LANES-1:0][WIDTH-1:0] V2,
    input  logic [LANES-1:0][WIDTH-1:0] Vacc,

    input  logic [5:0]                  op_code,
    input  logic [LANES-1:0]            MASK,

    input  logic [$clog2(8)-1:0]        dest_addr_in,

    output logic                        valid_out,
    output logic [LANES-1:0][WIDTH-1:0] Vout,
    output logic [$clog2(8)-1:0]        dest_addr_out
);

    localparam [5:0] OPC_VADD = 6'b10_0000;
    localparam [5:0] OPC_VMUL = 6'b10_0010;
    localparam [5:0] OPC_VFMA = 6'b10_0011;

    always_comb begin
        valid_out     = valid_in;
        dest_addr_out = dest_addr_in;
        Vout          = '0;

        if (valid_in) begin
            unique case (op_code)
                OPC_VADD: begin
                    for (int i = 0; i < LANES; i++) if (MASK[i]) Vout[i] = V1[i] + V2[i];
                end
                OPC_VMUL: begin
                    for (int i = 0; i < LANES; i++) if (MASK[i]) Vout[i] = V1[i] * V2[i];
                end
                OPC_VFMA: begin
                    for (int i = 0; i < LANES; i++) if (MASK[i]) Vout[i] = (V1[i] * V2[i]) + Vacc[i];
                end
                default: Vout = '0;
            endcase
        end
    end
endmodule



module EX_Queue #(
  parameter int  QUEUE_LEN = 2,
  parameter type T         = ex_types_pkg::ex_entry_t,
  parameter int  NUM_WARPS = gpu_cfg_pkg::NUM_WARPS
)(
  input  logic clk,
  input  logic rst_n,

  // enqueue
  input  logic enq_valid,
  output logic enq_ready,
  input  T     enq_data,

  // dequeue
  output logic deq_valid,
  input  logic deq_ready,
  output T     deq_data,

  // flush
  input  logic flush,  // global flush
  input  logic flush_wid_valid,
  input  logic [$clog2(NUM_WARPS)-1:0] flush_wid
);

  localparam int CNT_W = $clog2(QUEUE_LEN + 1);

  T                mem   [0:QUEUE_LEN-1];
  logic [CNT_W-1:0] count;

  // handshakes (baseline: flush aynı cycle'da ready/valid'i etkilemez)
  always_comb begin
    enq_ready = (count != QUEUE_LEN[CNT_W-1:0]);
    deq_valid = (count != 0);
    deq_data  = mem[0];
  end

  wire enq_fire = enq_valid && enq_ready && (!flush);
  wire deq_fire = deq_valid && deq_ready && (!flush);

  // helper: keep entry?
  function automatic logic keep_entry(input T e);
    keep_entry = !(flush_wid_valid && (e.warp_id == flush_wid));
  endfunction

  integer i;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || flush) begin
      count <= '0;
      for (i = 0; i < QUEUE_LEN; i++) begin
        mem[i] <= '0;
      end
    end else begin
      // -------------------------
      // 1) Optional warp-scoped flush: filter + compact
      // -------------------------
      if (flush_wid_valid && (count != 0)) begin
        T                new_mem [0:QUEUE_LEN-1];
        logic [CNT_W-1:0] new_cnt;
        integer           j;

        // init
        for (i = 0; i < QUEUE_LEN; i++) new_mem[i] = '0;
        new_cnt = '0;
        j = 0;

        // compact surviving entries (only from existing valid range [0..count-1])
        for (i = 0; i < QUEUE_LEN; i++) begin
          if (i < count) begin
            if (keep_entry(mem[i])) begin
              new_mem[j] = mem[i];
              j++;
            end
          end
        end

        new_cnt = j[CNT_W-1:0];

        // commit compacted state
        count <= new_cnt;
        for (i = 0; i < QUEUE_LEN; i++) begin
          mem[i] <= new_mem[i];
        end
      end

      // -------------------------
      // 2) Dequeue: shift left by 1
      // (deq_fire varsa, head çıkar)
      // -------------------------
      if (deq_fire) begin
        for (i = 0; i < QUEUE_LEN-1; i++) begin
          mem[i] <= mem[i+1];
        end
        mem[QUEUE_LEN-1] <= '0;
        count <= count - 1'b1;
      end

      // -------------------------
      // 3) Enqueue: append at tail (index = count_after_deq)
      // Not: aynı cycle deq+enq olursa, yukarıdaki count-- sonrası tail doğru olur.
      // -------------------------
      if (enq_fire) begin
        mem[count] <= enq_data;
        count <= count + 1'b1;
      end
    end
  end

endmodule


// ============================================================
// DMEM (shared) : addr lane0, data vector
// ============================================================
module DMEM (
    input  logic              clk,
    input  logic              mem_we,
    input  logic [31:0]       addr,
    input  logic [7:0][31:0]  wdata,
    output logic [7:0][31:0]  rdata
);
    logic [7:0][31:0] memory [0:63];

    assign rdata = memory[addr[5:0]];

    always_ff @(posedge clk) begin
        if (mem_we) memory[addr[5:0]] <= wdata;
    end
endmodule


// ============================================================
// WARP: PC bank + mask bank + vector regs + scoreboard busy
// - hazard uses sel_use_mask (3-bit) => minimal LUT
// ============================================================
`timescale 1ns/1ps
import gpu_cfg_pkg::*;

// ============================================================
// WARP (optimized, RF multi-cycle handshake safe)
// - ONLY 1 tiny state: rf_hold_wid (to keep scheduler on same warp while RF busy)
// ============================================================
module WARP (
    input  logic clk,
    input  logic rst_n,

    // -------------------------
    // Scheduler interface
    // -------------------------
    input  logic                         sel_valid,     // sched_valid
    input  logic [$clog2(NUM_WARPS)-1:0]  sel_warp_id,
    output logic [NUM_WARPS-1:0]          can_issue,

    output logic [PC_W-1:0]               pc_sel,
    output logic [LANES-1:0]              mask_sel,

    // -------------------------
    // Decode (selected warp's candidate instruction)
    // -------------------------
    input  logic [$clog2(VREGS)-1:0]      rs1,
    input  logic [$clog2(VREGS)-1:0]      rs2,
    input  logic [$clog2(VREGS)-1:0]      rd_old,
    input  logic [2:0]                    sel_use_mask,

    // -------------------------
    // RF read outputs (go to ID/EX)
    // -------------------------
    output logic [LANES-1:0][DATA_W-1:0]  V1_vec,
    output logic [LANES-1:0][DATA_W-1:0]  V2_vec,
    output logic [LANES-1:0][DATA_W-1:0]  Vrd_old_vec,
    output logic [LANES-1:0][DATA_W-1:0]  store_vec,

    // -------------------------
    // Issue (actual fire)
    // -------------------------
    input  logic                          do_issue_id,      // sched_valid && issue_ok_sel && id_ready
    input  logic                          issue_writes_rd,
    input  logic [$clog2(VREGS)-1:0]      issue_rd,

    output logic                          issue_ok_sel,

    // -------------------------
    // Writeback
    // -------------------------
    input  logic                          wb_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  wb_warp_id,
    input  logic [$clog2(VREGS)-1:0]      wb_rd,
    input  logic [LANES-1:0]              wb_mask,
    input  logic [LANES-1:0][DATA_W-1:0]  wb_data,

    // -------------------------
    // Stall controls (optional)
    // -------------------------
    input  logic                          stall_set_valid,
    input  logic                          stall_clr_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  stall_set_warp_id,
    input  logic [$clog2(NUM_WARPS)-1:0]  stall_clr_warp_id,

    // -------------------------
    // PC update
    // -------------------------
    input  logic                          pc_update_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  pc_update_warp_id,
    input  logic [PC_W-1:0]               pc_next,

    // -------------------------
    // Scoreboard exports
    // -------------------------
    output logic [NUM_WARPS-1:0][VREGS-1:0] busy,
    output logic [NUM_WARPS-1:0]            pending_LSU
);

    // -------------------------
    // Per-warp state banks
    // -------------------------
    logic [NUM_WARPS-1:0][PC_W-1:0]   pc_bank;
    logic [NUM_WARPS-1:0][LANES-1:0]  mask_bank;
    logic [NUM_WARPS-1:0]             stalled;
    logic [NUM_WARPS-1:0][VREGS-1:0]  busy_bank;

    assign busy     = busy_bank;
    assign pc_sel   = pc_bank[sel_warp_id];
    assign mask_sel = mask_bank[sel_warp_id];

    // V0.4'te LSQ ile set/clear edeceksin, şimdilik 0
    always_comb pending_LSU = '0;

    // -------------------------
    // Scoreboard hazard check (selected warp)
    // -------------------------
    wire rs1_ok   = (!sel_use_mask[0]) || (~busy_bank[sel_warp_id][rs1]);
    wire rs2_ok   = (!sel_use_mask[1]) || (~busy_bank[sel_warp_id][rs2]);
    wire rdold_ok = (!sel_use_mask[2]) || (~busy_bank[sel_warp_id][rd_old]);

    wire base_ok  = (~stalled[sel_warp_id]) && rs1_ok && rs2_ok && rdold_ok;

    // -------------------------
    // RF handshake (multi-cycle collector)
    // -------------------------
    logic rf_start, rf_ready, rf_busy;
    logic [$clog2(NUM_WARPS)-1:0] rf_hold_wid;

    // IMPORTANT:
    // - rf_start is a 1-cycle pulse when RF is idle and we have a valid+hazard-free candidate
    // - rf_busy stays high while RF is collecting operands
    // - rf_ready pulses when operands are available
    // - rf_consume = do_issue_id (so DONE pulse won't be lost)
    assign rf_start   = sel_valid && base_ok && !rf_busy;
    wire   rf_consume = do_issue_id;

    // Freeze which warp owns the RF while busy (2-bit register, nothing more)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rf_hold_wid <= '0;
        end else begin
            if (rf_start) rf_hold_wid <= sel_warp_id;
        end
    end

    // While RF is busy, only the held warp is "issuable" (keeps scheduler stable)
    genvar w;
    generate
        for (w = 0; w < NUM_WARPS; w++) begin : GEN_CAN_ISSUE
            assign can_issue[w] =
                (~stalled[w]) &&
                ( (!rf_busy) ? 1'b1 : (w[$clog2(NUM_WARPS)-1:0] == rf_hold_wid) );
        end
    endgenerate

    // issue_ok_sel must include rf_ready
    assign issue_ok_sel = base_ok && rf_ready;

    // store_vec legacy: rs2 vector
    assign store_vec = V2_vec;

    // -------------------------
    // Sequential updates (PC/mask/stall/scoreboard)
    // -------------------------
    integer wi;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (wi = 0; wi < NUM_WARPS; wi++) begin
                pc_bank[wi]   <= '0;
                mask_bank[wi] <= {LANES{1'b1}};
                stalled[wi]   <= 1'b0;
                busy_bank[wi] <= '0;
            end
        end else begin
            // PC update
            if (pc_update_valid) begin
                pc_bank[pc_update_warp_id] <= pc_next;
            end

            // stall set/clear
            if (stall_set_valid) stalled[stall_set_warp_id] <= 1'b1;
            if (stall_clr_valid) stalled[stall_clr_warp_id] <= 1'b0;

            // Issue -> busy set (selected warp issues)
            if (do_issue_id && issue_writes_rd) begin
                busy_bank[sel_warp_id][issue_rd] <= 1'b1;
            end

            // WB -> busy clear
            if (wb_valid) begin
                busy_bank[wb_warp_id][wb_rd] <= 1'b0;
            end
        end
    end

endmodule



module V_reg_file(
    input  logic clk,
    input  logic rst_n,

    // Request (candidate instruction)
    input  logic                          rf_start,     // 1-cycle pulse, only when busy==0
    input  logic [$clog2(NUM_WARPS)-1:0]  op_warp_id,
    input  logic [$clog2(VREGS)-1:0]      rs1,
    input  logic [$clog2(VREGS)-1:0]      rs2,
    input  logic [$clog2(VREGS)-1:0]      rd_old,
    input  logic [2:0]                    sel_use_mask,  // [0]=rs1 [1]=rs2 [2]=rd_old

    // Response consume
    input  logic                          rf_consume,   // 1 when upstream accepted operands

    // Read data (valid when rf_ready==1)
    output logic [LANES-1:0][DATA_W-1:0]  V1_vec,
    output logic [LANES-1:0][DATA_W-1:0]  V2_vec,
    output logic [LANES-1:0][DATA_W-1:0]  Vrd_old_vec,
    output logic                          rf_ready,      // level: 1 in DONE until consume
    output logic                          busy,

    // Writeback
    input  logic                          wb_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  wb_warp_id,
    input  logic [$clog2(VREGS)-1:0]      wb_rd,
    input  logic [LANES-1:0]              wb_mask,
    input  logic [LANES-1:0][DATA_W-1:0]  wb_data
);

    // -------------------------
    // Banked storage
    // -------------------------
    localparam int BANK_DEPTH = (VREGS + NUM_BANKS - 1) / NUM_BANKS;
    localparam int BANK_W     = (NUM_BANKS <= 1) ? 1 : $clog2(NUM_BANKS);
    localparam int IDX_W      = (BANK_DEPTH <= 1) ? 1 : $clog2(BANK_DEPTH);

    localparam bit NUM_BANKS_POW2 = (NUM_BANKS & (NUM_BANKS-1)) == 0;

    // [warp][bank][idx][lane][data]
    logic [NUM_WARPS-1:0][NUM_BANKS-1:0][BANK_DEPTH-1:0][LANES-1:0][DATA_W-1:0] mem;

    function automatic logic [BANK_W-1:0] bank_of(input logic [$clog2(VREGS)-1:0] reg_id);
        if (NUM_BANKS_POW2) bank_of = reg_id[BANK_W-1:0];
        else                bank_of = reg_id % NUM_BANKS;
    endfunction

    function automatic logic [IDX_W-1:0] idx_of(input logic [$clog2(VREGS)-1:0] reg_id);
        if (NUM_BANKS_POW2) idx_of = reg_id[$clog2(VREGS)-1:BANK_W];
        else                idx_of = reg_id / NUM_BANKS;
    endfunction

    // -------------------------
    // Collector FSM: IDLE -> COLLECT -> DONE
    // -------------------------
    typedef enum logic [1:0] { IDLE=2'd0, COLLECT=2'd1, DONE=2'd2 } st_e;
    st_e state, next_state;

    // Latched request
    logic [$clog2(NUM_WARPS)-1:0] wid_q;
    logic [$clog2(VREGS)-1:0]     rs1_q, rs2_q, rd_q;
    logic [2:0]                   use_q;

    logic [BANK_W-1:0] bank_rs1_q, bank_rs2_q, bank_rd_q;
    logic [IDX_W-1:0]  idx_rs1_q,  idx_rs2_q,  idx_rd_q;

    // Pending bits: which operands still need to be fetched
    logic [2:0] pending, pending_n;

    // Holds
    logic [LANES-1:0][DATA_W-1:0] rs1_hold, rs2_hold, rd_hold;

    // Status
    assign busy     = (state != IDLE);
    assign rf_ready = (state == DONE);

    // -------------------------
    // Per-cycle serve decision:
    // serve as many as possible in THIS cycle, BUT max 1 per bank.
    // Greedy priority: rs1 -> rs2 -> rd_old
    // -------------------------
    logic serve_rs1, serve_rs2, serve_rd;

    always_comb begin
        serve_rs1 = 1'b0;
        serve_rs2 = 1'b0;
        serve_rd  = 1'b0;

        // rs1 first
        if (pending[0]) serve_rs1 = 1'b1;

        // rs2 if different bank than already served
        if (pending[1]) begin
            if (!serve_rs1 || (bank_rs2_q != bank_rs1_q))
                serve_rs2 = 1'b1;
        end

        // rd_old if different from any served bank
        if (pending[2]) begin
            logic conflict;
            conflict = 1'b0;
            if (serve_rs1 && (bank_rd_q  == bank_rs1_q)) conflict = 1'b1;
            if (serve_rs2 && (bank_rd_q  == bank_rs2_q)) conflict = 1'b1;
            if (!conflict) serve_rd = 1'b1;
        end
    end

    // pending next after serving
    always_comb begin
        pending_n = pending;
        if (serve_rs1) pending_n[0] = 1'b0;
        if (serve_rs2) pending_n[1] = 1'b0;
        if (serve_rd)  pending_n[2] = 1'b0;
    end

    // -------------------------
    // Next-state
    // -------------------------
    always_comb begin
        next_state = state;

        unique case (state)
            IDLE: begin
                if (rf_start) begin
                    if (sel_use_mask == 3'b000) next_state = DONE;
                    else                        next_state = COLLECT;
                end
            end

            COLLECT: begin
                // after this cycle's capture, if nothing pending -> DONE
                if (pending_n == 3'b000) next_state = DONE;
                else                     next_state = COLLECT;
            end

            DONE: begin
                if (rf_consume) next_state = IDLE;
            end

            default: next_state = IDLE;
        endcase
    end

    // -------------------------
    // State + request latch + operand capture
    // -------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= IDLE;
            pending   <= 3'b000;

            wid_q     <= '0;
            rs1_q     <= '0;
            rs2_q     <= '0;
            rd_q      <= '0;
            use_q     <= 3'b000;

            bank_rs1_q<= '0; bank_rs2_q<= '0; bank_rd_q <= '0;
            idx_rs1_q <= '0; idx_rs2_q <= '0; idx_rd_q  <= '0;

            rs1_hold  <= '0;
            rs2_hold  <= '0;
            rd_hold   <= '0;
        end else begin
            state <= next_state;

            // Accept new request only in IDLE
            if (state == IDLE && rf_start) begin
                wid_q      <= op_warp_id;
                rs1_q      <= rs1;
                rs2_q      <= rs2;
                rd_q       <= rd_old;
                use_q      <= sel_use_mask;

                bank_rs1_q <= bank_of(rs1);
                bank_rs2_q <= bank_of(rs2);
                bank_rd_q  <= bank_of(rd_old);

                idx_rs1_q  <= idx_of(rs1);
                idx_rs2_q  <= idx_of(rs2);
                idx_rd_q   <= idx_of(rd_old);

                pending    <= sel_use_mask;

                // optional: clear holds for unused (not necessary, but sim-clean)
                // rs1_hold <= '0; rs2_hold <= '0; rd_hold <= '0;
            end
            else if (state == COLLECT) begin
                // capture operands served THIS cycle
                if (serve_rs1) rs1_hold <= mem[wid_q][bank_rs1_q][idx_rs1_q];
                if (serve_rs2) rs2_hold <= mem[wid_q][bank_rs2_q][idx_rs2_q];
                if (serve_rd)  rd_hold  <= mem[wid_q][bank_rd_q ][idx_rd_q ];

                pending <= pending_n;
            end
            else if (state == DONE && rf_consume) begin
                pending <= 3'b000;
            end
        end
    end

    // -------------------------
    // Outputs (stable in DONE)
    // -------------------------
    always_comb begin
        V1_vec      = '0;
        V2_vec      = '0;
        Vrd_old_vec = '0;

        if (rf_ready) begin
            if (use_q[0]) V1_vec      = rs1_hold;
            if (use_q[1]) V2_vec      = rs2_hold;
            if (use_q[2]) Vrd_old_vec = rd_hold;
        end
    end

    // -------------------------
    // Writeback
    // -------------------------
    always_ff @(posedge clk) begin
        if (wb_valid) begin
            logic [BANK_W-1:0] b;
            logic [IDX_W-1:0]  i;

            b = bank_of(wb_rd);
            i = idx_of(wb_rd);

            for (int l=0; l<LANES; l++) begin
                if (wb_mask[l]) mem[wb_warp_id][b][i][l] <= wb_data[l];
            end
        end
    end

endmodule





module Performance_Monitor#(
    parameter int STALL_W   = 4
)(
  input logic clk, rst_n,
  input logic clear,

  input logic do_issue_id,
  input logic wb_valid_mux,
  input logic sel_valid,
  input logic issue_ok_sel,
  input logic replay,
  
  //---- PC_Control branch outs ----
  input logic branch_sel,
  input logic pc_taken,
    
  input logic ex_valid,
  
  //---- execution units queues ----
  input logic ex_q_enq_vaild,
  input logic lsq_q_enq_vaild,
  
  //---- write-back unit ----
  input logic wb_arb_valid,
  
  //---- Warp_control stall outs ----
  input logic stall_valid,
  input logic [STALL_W-1:0] stall_reason,
  
  //---- from LSU and it's replay queue ----
  input logic replay_push,
  input logic replay_pop,
  input logic cpl_valid //LSU completed
);

  //---- STALL REASONS ----
  //  1 = FLUSH
  //  2 = SCOREBOARD/RF
  //  3 = BR_TAKEN_EVENT (technically it's an event, not stall)
  //  4 = LSU_PENDING
  //  5 = LSQ_FULL
  //  6 = EXQ_FULL
  //  7 = ID_NOT_READY


  // combinational flags
  logic stall_selected_blocked;
  logic stall_scoreboard;        // daha doğru bir RF/scoreboard göstergesi

  assign stall_selected_blocked = sel_valid && !do_issue_id;
  assign stall_scoreboard       = sel_valid && !issue_ok_sel; // WARP hazard / busy
  
  logic [63:0] replay_depth_peak;

  // counters
  logic [63:0] cyc, inst_issued, ex_cnt, wb_cnt, replay_push_cnt, replay_pop_cnt;
  logic [63:0] stall_selected_blocked_cnt, stall_scoreboard_cnt, replay_cnt;
  logic [63:0] branch_taken_cnt, ex_enq_cnt, lsq_enq_cnt;
  logic [63:0] lsu_cpl_cnt;

  // 16-entry stall reason counters (0..15)
  logic [63:0] stall_cnt [0:(1<<STALL_W)-1];

  integer i;

  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      cyc                       <= 64'd0;
      inst_issued               <= 64'd0;
      ex_cnt                    <= 64'd0;
      wb_cnt                    <= 64'd0;

      stall_selected_blocked_cnt<= 64'd0;
      stall_scoreboard_cnt      <= 64'd0;
      replay_cnt                <= 64'd0;

      replay_push_cnt           <= 64'd0;
      replay_pop_cnt            <= 64'd0;
      replay_depth_peak         <= 64'd0;

      for (i = 0; i < (1<<STALL_W); i++) begin
        stall_cnt[i] <= 64'd0;
      end

    end else begin
      cyc <= cyc + 64'd1;

      if (do_issue_id)  inst_issued <= inst_issued + 64'd1; //instr began
      if (ex_valid)     ex_cnt      <= ex_cnt + 64'd1;        
      if (wb_arb_valid) wb_cnt      <= wb_cnt + 64'd1; //instr completed

      if (stall_selected_blocked) stall_selected_blocked_cnt <= stall_selected_blocked_cnt + 64'd1;
      if (stall_scoreboard)       stall_scoreboard_cnt       <= stall_scoreboard_cnt + 64'd1;  //stall hazzard
      if (replay)                 replay_cnt                 <= replay_cnt + 64'd1;  //replay (can be many stall reason)

      if (branch_sel && pc_taken) branch_taken_cnt++; 
      
      if(ex_q_enq_vaild) ex_enq_cnt++;
      if(lsq_q_enq_vaild) lsq_enq_cnt++;
       
      // stall reason (single increment)
      if (stall_valid) begin
        stall_cnt[stall_reason] <= stall_cnt[stall_reason] + 64'd1;
      end

      // replay push/pop counters
      if (replay_pop)  replay_pop_cnt  <= replay_pop_cnt  + 64'd1;
      if (replay_push) replay_push_cnt <= replay_push_cnt + 64'd1;

      // replay depth peak (handles push/pop same cycle)
      if (replay_push || replay_pop) begin
        logic [63:0] next_depth;
        next_depth = (replay_push_cnt + (replay_push ? 64'd1 : 64'd0))
                   - (replay_pop_cnt  + (replay_pop  ? 64'd1 : 64'd0));

        if (replay_depth_peak < next_depth) begin
          replay_depth_peak <= next_depth;
        end
      end

      if (clear) begin
        inst_issued                <= 64'd0;
        ex_cnt                     <= 64'd0;
        wb_cnt                     <= 64'd0;
        stall_selected_blocked_cnt <= 64'd0;
        stall_scoreboard_cnt       <= 64'd0;
        replay_cnt                 <= 64'd0;

        replay_push_cnt            <= 64'd0;
        replay_pop_cnt             <= 64'd0;
        replay_depth_peak          <= 64'd0;

        for (i = 0; i < (1<<STALL_W); i++) begin
          stall_cnt[i] <= 64'd0;
        end
        // cyc genelde clear edilmez
      end
    end
  end

endmodule


// ============================================================
// LSQ_Queue.sv
// - Entry taşıyan queue + tag
// - Oldest-first ISSUE (head'den başlayıp scan)
// - In-order RETIRE (sadece head done ise)
// - issued/done bitleri slot başına tutulur
// ============================================================
module LSQ_Queue #(
    parameter int  DEPTH = 8,                  // entries (>=1)
    parameter int  ALMOST_FULL_TH = DEPTH-1,
    parameter type T = lsu_types_pkg::lsq_entry_t
)(
    input  logic          clk,
    input  logic          rst_n,

    // -------------------------
    // ENQUEUE (producer)
    // -------------------------
    input  logic          enq_valid,
    output logic          enq_ready,
    input  T              enq_data,

    // -------------------------
    // ISSUE (to LSU front-end)
    // -------------------------
    output logic          iss_valid,
    input  logic          iss_ready,
    output T              iss_data,
    output logic [$clog2(DEPTH)-1:0] iss_tag,

    // -------------------------
    // COMPLETE (from LSU back-end)
    // -------------------------
    input  logic          cpl_valid,
    input  logic [$clog2(DEPTH)-1:0] cpl_tag,

    // -------------------------
    // RETIRE (optional visibility)
    // -------------------------
    output logic          ret_valid,
    input  logic          ret_ready,
    output T              ret_data,

    input  logic              flush,

    output logic          almost_full,
    output logic          full,
    output logic          empty
);

    // -------------------------
    // widths
    // -------------------------
    localparam int PTR_W = (DEPTH <= 1) ? 1 : $clog2(DEPTH);
    localparam int CNT_W = $clog2(DEPTH + 1);

    // -------------------------
    // storage + per-entry bits
    // -------------------------
    T                 mem    [0:DEPTH-1];
    logic [DEPTH-1:0] vld;       // slot allocated
    logic [DEPTH-1:0] issued;    // already sent to LSU
    logic [DEPTH-1:0] done;      // completed by LSU

    logic [PTR_W-1:0] head_ptr;  // oldest
    logic [PTR_W-1:0] tail_ptr;  // next allocate
    logic [CNT_W-1:0] count;

    // -------------------------
    // helper: wrap increment
    // -------------------------
    function automatic logic [PTR_W-1:0] inc_ptr(input logic [PTR_W-1:0] p);
        if (DEPTH <= 1) inc_ptr = '0;
        else if (p == (DEPTH-1)) inc_ptr = '0;
        else inc_ptr = p + {{(PTR_W-1){1'b0}},1'b1};
    endfunction

    // -------------------------
    // flags (pure comb)
    // -------------------------
    always_comb begin
        empty       = (count == 0);
        full        = (count == DEPTH[CNT_W-1:0]);
        almost_full = (count >= ALMOST_FULL_TH[CNT_W-1:0]);
    end

    // -------------------------
    // ENQ ready (simple)
    // -------------------------
    assign enq_ready = !full;

    wire enq_fire = enq_valid && enq_ready;

    // -------------------------
    // ISSUE pick: oldest-first scan from head_ptr
    // eligible = vld && !issued
    // -------------------------
    logic             have_iss;
    logic [PTR_W-1:0] pick_idx;

    always_comb begin
        int unsigned tmp;
        int unsigned idx;
    
        have_iss = 1'b0;
        pick_idx = head_ptr;
    
        for (int unsigned k = 0; k < DEPTH; k++) begin
            tmp = int'(head_ptr) + k;
            idx = (tmp >= DEPTH) ? (tmp - DEPTH) : tmp;
    
            if (!have_iss && vld[idx] && !issued[idx]) begin
                have_iss = 1'b1;
                pick_idx = idx[PTR_W-1:0];
            end
        end
    
        iss_valid = have_iss;
        iss_tag   = pick_idx;
        iss_data  = have_iss ? mem[pick_idx] : '0;
    end

    wire iss_fire = iss_valid && iss_ready;

    // -------------------------
    // RETIRE: only head, only if done
    // -------------------------
    always_comb begin
        ret_valid = 1'b0;
        ret_data  = '0;

        if (!empty && vld[head_ptr] && done[head_ptr]) begin
            ret_valid = 1'b1;
            ret_data  = mem[head_ptr];
        end
    end

    wire ret_fire = ret_valid && ret_ready;

    // -------------------------
    // sequential update
    // -------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head_ptr <= '0;
            tail_ptr <= '0;
            count    <= '0;

            vld      <= '0;
            issued   <= '0;
            done     <= '0;
        end else begin
            if (flush) begin
              head_ptr <= '0;
              tail_ptr <= '0;
              count    <= '0;
              vld      <= '0;
              issued   <= '0;
              done     <= '0;
            end else begin
                // -------------------------
                // ENQUEUE: allocate at tail_ptr
                // -------------------------
                if (enq_fire) begin
                    mem[tail_ptr]    <= enq_data;
                    vld[tail_ptr]    <= 1'b1;
                    issued[tail_ptr] <= 1'b0;
                    done[tail_ptr]   <= 1'b0;
                    tail_ptr         <= inc_ptr(tail_ptr);
                end

                // -------------------------
                // ISSUE: mark slot as issued
                // -------------------------
                if (iss_fire) begin
                    issued[iss_tag] <= 1'b1;
                end

                // -------------------------
                // COMPLETE: mark slot done
                // -------------------------
                if (cpl_valid) begin
                    done[cpl_tag] <= 1'b1;
                end

                // -------------------------
                // RETIRE: free head slot (in-order)
                // -------------------------
                if (ret_fire) begin
                    vld[head_ptr]    <= 1'b0;
                    issued[head_ptr] <= 1'b0;
                    done[head_ptr]   <= 1'b0;
                    head_ptr         <= inc_ptr(head_ptr);
                end

                // -------------------------
                // count update (enq and ret can happen same cycle)
                // NOTE: iss/cpl don't change occupancy
                // -------------------------
                unique case ({enq_fire, ret_fire})
                    2'b10: count <= count + 1'b1;
                    2'b01: count <= count - 1'b1;
                    default: count <= count; // 00 or 11
                endcase
            end
        end
    end

endmodule



module Load_Store_Unit #(
    parameter int NUM_WARPS   = gpu_cfg_pkg::NUM_WARPS,
    parameter int LANES       = gpu_cfg_pkg::LANES,
    parameter int VREGS       = gpu_cfg_pkg::VREGS,
    parameter int DATA_W      = gpu_cfg_pkg::DATA_W,
    parameter int DEPTH_LSQ   = 8,
    parameter int REPLAY_LEN  = 2
)(
    input  logic clk,
    input  logic rst_n,

    // ----from LSQ-----
    input  logic                                iss_valid,
    output logic                                iss_ready,
    input  lsu_types_pkg::lsq_entry_t           iss_data,
    input  logic [$clog2(DEPTH_LSQ)-1:0]        iss_tag,

    // ----complete I/O's----
    output logic                                cpl_valid,
    output logic [$clog2(DEPTH_LSQ)-1:0]        cpl_tag,

    // ----DMEM I/O's----
    output logic                                dmem_write_enable,
    output logic                                dmem_req_valid,
    input  logic                                dmem_req_ready,
    output logic [DATA_W-1:0]                   dmem_addr,        // lane0 addr (baseline)
    output logic [LANES-1:0][DATA_W-1:0]        dmem_wdata_vec,
    input  logic [LANES-1:0][DATA_W-1:0]        dmem_rdata_vec,

    // ----Write back (load only)----
    output logic                                lsu_wb_valid,
    output logic [$clog2(NUM_WARPS)-1:0]        lsu_wb_wid,
    output logic [$clog2(VREGS)-1:0]            lsu_wb_rd,
    output logic [LANES-1:0]                    lsu_wb_mask,
    output logic [LANES-1:0][DATA_W-1:0]        lsu_wb_data_vec,

    input  logic                                flush,
    
    //----replay events for perf monitor----
    output logic                                replay_pop,
    output logic                                replay_push
);

    localparam int RPLY_PTR_W = (REPLAY_LEN <= 1) ? 1 : $clog2(REPLAY_LEN);
    localparam int RPLY_CNT_W = $clog2(REPLAY_LEN + 1);

    typedef struct packed {
        logic [$clog2(DEPTH_LSQ)-1:0]         tag;
        logic [$clog2(NUM_WARPS)-1:0]         wid;
        logic                                is_load;
        logic                                is_store;
        logic [$clog2(VREGS)-1:0]             rd;
        logic [LANES-1:0]                     mask;
        logic [DATA_W-1:0]                    addr0;      // baseline: addr_vec[0]
        logic [LANES-1:0][DATA_W-1:0]         store_vec;
    } rply_entry_t;

    rply_entry_t RPLY_Q [0:REPLAY_LEN-1];

    logic [RPLY_PTR_W-1:0] rply_enq_ptr, rply_deq_ptr;
    logic [RPLY_CNT_W-1:0] rply_q_in_use;

    function automatic rply_entry_t pack_from_lsq(
        input lsu_types_pkg::lsq_entry_t lsq,
        input logic [$clog2(DEPTH_LSQ)-1:0] tag
    );
        rply_entry_t e;
        e.tag       = tag;
        e.wid       = lsq.warp_id;
        e.is_load   = lsq.is_load;
        e.is_store  = lsq.is_store;
        e.rd        = lsq.rd;
        e.mask      = lsq.mask;
        e.addr0     = lsq.addr_vec[0];
        e.store_vec = lsq.store_vec;
        return e;
    endfunction

    // -------------------------
    // Status / selection
    // -------------------------
    wire rply_nonempty = (rply_q_in_use != 0);
    wire rply_full     = (rply_q_in_use == REPLAY_LEN[RPLY_CNT_W-1:0]);

    wire take_from_replay = rply_nonempty;

    assign iss_ready = (!flush) && (!take_from_replay) && (!rply_full);
    wire lsq_fire    = iss_valid && iss_ready;

    // request valid: replay varsa replay, yoksa LSQ fire ile gelen iş
    assign dmem_req_valid = (!flush) && (take_from_replay || lsq_fire);
    wire dmem_fire        = dmem_req_valid && dmem_req_ready;

    // current request payload
    rply_entry_t cur_e;
    always_comb begin
        cur_e = '0;
        if (take_from_replay) cur_e = RPLY_Q[rply_deq_ptr];
        else if (lsq_fire)    cur_e = pack_from_lsq(iss_data, iss_tag);
    end

    // -------------------------
    // DMEM request drive (comb)
    // -------------------------
    always_comb begin
        dmem_addr         = cur_e.addr0;
        dmem_write_enable = cur_e.is_store;
        dmem_wdata_vec    = cur_e.store_vec;
    end

    // -------------------------
    // Response registers (1-cycle latency)
    // -------------------------
    logic resp_valid;
    logic [$clog2(DEPTH_LSQ)-1:0]         resp_tag;
    logic                                resp_is_load;
    logic [$clog2(NUM_WARPS)-1:0]         resp_wid;
    logic [$clog2(VREGS)-1:0]             resp_rd;
    logic [LANES-1:0]                     resp_mask;
    logic [LANES-1:0][DATA_W-1:0]         resp_data;
    

    // Outputs are ONLY from resp regs
    always_comb begin
        cpl_valid = resp_valid;
        cpl_tag   = resp_tag;

        lsu_wb_valid    = resp_valid && resp_is_load;
        lsu_wb_wid      = resp_wid;
        lsu_wb_rd       = resp_rd;
        lsu_wb_mask     = resp_mask;
        lsu_wb_data_vec = resp_data;
    end
    
    // ---- Replay pop if replay head successfully issued to DMEM
    assign replay_pop = take_from_replay && dmem_fire;

    // ---- Replay push if LSQ fire happened but DMEM not ready (and we are not taking replay)
    assign replay_push = (!take_from_replay) && lsq_fire && (!dmem_req_ready);
    
    // -------------------------
    // Sequential: replay queue + resp capture
    // -------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rply_enq_ptr  <= '0;
            rply_deq_ptr  <= '0;
            rply_q_in_use <= '0;

            resp_valid    <= 1'b0;
            resp_tag      <= '0;
            resp_is_load  <= 1'b0;
            resp_wid      <= '0;
            resp_rd       <= '0;
            resp_mask     <= '0;
            resp_data     <= '0;
        end else if (flush) begin
            rply_enq_ptr  <= '0;
            rply_deq_ptr  <= '0;
            rply_q_in_use <= '0;

            resp_valid    <= 1'b0;
        end else begin
            // default: one-cycle pulse
            resp_valid <= 1'b0;

            // pop
            if (replay_pop) begin
                if (rply_deq_ptr == (REPLAY_LEN-1)) rply_deq_ptr <= '0;
                else                                rply_deq_ptr <= rply_deq_ptr + 1'b1;
            end

            // push
            if (replay_push) begin
                RPLY_Q[rply_enq_ptr] <= pack_from_lsq(iss_data, iss_tag);
                if (rply_enq_ptr == (REPLAY_LEN-1)) rply_enq_ptr <= '0;
                else                                rply_enq_ptr <= rply_enq_ptr + 1'b1;
            end

            // count update (pop/push can happen same cycle)
            unique case ({replay_push, replay_pop})
                2'b10: rply_q_in_use <= rply_q_in_use + 1'b1;
                2'b01: rply_q_in_use <= rply_q_in_use - 1'b1;
                default: rply_q_in_use <= rply_q_in_use; // 00 or 11
            endcase

            // ---- Capture response when DMEM accepted a request ----- 
            // (0-latency DMEM read assumed; resp makes it 1-cycle outward)
            if (dmem_fire) begin
                resp_valid   <= 1'b1;
                resp_tag     <= cur_e.tag;
                resp_is_load <= cur_e.is_load;
                resp_wid     <= cur_e.wid;
                resp_rd      <= cur_e.rd;
                resp_mask    <= cur_e.mask;
                resp_data    <= dmem_rdata_vec;
            end
        end
    end

endmodule



module WB_arb #(
  parameter int NUM_WARPS = gpu_cfg_pkg::NUM_WARPS,
  parameter int LANES     = gpu_cfg_pkg::LANES,
  parameter int VREGS     = gpu_cfg_pkg::VREGS,
  parameter int DATA_W    = gpu_cfg_pkg::DATA_W
)(
  input  logic clk,
  input  logic rst_n,
  input  logic flush,

  // ALU WB in
  input  logic                         alu_wb_valid,
  input  logic [$clog2(NUM_WARPS)-1:0]  alu_wb_wid,
  input  logic [$clog2(VREGS)-1:0]      alu_wb_rd,
  input  logic [LANES-1:0]              alu_wb_mask,
  input  logic [LANES-1:0][DATA_W-1:0]  alu_wb_data,

  // LSU WB in
  input  logic                         lsu_wb_valid,
  input  logic [$clog2(NUM_WARPS)-1:0]  lsu_wb_wid,
  input  logic [$clog2(VREGS)-1:0]      lsu_wb_rd,
  input  logic [LANES-1:0]              lsu_wb_mask,
  input  logic [LANES-1:0][DATA_W-1:0]  lsu_wb_data,

  // WB out (to fanout)
  output logic                         wb_out_valid,
  output logic [$clog2(NUM_WARPS)-1:0]  wb_out_wid,
  output logic [$clog2(VREGS)-1:0]      wb_out_rd,
  output logic [LANES-1:0]              wb_out_mask,
  output logic [LANES-1:0][DATA_W-1:0]  wb_out_data
);

  typedef struct packed {
    logic [$clog2(NUM_WARPS)-1:0]  wid;
    logic [$clog2(VREGS)-1:0]      rd;
    logic [LANES-1:0]              mask;
    logic [LANES-1:0][DATA_W-1:0]  data;
  } wb_pkt_t;

  wb_pkt_t skid_buf;
  logic    skid_in_use;

  // convenience packs
  wb_pkt_t alu_pkt, lsu_pkt;
  always_comb begin
    alu_pkt.wid  = alu_wb_wid;
    alu_pkt.rd   = alu_wb_rd;
    alu_pkt.mask = alu_wb_mask;
    alu_pkt.data = alu_wb_data;

    lsu_pkt.wid  = lsu_wb_wid;
    lsu_pkt.rd   = lsu_wb_rd;
    lsu_pkt.mask = lsu_wb_mask;
    lsu_pkt.data = lsu_wb_data;
  end

  // output regs
  wb_pkt_t out_pkt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n || flush) begin
      skid_in_use  <= 1'b0;
      skid_buf     <= '0;
      out_pkt      <= '0;
      wb_out_valid <= 1'b0;
    end else begin
      wb_out_valid <= 1'b0; // default pulse

      // Priority: LSU > skid > ALU
      if (lsu_wb_valid) begin
        out_pkt      <= lsu_pkt;
        wb_out_valid <= 1'b1;

        // if ALU also valid, capture ALU into skid (only if skid empty)
        if (alu_wb_valid) begin
          skid_buf    <= alu_pkt;
          skid_in_use <= 1'b1;
        end
      end
      else if (skid_in_use) begin
        out_pkt      <= skid_buf;
        wb_out_valid <= 1'b1;
        skid_in_use  <= 1'b0;
      end
      else if (alu_wb_valid) begin
        out_pkt      <= alu_pkt;
        wb_out_valid <= 1'b1;
      end
    end
  end

  // fanout-facing outputs
  always_comb begin
    wb_out_wid  = out_pkt.wid;
    wb_out_rd   = out_pkt.rd;
    wb_out_mask = out_pkt.mask;
    wb_out_data = out_pkt.data;
  end

endmodule




module Issue_Control #(
    parameter int NUM_WARPS = gpu_cfg_pkg::NUM_WARPS,
    parameter int LANES     = gpu_cfg_pkg::LANES,
    parameter int VREGS     = gpu_cfg_pkg::VREGS,
    parameter int DATA_W    = gpu_cfg_pkg::DATA_W,
    parameter int PC_W      = gpu_cfg_pkg::PC_W,
    parameter int OPC_W     = 6,
    parameter int STALL_W   = 4
)(
    input  logic                         clk,
    input  logic                         rst_n,

    // -------------------------
    // Scheduler / select
    // -------------------------
    input  logic                         scheduler_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  sel_warp_id,
    
    //--------------------------
    //branch input
    //--------------------------
    
     input logic        pc_taken,
     input logic [PC_W-1:0] pc_next_in,

    // -------------------------
    // Ready / hazard
    // -------------------------
    input  logic                         issue_ok_sel,      // scoreboard+rf ready vb.
    input  logic                         id_ready,          // (varsa) ID stage accept
    input  logic                         ex_q_enq_ready,
    input  logic                         lsq_q_enq_ready,

    // -------------------------
    // Decode classification
    // -------------------------
    input  logic                         branch_sel,
    input  logic                         is_jump,
    input  logic                         is_beq,

    input  logic                         reg_write_enable,
    input  logic                         mem_write_enable,
    input  logic                         write_back_sel,     // load? (reg_we && wb_sel)

    input  logic [OPC_W-1:0]             op_code,

    input  logic [$clog2(VREGS)-1:0]     rd,
    input  logic [$clog2(VREGS)-1:0]     rs1,
    input  logic [$clog2(VREGS)-1:0]     rs2,

    input  logic [LANES-1:0]             mask,
    input  logic [DATA_W-1:0]            imm,                // imm_value (32-bit)

    // -------------------------
    // Operands (vectors)
    // -------------------------
    input  logic [LANES-1:0][DATA_W-1:0] V1_vec,
    input  logic [LANES-1:0][DATA_W-1:0] V2_eff_vec,
    input  logic [LANES-1:0][DATA_W-1:0] Vold_vec,
    input  logic [LANES-1:0][DATA_W-1:0] store_vec,

    // -------------------------
    // LSU/LSQ status
    // -------------------------
    input  logic                         lsq_almost_full,
    input  logic                         lsq_full,
    input  logic [NUM_WARPS-1:0]         pending_LSU, 

    // -------------------------
    // Flush info (TOP üretir)
    // -------------------------
    input  logic                         flush_out,
    input  logic [$clog2(NUM_WARPS)-1:0]  flush_id,

    // -------------------------
    // To EXQ
    // -------------------------
    output logic                         ex_q_enq_valid,
    output ex_types_pkg::ex_entry_t      ex_q_enq_data,

    // -------------------------
    // To LSQ
    // -------------------------
    output logic                         lsq_q_enq_valid,
    output lsu_types_pkg::lsq_entry_t    lsq_q_enq_data,

    // -------------------------
    // Issue commit to WARP_State/Scoreboard
    // -------------------------
    output logic                         do_issue_id,
    output logic                         issue_writes_rd,
    output logic [$clog2(VREGS)-1:0]     issue_rd,
    output logic [$clog2(NUM_WARPS)-1:0] issue_w_id,

    // -------------------------
    // PC update interface (opsiyonel: çoğunlukla PC_control_unit üretir)
    // -------------------------
    output logic                         pc_update_valid,
    output logic [PC_W-1:0]              pc_next,

    // -------------------------
    // Stall / reason (perf/debug)
    // -------------------------
    output logic                         stall_valid,
    output logic [STALL_W-1:0]           stall_reason,        // enum için width
    output logic [$clog2(NUM_WARPS)-1:0] stall_w_id
);

wire is_load = reg_write_enable && write_back_sel;  // load = reg_we + wb_sel
wire is_store = mem_write_enable;                   // store
wire is_mem = is_load || is_store;

wire lsu_pending_sel = pending_LSU[sel_warp_id];

always_ff @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    // default reset
    ex_q_enq_valid   <= 1'b0;
    lsq_q_enq_valid  <= 1'b0;

    do_issue_id      <= 1'b0;
    issue_writes_rd  <= 1'b0;
    issue_rd         <= '0;
    issue_w_id       <= '0;

    pc_update_valid  <= 1'b0;
    pc_next          <= '0;

    stall_valid      <= 1'b0;
    stall_reason     <= '0;
    stall_w_id       <= '0;

    ex_q_enq_data    <= '0;
    lsq_q_enq_data   <= '0;

  end else begin
    // -------------------------
    // defaults (pulse-style)
    // -------------------------
    ex_q_enq_valid   <= 1'b0;
    lsq_q_enq_valid  <= 1'b0;

    do_issue_id      <= 1'b0;
    issue_writes_rd  <= 1'b0;
    issue_rd         <= '0;
    issue_w_id       <= sel_warp_id;

    pc_update_valid  <= 1'b0;
    pc_next          <= pc_next;     // hold

    stall_valid      <= 1'b0;
    stall_reason     <= '0;
    stall_w_id       <= sel_warp_id;

    // -------------------------
    // global flush cycle: enqueue yapma
    // -------------------------
    if (flush_out) begin
      stall_valid  <= 1'b1;
      stall_reason <= 4'd1; // STALL_FLUSH (enum'u sen belirle)
    end
    else if (!scheduler_valid) begin
      // aday yok
      stall_valid <= 1'b0;
    end
    else if (!issue_ok_sel) begin
      // hazard / rf not ready
      stall_valid  <= 1'b1;
      stall_reason <= 4'd2; // STALL_SCOREBOARD/RF
    end
    else if (!id_ready) begin
      // ID stage (RF rsp / issue_reg accept) hazır değilse hiçbir şey enqueue etme
      stall_valid  <= 1'b1;
      stall_reason <= 4'd7; // STALL_ID_NOT_READY  
    end
    else if (branch_sel) begin
      // -------------------------
      // BRANCH path: kuyruklara dokunma, PC update yap
      // -------------------------
      do_issue_id     <= 1'b1;
      pc_update_valid <= 1'b1;

      // perf/debug için reason yazmak istersen:
      if (pc_taken) begin
        stall_valid  <= 1'b1;   //1 cycle stall
        stall_reason <= 4'd3;   // BR_TAKEN
      end
    end
    else if (is_mem) begin
      // -------------------------
      // MEM path: LSQ enqueue dene
      // -------------------------
      if (lsu_pending_sel) begin
        stall_valid  <= 1'b1;
        stall_reason <= 4'd4; // STALL_LSU_PENDING
      end
      else if (!lsq_q_enq_ready || lsq_full) begin
        stall_valid  <= 1'b1;
        stall_reason <= 4'd5; // STALL_LSQ_FULL
      end
      else begin
        // enqueue LSQ
        lsq_q_enq_valid <= 1'b1;
        lsq_q_enq_data.warp_id  <= sel_warp_id;
        lsq_q_enq_data.is_load  <= is_load;
        lsq_q_enq_data.is_store <= is_store;
        lsq_q_enq_data.rd       <= rd;
        lsq_q_enq_data.mask     <= mask;
        // baseline: addr_vec'ü V1_vec'den türetiliyor;
        lsq_q_enq_data.addr_vec <= V1_vec;
        lsq_q_enq_data.store_vec<= store_vec;

        if (lsq_q_enq_ready) begin
          do_issue_id     <= 1'b1;
          issue_w_id      <= sel_warp_id;
          issue_rd        <= rd;
          issue_writes_rd <= is_load; // sadece load rd yazar
        end
      end
    end
    else begin
      // -------------------------
      // ALU path: EXQ enqueue dene
      // -------------------------
      if (!ex_q_enq_ready) begin
        stall_valid  <= 1'b1;
        stall_reason <= 4'd6; // STALL_EXQ_FULL
      end
      else begin
        ex_q_enq_valid <= 1'b1;
        ex_q_enq_data.warp_id <= sel_warp_id;
        ex_q_enq_data.op_code <= op_code;
        ex_q_enq_data.rd      <= rd;
        ex_q_enq_data.mask    <= mask;
        ex_q_enq_data.V1_vec  <= V1_vec;
        ex_q_enq_data.V2_vec  <= V2_eff_vec;
        ex_q_enq_data.Vold_vec<= Vold_vec;

        if (ex_q_enq_ready) begin
          do_issue_id     <= 1'b1;
          issue_w_id      <= sel_warp_id;
          issue_rd        <= rd;
          issue_writes_rd <= reg_write_enable;
        end
      end
    end
  end
end

endmodule

