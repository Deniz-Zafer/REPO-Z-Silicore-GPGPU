`timescale 1ns / 1ps

// ============================================================
// GPU Core V0.3
// - Mask'li ALL-reduce BEQ
// - Unconditional JUMP
// - PC update logic in PC_control_unit
// - WARP hazard optimized via sel_use_mask (3-bit)
// ============================================================

module GPU_Core #(
    parameter int NUM_WARPS = 4,
    parameter int LANES     = 8,
    parameter int VREGS     = 8,
    parameter int DATA_W    = 32,
    parameter int PC_W      = 32
)(
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
    assign ex_ready   = 1'b1;

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
        .do_issue      (do_issue_id),
        .is_jump       (is_jump_wire),
        .is_beq        (is_beq_wire),

        .pc_cur        (pc_sel),
        .branch_offset (branch_offset_wire),

        .mask          (mask_sel),   // warp active mask
        .V1_vec        (V1_vec),
        .V2_vec        (V2_vec),

        .pc_update_valid(pc_update_valid_wire),
        .pc_next       (pc_next_wire)
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
module WARP #(
    parameter int NUM_WARPS = 4,
    parameter int LANES     = 8,
    parameter int VREGS     = 8,
    parameter int DATA_W    = 32,
    parameter int PC_W      = 32
)(
    input  logic clk,
    input  logic rst_n,

    input  logic [$clog2(NUM_WARPS)-1:0] sel_warp_id,
    output logic [PC_W-1:0]              pc_sel,
    output logic [LANES-1:0]             mask_sel,
    output logic [NUM_WARPS-1:0]         can_issue,

    input  logic [$clog2(NUM_WARPS)-1:0] op_warp_id,
    input  logic [$clog2(VREGS)-1:0]     rs1,
    input  logic [$clog2(VREGS)-1:0]     rs2,
    input  logic [$clog2(VREGS)-1:0]     rd_old,

    output logic [LANES-1:0][DATA_W-1:0] V1_vec,
    output logic [LANES-1:0][DATA_W-1:0] V2_vec,
    output logic [LANES-1:0][DATA_W-1:0] Vrd_old_vec,
    output logic [LANES-1:0][DATA_W-1:0] store_vec,

    input  logic                          wb_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  wb_warp_id,
    input  logic [$clog2(VREGS)-1:0]      wb_rd,
    input  logic [LANES-1:0]              wb_mask,
    input  logic [LANES-1:0][DATA_W-1:0]  wb_data,

    input  logic                          issue_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  issue_warp_id,
    input  logic                          issue_writes_rd,
    input  logic [$clog2(VREGS)-1:0]      issue_rd,

    output logic [NUM_WARPS-1:0][VREGS-1:0] busy,

    input  logic [2:0]                    sel_use_mask,
    output logic                          issue_ok_sel,

    input  logic                          stall_set_valid,
    input  logic                          stall_clr_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  stall_set_warp_id,
    input  logic [$clog2(NUM_WARPS)-1:0]  stall_clr_warp_id,

    input  logic                          pc_update_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  pc_update_warp_id,
    input  logic [PC_W-1:0]               pc_next
);

    logic [NUM_WARPS-1:0][PC_W-1:0]                         pc_bank;
    logic [NUM_WARPS-1:0][LANES-1:0]                        mask_bank;
    logic [NUM_WARPS-1:0]                                   stalled;
    logic [NUM_WARPS-1:0][VREGS-1:0]                        busy_bank;
    logic [NUM_WARPS-1:0][VREGS-1:0][LANES-1:0][DATA_W-1:0] V;

    assign busy     = busy_bank;

    assign pc_sel   = pc_bank[sel_warp_id];
    assign mask_sel = mask_bank[sel_warp_id];

    // reads
    assign V1_vec      = V[op_warp_id][rs1];
    assign V2_vec      = V[op_warp_id][rs2];
    assign Vrd_old_vec = V[op_warp_id][rd_old];
    assign store_vec   = V[op_warp_id][rs2];

    // can_issue: stall yoksa aday
    genvar w;
    generate
        for (w = 0; w < NUM_WARPS; w++) begin : GEN_CAN_ISSUE
            assign can_issue[w] = ~stalled[w];
        end
    endgenerate

    // hazard check (min LUT)
    wire rs1_ok   = (!sel_use_mask[0]) || (~busy_bank[sel_warp_id][rs1]);
    wire rs2_ok   = (!sel_use_mask[1]) || (~busy_bank[sel_warp_id][rs2]);
    wire rdold_ok = (!sel_use_mask[2]) || (~busy_bank[sel_warp_id][rd_old]);

    assign issue_ok_sel =
        (~stalled[sel_warp_id]) &&
        rs1_ok && rs2_ok && rdold_ok;

    integer wi, li;

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (wi = 0; wi < NUM_WARPS; wi++) begin
                pc_bank[wi]    <= '0;
                stalled[wi]    <= 1'b0;
                busy_bank[wi]  <= '0;
                mask_bank[wi]  <= {LANES{1'b1}};
            end

            for (wi = 0; wi < NUM_WARPS; wi++) begin
                for (int r = 0; r < VREGS; r++) begin
                    for (li = 0; li < LANES; li++) begin
                        V[wi][r][li] <= '0;
                    end
                end
            end
        end else begin
            // PC update
            if (pc_update_valid) begin
                pc_bank[pc_update_warp_id] <= pc_next;
            end

            // stall set/clear
            if (stall_set_valid) stalled[stall_set_warp_id] <= 1'b1;
            if (stall_clr_valid) stalled[stall_clr_warp_id] <= 1'b0;

            // Issue -> busy set
            if (issue_valid && issue_writes_rd) begin
                busy_bank[issue_warp_id][issue_rd] <= 1'b1;
            end

            // WB -> reg write + busy clear
            if (wb_valid) begin
                for (li = 0; li < LANES; li++) begin
                    if (wb_mask[li]) V[wb_warp_id][wb_rd][li] <= wb_data[li];
                end
                busy_bank[wb_warp_id][wb_rd] <= 1'b0;
            end
        end
    end

endmodule
