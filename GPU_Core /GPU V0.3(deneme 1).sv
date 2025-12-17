`timescale 1ns / 1ps

module GPU_Core #(
    parameter int NUM_WARPS = 4,
    parameter int LANES     = 8,
    parameter int VREGS     = 8,
    parameter int DATA_W    = 32,
    parameter int PC_W      = 32
)(
    input logic clk,
    input logic rst_n
);

    // -------------------------
    // Scheduler <-> WarpBank
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

    logic                          reg_we_wire, mem_we_wire, wb_sel_wire;
    logic                          SV_sel_wire;
    logic                          Imm_sel_wire;
    logic                          branch_sel_wire;

    logic [7:0]                    imm_field_wire;
    logic [31:0]                   imm_value_wire;
    logic [31:0]                   branch_offset_wire;

    // -------------------------
    // Issue_reg wiring
    // -------------------------
    logic                         id_ready;
    logic                         ex_ready;
    logic                         do_issue_id;

    logic                         ex_valid;
    logic                         ex_stall;

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
    // Warp operands
    // -------------------------
    logic [LANES-1:0][DATA_W-1:0]  V1_vec, V2_vec, Vrd_old_vec, store_vec;
    logic [NUM_WARPS-1:0][VREGS-1:0] busy;
    logic                          issue_ok_sel;

    // rd_old: VFMA accumulate için şimdilik dest kullan
    wire [$clog2(VREGS)-1:0] rd_old_addr = dest_addr;

    // -------------------------
    // Imm broadcast
    // -------------------------
    logic [LANES-1:0][DATA_W-1:0]  V2_eff;

    generate
        for (genvar i = 0; i < LANES; i++) begin : GEN_V2
            assign V2_eff[i] = (Imm_sel_wire) ? imm_value_wire : V2_vec[i];
        end
    endgenerate

    // -------------------------
    // WB mux wires to WARP
    // -------------------------
    logic                          wb_valid_mux;
    logic [$clog2(NUM_WARPS)-1:0]  wb_warp_id_mux;
    logic [$clog2(VREGS)-1:0]      wb_rd_mux;
    logic [LANES-1:0]              wb_mask_mux;
    logic [LANES-1:0][DATA_W-1:0]  wb_data_mux;

    // -------------------------
    // ALU output
    // -------------------------
    logic                         alu_valid_out;
    logic [LANES-1:0][DATA_W-1:0] alu_vout;
    logic [$clog2(VREGS)-1:0]     alu_rd_out;

    // -------------------------
    // DMEM read data
    // -------------------------
    logic [LANES-1:0][DATA_W-1:0]  mem_rdata_wire;

    // -------------------------
    // Handshake policy (şimdilik stall yok)
    // -------------------------
    assign ex_ready   = 1'b1;

    // scheduler seçti + hazard ok + issue_reg alabiliyor
    assign do_issue_id = sched_valid && issue_ok_sel && id_ready;

    // -------------------------
    // Scheduler
    // -------------------------
    Scheduler #(.NUM_WARPS(NUM_WARPS)) sched_inst (
        .clk        (clk),
        .rst_n      (rst_n),
        .can_issue  (can_issue),
        .sel_warp_id(sel_warp_id),
        .sel_valid  (sched_valid)
    );

    // -------------------------
    // IMEM
    // -------------------------
    IMEM imem_inst (
        .clk        (clk),
        .pc_addr     (pc_sel),
        .instruction (instruction_wire)
    );

    // -------------------------
    // Decoder
    // -------------------------
    decoder dec_inst (
        .instr      (instruction_wire),
        .opcode     (opcode_wire),
        .addr_dest  (dest_addr),
        .addr_src1  (src1_addr),
        .addr_src2  (src2_addr),

        .SV_sel     (SV_sel_wire),
        .Imm_sel    (Imm_sel_wire),
        .branch_sel (branch_sel_wire),

        .write_mask (mask_wire),
        .imm_field  (imm_field_wire),

        .is_jump   (is_jump_wire),
        .is_beq    (is_beq_wire),
   
        .o_reg_we   (reg_we_wire),
        .o_mem_we   (mem_we_wire),
        .o_wb_sel   (wb_sel_wire)
    );

    // -------------------------
    // Imm Unit
    // -------------------------
    Imm_Unit imm_inst (
        .imm_field     (imm_field_wire),
        .imm_value     (imm_value_wire),
        .branch_offset (branch_offset_wire)
    );
    
    
    //--------------------------
    //PC Control Unit (Brancher)
    //--------------------------
    logic is_jump_wire, is_beq_wire;
    logic beq_taken_all_wire, beq_taken_scalar_wire;

    PC_control_unit #(
        .PC_W(PC_W),
        .LANES(LANES),
        .DATA_W(DATA_W)
    ) pc_ctrl (
        .do_issue       (do_issue_id),
    
        .is_jump        (is_jump_wire),
        .is_beq         (is_beq_wire),
    
        .pc_cur         (pc_sel),
        .branch_offset  (branch_offset_wire),
    
        .mask           (mask_sel),      // IMPORTANT: warp mask state
        .V1_vec         (V1_vec),
        .V2_vec         (V2_vec),
    
        .beq_taken_all    (beq_taken_all_wire),
        .beq_taken_scalar (beq_taken_scalar_wire),
    
        .pc_update_valid (pc_update_valid_wire),
        .pc_next         (pc_next_wire)
    );


    // -------------------------
    // WARP bank
    // -------------------------
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

        .busy        (busy),
        .issue_ok_sel(issue_ok_sel),

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

    // -------------------------
    // Issue_reg
    // -------------------------
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

    // -------------------------
    // ALU
    // -------------------------
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

    // -------------------------
    // DMEM
    // -------------------------
    DMEM dmem_inst (
        .clk    (clk),
        .mem_we (ex_valid && ex_mem_we),
        .addr   (ex_V1_vec[0]),
        .wdata  (ex_store_vec),
        .rdata  (mem_rdata_wire)
    );

    // -------------------------
    // WB mux (EX stage based)
    // -------------------------
    wire wb_fire = ex_valid && ex_reg_we && !ex_is_branch && !ex_is_store;

    assign wb_valid_mux   = wb_fire;
    assign wb_warp_id_mux = ex_warp_id;
    assign wb_rd_mux      = ex_rd;
    assign wb_mask_mux    = ex_mask;

    assign wb_data_mux    = (ex_wb_sel) ? mem_rdata_wire : alu_vout;

endmodule



//Instruction Memory
module IMEM (
    input  logic        clk,
    input  logic [31:0] pc_addr,    // PC dışarıdan gelir (0, 4, 8...)
    output logic [31:0] instruction // CU'ya giden komut
);

    // 32-bit genişlik, 64 satır derinlik
    logic [31:0] memory [0:63];

    assign instruction = memory[pc_addr[7:2]];

endmodule



module decoder (
    input  logic [31:0] instr,
    
    // Adres/Opcode
    output logic [5:0]  opcode,
    output logic [2:0]  addr_dest,
    output logic [2:0]  addr_src1,
    output logic [2:0]  addr_src2,
    
    output logic        SV_sel,
    output logic        Imm_sel,
    output logic        branch_sel,

    output logic [7:0]  write_mask,
    output logic [7:0]  imm_field,
    
    output logic        is_jump,
    output logic        is_beq,
    
    output logic        o_reg_we,
    output logic        o_mem_we,
    output logic        o_wb_sel
);


    // [31] : SV_sel
    // [30] : Imm_sel
    // [29:26] : op_field
    // [25:23] : dest
    // [22:20] : src1
    // [19:17] : src2
    // [15:8]  : imm_field
    // [7:0]   : write_mask

    logic       sv_bit;
    logic       imm_bit;
    logic [3:0] op_field;

    assign sv_bit      = instr[31];
    assign imm_bit     = instr[30];
    assign op_field    = instr[29:26];

    assign opcode      = {sv_bit, imm_bit, op_field};

    assign addr_dest   = instr[25:23];
    assign addr_src1   = instr[22:20];
    assign addr_src2   = instr[19:17];

    assign imm_field   = instr[15:8];
    assign write_mask  = instr[7:0];

    assign SV_sel      = sv_bit;
    assign Imm_sel     = imm_bit;

    // opcode sabitleri
    localparam [5:0] OPC_VADD   = 6'b10_0000;
    localparam [5:0] OPC_VADDI  = 6'b11_0000;
    localparam [5:0] OPC_VLOAD  = 6'b10_1000;
    localparam [5:0] OPC_VSTORE = 6'b10_1001;

    localparam [5:0] OPC_SADD   = 6'b00_0000;
    localparam [5:0] OPC_SADDI  = 6'b01_0000;

    localparam [5:0] OPC_JUMP   = 6'b01_1110;  
    localparam [5:0] OPC_BRANCH = 6'b01_1111;  

    always_comb begin
        o_reg_we   = 1'b0;
        o_mem_we   = 1'b0;
        o_wb_sel   = 1'b0;
        branch_sel = 1'b0;

        unique case (opcode)
            OPC_VADD,
            OPC_VADDI: begin
                o_reg_we = 1'b1;
                o_wb_sel = 1'b0; // Vector ALU sonucu
            end

            OPC_VLOAD: begin
                o_reg_we = 1'b1;
                o_mem_we = 1'b0;
                o_wb_sel = 1'b1; // LSU'dan
            end

            OPC_VSTORE: begin
                o_reg_we = 1'b0;
                o_mem_we = 1'b1;
            end

            OPC_SADD,
            OPC_SADDI: begin
                o_reg_we = 1'b1;  // Scalar reg file'a yazacağını varsayıyoruz
                o_wb_sel = 1'b0;  // S_ALU yolu (datapath'te o şekilde bağlayacaksın)
            end

            OPC_JUMP: begin
                is_jump   = 1'b1;
                branch_sel = 1'b1; // PC update yolu
            end
        
            OPC_BRANCH: begin
                is_beq    = 1'b1;
                branch_sel = 1'b1;
            end

            default: begin end
        endcase
    end

endmodule




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

    input  logic [$clog2(8)-1:0]        dest_addr_in, // 3-bit gibi düşün

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
                    for (int i = 0; i < LANES; i++) begin
                        if (MASK[i]) Vout[i] = V1[i] + V2[i];
                    end
                end

                OPC_VMUL: begin
                    for (int i = 0; i < LANES; i++) begin
                        if (MASK[i]) Vout[i] = V1[i] * V2[i];
                    end
                end

                OPC_VFMA: begin
                    for (int i = 0; i < LANES; i++) begin
                        if (MASK[i]) Vout[i] = (V1[i] * V2[i]) + Vacc[i];
                    end
                end

                default: begin
                    Vout = '0;
                end
            endcase
        end
    end

endmodule



module S_ALU(
    input  logic       clk, rst_n,
    input  logic [31:0] a, b,
    input  logic [5:0]  op_code,
    output logic [31:0] Sout
);

    always_comb begin
        Sout = '0;

        unique case (op_code)
            6'b00_0000, // SADD
            6'b01_0000: // SADDI
                Sout = a + b;

            default: begin end
        endcase
    end

endmodule



module V_reg_file(
    input  logic              clk,
    input  logic              we,
    input  logic              wb_sel,
    input  logic [7:0]        mask,

    input  logic [7:0][31:0]  result_vec,
    input  logic [7:0][31:0]  load_vec,
    
    input  logic [2:0]        addr_dest,
    input  logic [2:0]        addr_src1,
    input  logic [2:0]        addr_src2,
    
    output logic [7:0][31:0]  val1_o,
    output logic [7:0][31:0]  val2_o,
    output logic [7:0][31:0]  store_vec
);

    logic [7:0][7:0][31:0] M_V_Regs;

    assign val1_o    = M_V_Regs[addr_src1];
    assign val2_o    = M_V_Regs[addr_src2];
    assign store_vec = M_V_Regs[addr_dest];

    always_ff @(posedge clk) begin
        if (we) begin
            for (int i = 0; i < 8; i++) begin
                if (mask[i])
                    M_V_Regs[addr_dest][i] <= wb_sel ? load_vec[i] : result_vec[i];
            end
        end
    end

endmodule


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
        if (mem_we)
            memory[addr[5:0]] <= wdata;
    end

endmodule




module WARP #(
    parameter int NUM_WARPS = 4,
    parameter int LANES     = 8,
    parameter int VREGS     = 8,
    parameter int DATA_W    = 32,
    parameter int PC_W      = 32
) (
    input  logic clk,
    input  logic rst_n,

    // ------------------------------------------------------------
    // WARP selection (from scheduler)
    // ------------------------------------------------------------
    input  logic [$clog2(NUM_WARPS)-1:0] sel_warp_id,
    output logic [PC_W-1:0]              pc_sel,
    output logic [LANES-1:0]             mask_sel,
    output logic [NUM_WARPS-1:0]         can_issue,

    // ------------------------------------------------------------
    // Operand read interface (issue stage)
    // (op_warp_id: hangi warp'ın regleri okunacak)
    // ------------------------------------------------------------
    input  logic [$clog2(NUM_WARPS)-1:0] op_warp_id,
    input  logic [$clog2(VREGS)-1:0]     rs1,
    input  logic [$clog2(VREGS)-1:0]     rs2,
    input  logic [$clog2(VREGS)-1:0]     rd_old,

    output logic [LANES-1:0][DATA_W-1:0] V1_vec,
    output logic [LANES-1:0][DATA_W-1:0] V2_vec,
    output logic [LANES-1:0][DATA_W-1:0] Vrd_old_vec,
    output logic [LANES-1:0][DATA_W-1:0] store_vec,

    // ------------------------------------------------------------
    // Writeback interface (from V_ALU or LSU)
    // ------------------------------------------------------------
    input  logic                          wb_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  wb_warp_id,
    input  logic [$clog2(VREGS)-1:0]      wb_rd,
    input  logic [LANES-1:0]              wb_mask,
    input  logic [LANES-1:0][DATA_W-1:0]  wb_data,

    // ------------------------------------------------------------
    // Issue interface (decoder/dispatch -> scoreboard busy set)
    // ------------------------------------------------------------
    input  logic                          issue_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  issue_warp_id,
    input  logic                          issue_writes_rd,
    input  logic [$clog2(VREGS)-1:0]      issue_rd,

    // Export busy for top-level hazard debug / checks
    output logic [NUM_WARPS-1:0][VREGS-1:0] busy,

    // Selected-warp hazard result
    output logic issue_ok_sel,

    // ------------------------------------------------------------
    // Stall Interface (LSU integration)
    // ------------------------------------------------------------
    input  logic                          stall_set_valid,
    input  logic                          stall_clr_valid,
    input  logic [$clog2(NUM_WARPS)-1:0]  stall_set_warp_id,
    input  logic [$clog2(NUM_WARPS)-1:0]  stall_clr_warp_id,

    // ------------------------------------------------------------
    // PC update interface
    // ------------------------------------------------------------
    input  logic                         pc_update_valid,
    input  logic [$clog2(NUM_WARPS)-1:0] pc_update_warp_id,
    input  logic [PC_W-1:0]              pc_next
);

    // ------------------------------------------------------------
    // WARP STATE
    // ------------------------------------------------------------
    logic [NUM_WARPS-1:0][PC_W-1:0]                         pc_bank;
    logic [NUM_WARPS-1:0][LANES-1:0]                        mask_bank;
    logic [NUM_WARPS-1:0]                                   stalled;
    logic [NUM_WARPS-1:0][VREGS-1:0]                        busy_bank;
    logic [NUM_WARPS-1:0][VREGS-1:0][LANES-1:0][DATA_W-1:0] V;

    // Export busy
    assign busy = busy_bank;

    // ------------------------------------------------------------
    // Selected warp outputs
    // ------------------------------------------------------------
    assign pc_sel   = pc_bank[sel_warp_id];
    assign mask_sel = mask_bank[sel_warp_id];

    // ------------------------------------------------------------
    // Operand reads (combinational)
    // ------------------------------------------------------------
    assign V1_vec      = V[op_warp_id][rs1];
    assign V2_vec      = V[op_warp_id][rs2];
    assign Vrd_old_vec = V[op_warp_id][rd_old];
    assign store_vec   = V[op_warp_id][rs2];

    // ------------------------------------------------------------
    // can_issue: scheduler'a "warp aday mı?" bilgisi
    // (hazard check burada değil)
    // ------------------------------------------------------------
    genvar w;
    generate
        for (w = 0; w < NUM_WARPS; w++) begin : GEN_CAN_ISSUE
            assign can_issue[w] = ~stalled[w];
        end
    endgenerate

    // ------------------------------------------------------------
    // Selected hazard check (tek assign!)
    // Bu sinyal core_top'ta issue_valid'i gate eder:
    //   issue_to_alu = sched_valid & issue_ok_sel
    // ------------------------------------------------------------
    assign issue_ok_sel =
        (~stalled[sel_warp_id]) &&
        (~busy_bank[sel_warp_id][rs1]) &&
        (~busy_bank[sel_warp_id][rs2]) &&
        (~busy_bank[sel_warp_id][rd_old]);

    // ------------------------------------------------------------
    // Sequential state updates
    // ------------------------------------------------------------
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

            // Stall set/clear (clear öncelikli)
            if (stall_set_valid) begin
                stalled[stall_set_warp_id] <= 1'b1;
            end
            if (stall_clr_valid) begin
                stalled[stall_clr_warp_id] <= 1'b0;
            end

            // Issue -> scoreboard busy set
            if (issue_valid && issue_writes_rd) begin
                busy_bank[issue_warp_id][issue_rd] <= 1'b1;
            end

            // Writeback -> reg write + scoreboard clear
            if (wb_valid) begin
                for (li = 0; li < LANES; li++) begin
                    if (wb_mask[li]) begin
                        V[wb_warp_id][wb_rd][li] <= wb_data[li];
                    end
                end
                busy_bank[wb_warp_id][wb_rd] <= 1'b0;
            end
        end
    end

endmodule



module Scheduler #(
    parameter int NUM_WARPS = 4
)(
    input  logic clk,
    input  logic rst_n,

    input  logic [NUM_WARPS-1:0]         can_issue,

    output logic [$clog2(NUM_WARPS)-1:0] sel_warp_id,
    output logic                         sel_valid
);
    localparam int WARP_ID_W = $clog2(NUM_WARPS);

    logic [WARP_ID_W-1:0] last_grant;
    logic [WARP_ID_W-1:0] pick_id;
    logic                pick_valid;

    always_comb begin
        pick_id    = last_grant;
        pick_valid = 1'b0;

        for (int k = 1; k <= NUM_WARPS; k++) begin
            int idx = (last_grant + k) % NUM_WARPS;
            if (!pick_valid && can_issue[idx]) begin
                pick_valid = 1'b1;
                pick_id    = idx[WARP_ID_W-1:0];
            end
        end

        sel_valid   = pick_valid;
        sel_warp_id = pick_id;
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            last_grant <= '0;
        end else if (pick_valid) begin
            last_grant <= pick_id;
        end
    end
endmodule



module PC_control_unit #(
    parameter int PC_W   = 32,
    parameter int LANES  = 8,
    parameter int DATA_W = 32
)(
    input  logic                     do_issue,

    // branch / jump select
    input  logic                     is_jump,
    input  logic                     is_beq,

    // warp context
    input  logic [PC_W-1:0]          pc_cur,
    input  logic [PC_W-1:0]          branch_offset,

    // vector operands + mask (ID aşamasından)
    input  logic [LANES-1:0]         mask,
    input  logic [LANES-1:0][DATA_W-1:0] V1_vec,
    input  logic [LANES-1:0][DATA_W-1:0] V2_vec,

    // debug/telemetry (sen istedin: ALL ve scalar ayrı ayrı)
    output logic                     beq_taken_all,
    output logic                     beq_taken_scalar,

    output logic                     pc_update_valid,
    output logic [PC_W-1:0]          pc_next
);

    logic taken;

    always_comb begin
        // defaults
        pc_update_valid  = 1'b0;
        pc_next          = pc_cur;

        beq_taken_all    = 1'b1;   // AND reduce identity
        beq_taken_scalar = 1'b0;

        // scalar compare: lane0
        beq_taken_scalar = (V1_vec[0] == V2_vec[0]);

        // ALL reduce compare (mask'li)
        // taken_all = AND over lanes: (!mask[i]) OR (V1[i]==V2[i])
        for (int i = 0; i < LANES; i++) begin
            beq_taken_all &= ( (!mask[i]) || (V1_vec[i] == V2_vec[i]) );
        end

        taken = 1'b0;
        if (is_jump) begin
            taken = 1'b1;
        end else if (is_beq) begin
            taken = beq_taken_all;
        end

        // PC update only if we actually issue
        if (do_issue) begin
            pc_update_valid = 1'b1;
            if (taken) begin
                pc_next = pc_cur + branch_offset;
            end else begin
                pc_next = pc_cur + 32'd4;
            end
        end
    end

endmodule




module Imm_Unit (
    input  logic [7:0]  imm_field,      // instr[15:8]

    // ALU tarafı için immediate (sign-extended)
    output logic [31:0] imm_value,

    // Branch / PC tarafı için offset (sign-extended, word -> byte)
    output logic [31:0] branch_offset
);

    // Internal sign-extended immediate
    logic [31:0] imm_sext;

    always_comb begin
        // 8-bit signed immediate -> 32-bit
        imm_sext = {{24{imm_field[7]}}, imm_field};

        // ALU immediates (S_ALU / V_ALU broadcast)
        imm_value = imm_sext;

        // Branch target offset: instruction count -> byte address
        // PC + (imm << 2)
        branch_offset = imm_sext <<< 2;
    end

endmodule



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
            end
            else if (load) begin
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
            end
            else if (bubble) begin
                ex_valid <= 1'b0;
            end
        end
    end

endmodule






