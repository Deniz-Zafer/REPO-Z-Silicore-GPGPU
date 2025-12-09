`timescale 1ns / 1ps

module GPU_Core (
    input logic clk,
    input logic rst_n
);

    // --- KABLOLAR (Interconnects) ---
    logic [31:0]       pc_wire;
    logic [31:0]       instruction_wire;
    
    // Decoder Çıkışları
    logic [5:0]        opcode_wire;
    logic [2:0]        dest_addr, src1_addr, src2_addr;
    logic [7:0]        mask_wire;
    logic              reg_we_wire, mem_we_wire, wb_sel_wire;

    // Branch / Imm
    logic              SV_sel_wire;
    logic              Imm_sel_wire;
    logic              branch_sel_wire;
    logic [7:0]        imm_field_wire;
    logic [31:0]       imm_value_wire;      // S_ALU vs. için
    logic [31:0]       branch_offset_wire;  // PC için

    // Veri Yolları (8 Lane)
    logic [7:0][31:0]  alu_result_wire;
    logic [7:0][31:0]  mem_rdata_wire;
    logic [7:0][31:0]  mem_wdata_wire;
    logic [7:0][31:0]  reg_out1_wire;
    logic [7:0][31:0]  reg_out2_wire;

    // --- 0. Imm Unit ---
    Imm_Unit imm_inst (
        .imm_field    (imm_field_wire),
        .imm_value    (imm_value_wire),
        .branch_offset(branch_offset_wire)
    );

    // --- 1. Program Counter ---
    PC pc_inst (
        .clk        (clk), 
        .rst_n      (rst_n),
        .branch_sel (branch_sel_wire),
        .offset     (branch_offset_wire),   // <<2 işi Imm_Unit içinde
        .pc_addr    (pc_wire)
    );

    // --- 2. Instruction Memory ---
    IMEM imem_inst (
        .clk(clk), 
        .pc_addr(pc_wire), 
        .instruction(instruction_wire)
    );

    // --- 3. Decoder (Control Unit) ---
    decoder dec_inst (
        .instr     (instruction_wire),
        .opcode    (opcode_wire),
        .addr_dest (dest_addr),
        .addr_src1 (src1_addr),
        .addr_src2 (src2_addr),

        .SV_sel    (SV_sel_wire),
        .Imm_sel   (Imm_sel_wire),
        .branch_sel(branch_sel_wire),

        .write_mask(mask_wire),
        .imm_field (imm_field_wire),

        .o_reg_we  (reg_we_wire),
        .o_mem_we  (mem_we_wire),
        .o_wb_sel  (wb_sel_wire)
    );

    // --- 4. Vector Register File ---
    V_reg_file vrf_inst (
        .clk       (clk),
        .we        (reg_we_wire),
        .wb_sel    (wb_sel_wire),
        .mask      (mask_wire),
        .result_vec(alu_result_wire),
        .load_vec  (mem_rdata_wire),
        .addr_dest (dest_addr),
        .addr_src1 (src1_addr),
        .addr_src2 (src2_addr),
        .val1_o    (reg_out1_wire),
        .val2_o    (reg_out2_wire),
        .store_vec (mem_wdata_wire)
    );

    // --- 5. Vector ALU ---
    ALU alu_inst (
        .clk    (clk), 
        .rst_n  (rst_n),
        .V1     (reg_out1_wire),
        .V2     (reg_out2_wire),
        .op_code(opcode_wire),
        .MASK   (mask_wire),
        .Vout   (alu_result_wire)
    );

    // (İleride isteğe göre: S_ALU burada bağlanır, Imm_sel=1 ise b tarafına imm_value_wire verilir vs.)

    // --- 6. Data Memory (LSU Birimi) ---
    DMEM dmem_inst (
        .clk  (clk),
        .mem_we(mem_we_wire),
        .addr (reg_out1_wire[0]),
        .wdata(mem_wdata_wire),
        .rdata(mem_rdata_wire)
    );

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



// Program counter
module PC(
    input  logic        clk, 
    input  logic        rst_n,
    input  logic        branch_sel,      // 1 ise zıplama
    input  logic [31:0] offset,          // Imm_Unit'ten gelen (sign-extend + <<2)

    output logic [31:0] pc_addr 
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pc_addr <= 32'b0;
        end else begin
            if (branch_sel) begin
                pc_addr <= pc_addr + offset; // Zıplama burada
            end else begin
                pc_addr <= pc_addr + 4;      // Normal PC+4
            end
        end
    end

endmodule



module Imm_Unit (
    input  logic [7:0]  imm_field,      // instr[15:8]

    output logic [31:0] imm_value,      // S/V ALU için normal imm (sign-extended)
    output logic [31:0] branch_offset   // PC için offset (sign-extended + <<2)
);
    logic [31:0] imm_ext;

    always_comb begin
        // 8 bit signed immed'i 32 bite genişlet
        imm_ext   = {{24{imm_field[7]}}, imm_field};  // sign extend
        imm_value = imm_ext;          // ALU için direkt bu kullanılabilir
        branch_offset = imm_ext << 2; // instr sayısı ⇒ byte adresi ( *4 )
    end

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

            OPC_BRANCH: begin
                branch_sel = 1'b1; // PC bu cycle'da offset ile zıplayacak
            end

            default: begin end
        endcase
    end

endmodule




module ALU (
    input  logic               clk, rst_n,
    input  logic [7:0][31:0]   V1, V2,
    input  logic [5:0]         op_code,
    input  logic [7:0]         MASK,
    output logic [7:0][31:0]   Vout
);

    always_comb begin
        Vout = '0;

        // Sadece alt 4 bit'e göre VADD yapalım (op_field = 0000)
        unique case (op_code[3:0])
            4'b0000: begin
                for (int i = 0; i < 8; i++) begin
                    if (MASK[i])
                        Vout[i] = V1[i] + V2[i];
                end
            end

            default: begin end
        endcase
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
