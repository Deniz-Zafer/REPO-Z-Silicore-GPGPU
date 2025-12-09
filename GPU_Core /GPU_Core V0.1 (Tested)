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
    logic [15:0]       mask_wire;
    logic              reg_we_wire, mem_we_wire, wb_sel_wire;

    // Veri Yolları
    logic [15:0][31:0] alu_result_wire;
    logic [15:0][31:0] mem_rdata_wire;  // RAM -> Reg
    logic [15:0][31:0] mem_wdata_wire;  // Reg -> RAM
    logic [15:0][31:0] reg_out1_wire;   // Reg -> ALU A
    logic [15:0][31:0] reg_out2_wire;   // Reg -> ALU B

    // --- 1. Program Counter ---
    PC pc_inst (
        .clk(clk), 
        .rst_n(rst_n), 
        .pc_addr(pc_wire)
    );

    // --- 2. Instruction Memory ---
    IMEM imem_inst (
        .clk(clk), 
        .pc_addr(pc_wire), 
        .instruction(instruction_wire)
    );

    // --- 3. Decoder (Control Unit) ---
    decoder dec_inst (
        .instr(instruction_wire),
        .opcode(opcode_wire),
        .addr_dest(dest_addr),
        .addr_src1(src1_addr),
        .addr_src2(src2_addr),
        .write_mask(mask_wire),
        .o_reg_we(reg_we_wire),
        .o_mem_we(mem_we_wire),
        .o_wb_sel(wb_sel_wire)
    );

    // --- 4. Vector Register File ---
    V_reg_file vrf_inst (
        .clk(clk),
        .we(reg_we_wire),
        .wb_sel(wb_sel_wire),
        .mask(mask_wire),
        .result_vec(alu_result_wire), // ALU sonucu girişi
        .load_vec(mem_rdata_wire),    // Memory okuma girişi
        .addr_dest(dest_addr),
        .addr_src1(src1_addr),
        .addr_src2(src2_addr),
        .val1_o(reg_out1_wire),       // ALU'ya giden A
        .val2_o(reg_out2_wire),       // ALU'ya giden B
        .store_vec(mem_wdata_wire)    // Memory'ye giden veri
    );

    // --- 5. ALU ---
    ALU alu_inst (
        .clk(clk), 
        .rst_n(rst_n),
        .V1(reg_out1_wire),
        .V2(reg_out2_wire),
        .op_code(opcode_wire), // Decoder'dan gelen Opcode
        .MASK(mask_wire),
        .Vout(alu_result_wire)
    );

    // --- 6. Data Memory (LSU Birimi) ---
    // Not: Adresleme için basitlik adına Kaynak1'in ilk elemanını pointer yaptık
    // Gerçekte daha kompleks bir adresleme mantığı (Base + Offset) kurulabilir.
    DMEM dmem_inst (
        .clk(clk),
        .mem_we(mem_we_wire),
        .addr(reg_out1_wire[0]), // Vektörün 0. şeridini adres olarak kullandık!
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



//Program counter
module PC(
    input  logic        clk, 
    input  logic        rst_n,     
    output logic [31:0] pc_addr 
);

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin           
            pc_addr <= 32'b0;       
        end else begin
            pc_addr <= pc_addr + 4; 
        end
    end

endmodule



//Control Unit
module decoder (
    input  logic [31:0] instr,
    
    // Eski Sinyaller
    output logic [5:0]  opcode,
    output logic [2:0]  addr_dest,
    output logic [2:0]  addr_src1,   // VLOAD/VSTORE için ADRESİ tutar
    output logic [2:0]  addr_src2,
    output logic [15:0] write_mask,  // 16-Lane Maskesi
    
    // --- YENİ EKLENEN LSU SİNYALLERİ ---
    output logic        o_reg_we,    // Register'a yazma izni (Load=1, Store=0)
    output logic        o_mem_we,    // RAM'e yazma izni (Store=1, Load=0)
    output logic        o_wb_sel     // Geri Dönüş Seçici (0=ALU, 1=LSU)
);

    // Wire atamaları (Aynı kalıyor)
    assign opcode     = instr[31:26];
    assign addr_dest  = instr[25:23];
    assign addr_src1  = instr[22:20]; // Adres Pointer (örn: v2)
    assign addr_src2  = instr[19:17];
    assign write_mask = instr[15:0];

    // --- LOGIC KISMI ---
    always_comb begin
        // Varsayılan değerler (Kaza olmasın diye)
        o_reg_we = 0;
        o_mem_we = 0;
        o_wb_sel = 0; // Default ALU

        case (opcode)
            // --- ALU İŞLEMLERİ ---
            6'b000001: begin // VADD
                o_reg_we = 1; // Sonucu kaydet
                o_wb_sel = 0; // Veri ALU'dan geliyor
            end
            
            // --- LSU İŞLEMLERİ ---
            6'b100000: begin // VLOAD (RAM -> Reg)
                o_reg_we = 1; // Register'a yazacağız
                o_mem_we = 0; // RAM'e yazmıyoruz, okuyoruz
                o_wb_sel = 1; // DİKKAT: Veri LSU'dan geliyor!
            end

            6'b100001: begin // VSTORE (Reg -> RAM)
                o_reg_we = 0; // Register'a bir şey yazma
                o_mem_we = 1; // RAM'e yaz
            end
            
            default: ; // NOP
        endcase
    end
    
endmodule



//ALU_Array
module ALU (
    input  logic               clk, rst_n,

    // Düzeltme 1: [15:0] = 16 Lane (Şerit), [31:0] = 32 Bit Veri
    input  logic [15:0][31:0]  V1, V2, 
    input  logic [5:0]         op_code,
    input  logic [15:0]        MASK,

    output logic [15:0][31:0]  Vout
);

    always_comb begin
        Vout = '0; 

        case (op_code)
            // VADD (Toplama)
            6'b000001: begin 
                for (int i = 0; i < 16; i++) begin
                    if (MASK[i]) begin
                        Vout[i] = V1[i] + V2[i]; 
                    end else begin
                        Vout[i] = 32'b0; // Maske kapalıysa 0 ver (veya eski değeri koru)
                    end
                end
            end
            //diğer operatörler
            
            default: Vout = '0;
        endcase
    end

endmodule


module V_reg_file(
    input  logic              clk,          // Yazma için saat sinyali şart
    input  logic              we,           // Write Enable (Yazma izni)
    input  logic              wb_sel,       // 0: ALU sonucu, 1: Load (RAM) verisi
    input  logic [15:0]       mask,         // Hangi şeritlere (Lane) yazılacak?

    input  logic [15:0][31:0] result_vec,   // ALU'dan gelen veri
    input  logic [15:0][31:0] load_vec,     // RAM'den gelen veri (LSU)
    
    input  logic [2:0]        addr_dest,    // Hedef adres (Nereye yazılacak)
    input  logic [2:0]        addr_src1,    // Kaynak 1 adresi
    input  logic [2:0]        addr_src2,    // Kaynak 2 adresi
    
    output logic [15:0][31:0] val1_o,       // ALU'ya giden Vektör 1 (Eksikti)
    output logic [15:0][31:0] val2_o,       // ALU'ya giden Vektör 2 (Eksikti)
    output logic [15:0][31:0] store_vec     // RAM'e gönderilecek veri
);

    // 8 Adet, 16 Şeritli, 32-bitlik Register Bloğu
    // [7:0] = Register Sayısı (3 bit adres 0-7 arası erişir)
    logic [7:0][15:0][31:0] M_V_Regs;

    // --- OKUMA İŞLEMİ (Asenkron) ---
    // Adres geldiği an veriyi ALU'ya ve LSU'ya gönder
    assign val1_o    = M_V_Regs[addr_src1];
    assign val2_o    = M_V_Regs[addr_src2];
    assign store_vec = M_V_Regs[addr_dest]; // Store komutu için veri kaynağı

    // --- YAZMA İŞLEMİ (Senkron) ---
    always_ff @(posedge clk) begin
        if (we) begin
            // 16 Lane'i tek tek kontrol et
            for (int i = 0; i < 16; i++) begin
                if (mask[i]) begin
                    // Mux Mantığı: wb_sel 1 ise Load, 0 ise Result verisi
                    M_V_Regs[addr_dest][i] <= wb_sel ? load_vec[i] : result_vec[i];
                end
            end
        end
    end

endmodule



// Data Memory (Veri Belleği)
module DMEM (
    input  logic              clk,
    input  logic              mem_we,    // Decoder'dan gelen o_mem_we
    input  logic [31:0]       addr,      // Pointer (Registers'dan gelir)
    input  logic [15:0][31:0] wdata,     // Store edilecek vektör
    output logic [15:0][31:0] rdata      // Load edilecek vektör
);

    // 64 satırlık, her satırı 512-bit (16x32) olan dev bellek
    logic [15:0][31:0] memory [0:63];
    
    assign rdata = memory[addr[5:0]]; // Sadece alt 6 bit

    // Yazma (Senkron)
    always_ff @(posedge clk) begin
        if (mem_we) begin
            memory[addr[5:0]] <= wdata;
        end
    end

endmodule

