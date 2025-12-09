`timescale 1ns/1ps

module tb_GPU_Core;

    // DUT girişleri
    logic clk;
    logic rst_n;

    // DUT
    GPU_Core dut (
        .clk   (clk),
        .rst_n (rst_n)
    );

    // Basit saat üretimi: 100 MHz -> 10 ns period
    initial clk = 0;
    always #5 clk = ~clk;

    // Küçük yardımcı: instruction oluşturma fonksiyonu
    function automatic [31:0] make_instr (
        input logic [5:0] opcode,
        input logic [2:0] addr_dest,
        input logic [2:0] addr_src1,
        input logic [2:0] addr_src2,
        input logic [15:0] mask
    );
        make_instr = '0;
        make_instr[31:26] = opcode;
        make_instr[25:23] = addr_dest;
        make_instr[22:20] = addr_src1;
        make_instr[19:17] = addr_src2;
        make_instr[15:0]  = mask;
        // instr[16] kullanılmıyor, 0 bırakıyoruz
    endfunction


    initial begin : tb_main
    
    
        int r, lane;

        // Başlangıç
        rst_n = 0;
        
        
         #1;
         dut.imem_inst.memory[0] = 32'h12345678;

        // Önce register dosyası ve DMEM'i temizleyelim
        // (Hiyerarşik erişim, sadece TB tarafında)
        for (r = 0; r < 8; r++) begin
            for (lane = 0; lane < 16; lane++) begin
                dut.vrf_inst.M_V_Regs[r][lane] = '0;
            end
        end

        for (r = 0; r < 64; r++) begin
            for (lane = 0; lane < 16; lane++) begin
                dut.dmem_inst.memory[r][lane] = '0;
            end
        end

        // Test için:
        // v1: 1,2,3,...,16
        // v2: 100,101,102,...,115
        // v3: pointer register (tüm lane'leri 5, ama sadece lane0 adres olarak kullanılacak)
        for (lane = 0; lane < 16; lane++) begin
            dut.vrf_inst.M_V_Regs[3'd1][lane] = lane + 1;         // v1
            dut.vrf_inst.M_V_Regs[3'd2][lane] = 32'd100 + lane;   // v2
            dut.vrf_inst.M_V_Regs[3'd3][lane] = 32'd5;            // v3 -> addr = 5
        end

        // DMEM[5] başlangıçta 0, VSTORE ile buraya yazacağız,
        // sonra VLOAD ile v4'e çekeceğiz.

        // Programı Instruction Memory'e yükleme
        // 0: VADD  v0, v1, v2, mask=FFFF
        dut.imem_inst.memory[0] = make_instr(
            6'b000001,   // VADD
            3'd0,        // dest = v0
            3'd1,        // src1 = v1
            3'd2,        // src2 = v2
            16'hFFFF     // tüm lane'ler aktif
        );

        // 1: VSTORE v0, v3, -, mask=FFFF
        //     addr_src1 = v3 pointer (reg_out1_wire)
        //     addr_dest = v0 store edilecek veri (store_vec)
        dut.imem_inst.memory[1] = make_instr(
            6'b100001,   // VSTORE
            3'd0,        // dest = v0 (store_vec kaynağı)
            3'd3,        // src1 = v3 (addr pointer)
            3'd0,        // src2 kullanılmıyor
            16'hFFFF
        );

        // 2: VLOAD v4, v3, -, mask=FFFF
        //     addr_src1 = v3 pointer
        //     addr_dest = v4 hedef
        dut.imem_inst.memory[2] = make_instr(
            6'b100000,   // VLOAD
            3'd4,        // dest = v4
            3'd3,        // src1 = v3
            3'd0,        // src2 kullanılmıyor
            16'hFFFF
        );

        // Diğer adreslere NOP (00'lı opcode) yazalım, çöp işlem olmasın
        for (r = 3; r < 64; r++) begin
            dut.imem_inst.memory[r] = 32'b0;
        end

        // Reset'i bir süre tut
        #20;
        rst_n = 1;

        // Programı çalıştırmak için birkaç clock bekleyelim
        // PC: 0 -> 4 -> 8 ... 3 komut için 3-4 clock yeterli
        repeat (10) @(posedge clk);

        // --- Kontrol 1: VADD sonucu v0 = v1 + v2 olmalı ---
        $display("Kontrol 1: VADD sonucu (v0 = v1 + v2) kontrol ediliyor...");
        for (lane = 0; lane < 16; lane++) begin
            logic [31:0] exp;
            exp = dut.vrf_inst.M_V_Regs[3'd1][lane] + dut.vrf_inst.M_V_Regs[3'd2][lane];
            if (dut.vrf_inst.M_V_Regs[3'd0][lane] !== exp) begin
                $error("VADD HATA: lane %0d: v0=%0d, beklenen=%0d",
                       lane,
                       dut.vrf_inst.M_V_Regs[3'd0][lane],
                       exp);
            end
        end

        // --- Kontrol 2: VSTORE sonrası DMEM[5] == v0 olmalı ---
        $display("Kontrol 2: VSTORE sonucu (DMEM[5] == v0) kontrol ediliyor...");
        for (lane = 0; lane < 16; lane++) begin
            if (dut.dmem_inst.memory[5][lane] !== dut.vrf_inst.M_V_Regs[3'd0][lane]) begin
                $error("VSTORE HATA: lane %0d: DMEM[5]=%0d, v0=%0d",
                       lane,
                       dut.dmem_inst.memory[5][lane],
                       dut.vrf_inst.M_V_Regs[3'd0][lane]);
            end
        end

        // --- Kontrol 3: VLOAD sonrası v4 == DMEM[5] olmalı ---
        $display("Kontrol 3: VLOAD sonucu (v4 == DMEM[5]) kontrol ediliyor...");
        for (lane = 0; lane < 16; lane++) begin
            if (dut.vrf_inst.M_V_Regs[3'd4][lane] !== dut.dmem_inst.memory[5][lane]) begin
                $error("VLOAD HATA: lane %0d: v4=%0d, DMEM[5]=%0d",
                       lane,
                       dut.vrf_inst.M_V_Regs[3'd4][lane],
                       dut.dmem_inst.memory[5][lane]);
            end
        end

        $display("Test tamamlandi.");
        $finish;
    end

endmodule
