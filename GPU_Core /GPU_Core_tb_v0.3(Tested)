`timescale 1ns/1ps

module tb_GPU_Core;

  localparam int NUM_WARPS = 4;
  localparam int LANES     = 8;
  localparam int VREGS     = 8;
  localparam int DATA_W    = 32;
  localparam int PC_W      = 32;

  logic clk;
  logic rst_n;

  GPU_Core #(
    .NUM_WARPS(NUM_WARPS),
    .LANES    (LANES),
    .VREGS    (VREGS),
    .DATA_W   (DATA_W),
    .PC_W     (PC_W)
  ) dut (
    .clk  (clk),
    .rst_n(rst_n)
  );

  // -------------------------
  // Clock / Reset
  // -------------------------
  int unsigned cycle;

  initial begin
    clk = 0;
    forever #5 clk = ~clk; // 100MHz
  end

  initial begin
    rst_n = 0;
    cycle = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
  end

  always @(posedge clk) begin
    if (!rst_n) cycle <= 0;
    else        cycle <= cycle + 1;
  end

  // -------------------------
  // Helpers: vector print
  // -------------------------
  task automatic print_vec3(
    input string tag,
    input logic [LANES-1:0][DATA_W-1:0] a,
    input logic [LANES-1:0][DATA_W-1:0] b,
    input logic [LANES-1:0][DATA_W-1:0] c
  );
    int i;
    $display("[%0d] %s", cycle, tag);
    for (i = 0; i < LANES; i++) begin
      $display("    lane%0d  A=%08x  B=%08x  C=%08x", i, a[i], b[i], c[i]);
    end
  endtask

  // -------------------------
  // ISA opcodes (decoder ile uyumlu)
  // -------------------------
  localparam [5:0] OPC_VADD   = 6'b10_0000;
  localparam [5:0] OPC_VADDI  = 6'b11_0000;
  localparam [5:0] OPC_VLOAD  = 6'b10_1000;
  localparam [5:0] OPC_VSTORE = 6'b10_1001;
  localparam [5:0] OPC_VFMA   = 6'b10_0011;
  localparam [5:0] OPC_JUMP   = 6'b01_1110;
  localparam [5:0] OPC_BEQ    = 6'b01_1111;

  function automatic [31:0] enc_instr(
    input bit sv,
    input bit imm,
    input logic [3:0] op_field,
    input logic [2:0] rd,
    input logic [2:0] rs1,
    input logic [2:0] rs2,
    input logic [7:0] imm_field,
    input logic [7:0] mask
  );
    enc_instr = '0;
    enc_instr[31]    = sv;
    enc_instr[30]    = imm;
    enc_instr[29:26] = op_field;
    enc_instr[25:23] = rd;
    enc_instr[22:20] = rs1;
    enc_instr[19:17] = rs2;
    enc_instr[15:8]  = imm_field;
    enc_instr[7:0]   = mask;
  endfunction

  function automatic [3:0] opfield_from_opcode(input logic [5:0] opc);
    opfield_from_opcode = opc[3:0];
  endfunction

  // -------------------------
  // Program & Memory Init
  // -------------------------
  localparam int PROG_STRIDE_BYTES = 16; // 4 instr * 4 bytes
  localparam int PROG0_PC = 32'd0;
  localparam int PROG1_PC = 32'd16;
  localparam int PROG2_PC = 32'd32;
  localparam int PROG3_PC = 32'd48;

  // Register mapping for TB program:
  // v1 = base address pointer (lane0 used)
  // v0 = load dst0
  // v2 = load dst1
  // v3 = v0 + v2

  task automatic init_imem_program();
    // program: LOAD v0,[v1] ; LOAD v2,[v1] (same base, just demo) ; VADD v3=v0+v2 ; JUMP 0 (loop)
    // mask = 0xFF (8 lane aktif)
    logic [31:0] i0, i1, i2, i3;

    i0 = enc_instr(1'b1, 1'b0, opfield_from_opcode(OPC_VLOAD), 3'd0, 3'd1, 3'd0, 8'd0, 8'hFF);
    i1 = enc_instr(1'b1, 1'b0, opfield_from_opcode(OPC_VLOAD), 3'd2, 3'd1, 3'd0, 8'd0, 8'hFF);
    i2 = enc_instr(1'b1, 1'b0, opfield_from_opcode(OPC_VADD),  3'd3, 3'd0, 3'd2, 8'd0, 8'hFF);
    i3 = enc_instr(1'b0, 1'b1, opfield_from_opcode(OPC_JUMP),  3'd0, 3'd0, 3'd0,8'hFD,8'hFF);

    // load same program into 4 regions
    // IMEM word index = pc[7:2]
    dut.imem_inst.memory[(PROG0_PC+0)  >> 2] = i0;
    dut.imem_inst.memory[(PROG0_PC+4)  >> 2] = i1;
    dut.imem_inst.memory[(PROG0_PC+8)  >> 2] = i2;
    dut.imem_inst.memory[(PROG0_PC+12) >> 2] = i3;

    dut.imem_inst.memory[(PROG1_PC+0)  >> 2] = i0;
    dut.imem_inst.memory[(PROG1_PC+4)  >> 2] = i1;
    dut.imem_inst.memory[(PROG1_PC+8)  >> 2] = i2;
    dut.imem_inst.memory[(PROG1_PC+12) >> 2] = i3;

    dut.imem_inst.memory[(PROG2_PC+0)  >> 2] = i0;
    dut.imem_inst.memory[(PROG2_PC+4)  >> 2] = i1;
    dut.imem_inst.memory[(PROG2_PC+8)  >> 2] = i2;
    dut.imem_inst.memory[(PROG2_PC+12) >> 2] = i3;

    dut.imem_inst.memory[(PROG3_PC+0)  >> 2] = i0;
    dut.imem_inst.memory[(PROG3_PC+4)  >> 2] = i1;
    dut.imem_inst.memory[(PROG3_PC+8)  >> 2] = i2;
    dut.imem_inst.memory[(PROG3_PC+12) >> 2] = i3;
  endtask

  task automatic init_dmem();
    // DMEM: memory[addr[5:0]] holds 8x32 vector
    // Her warp farklı base addr kullanacak => farklı data görecek.
    int a;
    for (a = 0; a < 64; a++) begin
      dut.dmem_inst.memory[a] = '0;
    end

    // base 0x00 => idx 0
    dut.dmem_inst.memory[0] = '{
      32'h0000_0001, 32'h0000_0002, 32'h0000_0003, 32'h0000_0004,
      32'h0000_0005, 32'h0000_0006, 32'h0000_0007, 32'h0000_0008
    };
    // base 0x04 => idx 4
    dut.dmem_inst.memory[4] = '{
      32'h0000_0010, 32'h0000_0020, 32'h0000_0030, 32'h0000_0040,
      32'h0000_0050, 32'h0000_0060, 32'h0000_0070, 32'h0000_0080
    };
    // base 0x08 => idx 8
    dut.dmem_inst.memory[8] = '{
      32'h0000_0100, 32'h0000_0200, 32'h0000_0300, 32'h0000_0400,
      32'h0000_0500, 32'h0000_0600, 32'h0000_0700, 32'h0000_0800
    };
    // base 0x0C => idx 12
    dut.dmem_inst.memory[12] = '{
      32'h0000_1000, 32'h0000_2000, 32'h0000_3000, 32'h0000_4000,
      32'h0000_5000, 32'h0000_6000, 32'h0000_7000, 32'h0000_8000
    };
  endtask

    task init_warps_state;
      integer w;
      integer l;
      begin
        dut.warp_inst.pc_bank[0] = PROG0_PC;
        dut.warp_inst.pc_bank[1] = PROG1_PC;
        dut.warp_inst.pc_bank[2] = PROG2_PC;
        dut.warp_inst.pc_bank[3] = PROG3_PC;
    
        for (w = 0; w < NUM_WARPS; w = w + 1) begin
          for (l = 0; l < LANES; l = l + 1) begin
            case (w)
              0: dut.warp_inst.V[w][1][l] = 32'h0000_0000;
              1: dut.warp_inst.V[w][1][l] = 32'h0000_0004;
              2: dut.warp_inst.V[w][1][l] = 32'h0000_0008;
              3: dut.warp_inst.V[w][1][l] = 32'h0000_000C;
            endcase
          end
        end
      end
    endtask

  // Reset sonrası init
  initial begin
    // IMEM/DMEM init için resetten önce de yazılabilir
    init_imem_program();
    init_dmem();

    @(posedge rst_n);
    // reset kalkınca 1 cycle sonra internal init'ler otursun
    @(posedge clk);
    init_warps_state();

    $display("=== TB init done @cycle %0d ===", cycle);
  end

  // -------------------------
  // LOAD latency log simulation (TB-only)
  // -------------------------
  typedef struct {
    bit     pend;
    int     start_cycle;
    int     done_cycle;
    logic [31:0] addr;
  } load_track_t;

  load_track_t lt[NUM_WARPS];

  function automatic int rand_lat();
    // 2..8 cycle arası "LSU" latency (istersen değiştir)
    rand_lat = 2 + ($urandom_range(0,6));
  endfunction

  // -------------------------
  // Main monitors
  // -------------------------
  // Hier taps:
  // dut.do_issue_id
  // dut.sel_warp_id
  // dut.opcode_wire
  // dut.ex_valid, dut.ex_is_load, dut.ex_warp_id
  // dut.ex_V1_vec, dut.ex_V2_eff
  // dut.mem_rdata_wire, dut.alu_vout

  function automatic string opc_name(input logic [5:0] opc);
    case (opc)
      OPC_VADD:   opc_name = "VADD";
      OPC_VADDI:  opc_name = "VADDI";
      OPC_VLOAD:  opc_name = "VLOAD";
      OPC_VSTORE: opc_name = "VSTORE";
      OPC_VFMA:   opc_name = "VFMA";
      OPC_JUMP:   opc_name = "JUMP";
      OPC_BEQ:    opc_name = "BEQ";
      default:    opc_name = $sformatf("OPC_%0h", opc);
    endcase
  endfunction

  // Issue log (ID stage)
  always @(posedge clk) begin
    if (rst_n) begin
      if (dut.do_issue_id) begin
        $display("[%0d] WARP-%0d ISSUE %s  PC=%08x  mask=%02x",
                 cycle, dut.sel_warp_id, opc_name(dut.opcode_wire),
                 dut.pc_sel, dut.mask_wire);

        // 8-lane x2 input + 1 output (o anki op'a göre)
        if (dut.opcode_wire == OPC_VLOAD) begin
          // LOAD: out olarak mem_rdata_wire gösteriyoruz (core comb read)
          print_vec3(
            $sformatf("WARP-%0d VEC (LOAD) V1(addr) / V2(dummy) / MEM_OUT",
                      dut.sel_warp_id),
            dut.V1_vec, dut.V2_vec, dut.mem_rdata_wire
          );
        end else begin
          // ALU ops: out alu_vout (valid_out aynı cycle'da)
          print_vec3(
            $sformatf("WARP-%0d VEC (ALU) V1 / V2eff / ALU_OUT",
                      dut.sel_warp_id),
            dut.V1_vec, dut.V2_eff, dut.alu_vout
          );
        end
      end
    end
  end

  // "LSU handshake" (TB-only) log
  always @(posedge clk) begin
    if (!rst_n) begin
      for (int w=0; w<NUM_WARPS; w++) begin
        lt[w].pend        <= 0;
        lt[w].start_cycle <= 0;
        lt[w].done_cycle  <= 0;
        lt[w].addr        <= '0;
      end
    end else begin
      // LOAD request detect: issue edilen opcode VLOAD ise
      if (dut.do_issue_id && dut.opcode_wire == OPC_VLOAD) begin
        int w = dut.sel_warp_id;
        if (!lt[w].pend) begin
          lt[w].pend        <= 1;
          lt[w].start_cycle <= cycle;
          lt[w].addr        <= dut.V1_vec[0];
          lt[w].done_cycle  <= cycle + rand_lat();

          $display("[%0d] WARP-%0d Load %08x'dan veri cekmek icin REQ gonderdi (planlanan sure: %0d cycle)",
                   cycle, w, dut.V1_vec[0], (lt[w].done_cycle - cycle));
          $display("[%0d] Load el sikismasi gerceklesti -> LSU gorevde (WARP-%0d)",
                   cycle, w);
        end
      end

      // complete prints
      for (int w=0; w<NUM_WARPS; w++) begin
        if (lt[w].pend && (cycle == lt[w].done_cycle)) begin
          int dt = lt[w].done_cycle - lt[w].start_cycle;
          $display("[%0d] WARP-%0d Load TAMAMLANDI addr=%08x (gecen sure: %0d cycle, start=%0d)",
                   cycle, w, lt[w].addr, dt, lt[w].start_cycle);
          lt[w].pend <= 0;
        end
      end
    end
  end

  // Stop
  initial begin
    // 200 cycle koş
    wait(rst_n);
    repeat (200) @(posedge clk);
    $display("=== TB finished @cycle %0d ===", cycle);
    $finish;
  end

endmodule
