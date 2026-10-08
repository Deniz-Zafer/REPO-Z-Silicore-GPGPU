# REPO-Z-Silicore-GPGPU
General purpose / AI Accelerator GPU Design

A SIMT-style GPGPU core written in SystemVerilog: 4 warps × 8 lanes, 32-bit data path, vector register file with masked execution.

## Versions (`GPU_Core/`)

| Version | Highlights |
|---|---|
| V0.1 (tested) | Basic GPU core: IMEM, decoder, warp scheduler, vector ALU, DMEM |
| V0.2 (untested) | Immediates, branching, scalar unit |
| V0.3 (tested) | Masked all-reduce `BEQ`, unconditional `JUMP`, PC control unit, warp hazard optimization |
| **V0.4 (latest)** | Execution queue (`EX_Queue`), banked vector register file (`V_reg_file`), load/store queue and unit (`LSQ_Queue`, `Load_Store_Unit`), write-back arbiter (`WB_arb`), issue control (`Issue_Control`), performance monitor, VFMA support |

Testbench: `GPU_Core_tb_V0.3(Tested).sv` (also used with V0.4).

Architecture diagrams are in `GPU_Architecture/`.

**Tools:** SystemVerilog, AMD Vivado
