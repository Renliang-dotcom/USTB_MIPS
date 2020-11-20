`timescale 1ns / 1ps

`include "../../../include/bus.v"
`include "../../../include/segpos.v"
`include "../../../include/funct.v"
`include "../../../include/opcode.v"
`include "../../../include/regimm.v"
`include "../../../include/cp0.v"

module ID (
    input                       delayslot_flag_in,
    // load related signals
    input                       load_related_1,
    input                       load_related_2,
    // from IF stage
    input       [`ADDR_BUS]     pc,
    input       [`INST_BUS]     inst,

    // from/to RegFile
    input       [`DATA_BUS]     reg_data_1,
    output  reg                 reg_read_en_1,
    output  reg [`REG_ADDR_BUS] reg_addr_1,

    input       [`DATA_BUS]     reg_data_2,
    output  reg                 reg_read_en_2,
    output  reg [`REG_ADDR_BUS] reg_addr_2,

    // from/to BTB
    input                       btb_pre_taken,
    input       [`ADDR_BUS]     btb_pre_addr,
    output  reg                 btb_set,
    output      [`ADDR_BUS]     btb_set_pc,
    output                      btb_set_taken,
    output      [`ADDR_BUS]     btb_set_target,

    // stall request
    output                      stall_request,

    //next delayslot is error(to rom selector_2_way)
    output  reg                 next_pc_is_error,

    // to IF stage
    output  reg                 branch_flag,
    output  reg [`ADDR_BUS]     branch_addr,
    

    // to EX stage
    output  reg [`FUNCT_BUS]    ex_funct,
    output      [`SHAMT_BUS]    ex_shamt,

    output  reg [`DATA_BUS]     ex_operand_1,
    output  reg [`DATA_BUS]     ex_operand_2,
    output                      delayslot_flag_out,
    output                      next_delayslot_flag_out,

    // to MEM stage
    output  reg                 mem_read_flag,
    output  reg                 mem_write_flag,
    output  reg                 mem_sign_ext_flag,
    output  reg [ 3: 0]         mem_sel,
    output  reg [`DATA_BUS]     mem_write_data,

    // to WB stage 
    output  reg                 wb_reg_write_en,
    output  reg [`REG_ADDR_BUS] wb_reg_write_addr,

    // to cp0
    output  reg                 cp0_write_en,
    output  reg                 cp0_read_en,
    output  reg [`CP0_ADDR_BUS] cp0_addr,
    output  reg [`DATA_BUS]     cp0_write_data,

    //exception signal
    output      [`EXC_TYPE_BUS] exception_type,
    output      [`ADDR_BUS]     current_pc

);

    // extract information from instruction
    wire[`INST_OP_BUS]    inst_op     = inst[`SEG_OPCODE];
    wire[`REG_ADDR_BUS]   inst_rs     = inst[`SEG_RS];
    wire[`REG_ADDR_BUS]   inst_rt     = inst[`SEG_RT];
    wire[`REG_ADDR_BUS]   inst_rd     = inst[`SEG_RD];
    wire[`SHAMT_BUS]      inst_shamt  = inst[`SEG_SHAMT];
    wire[`FUNCT_BUS]      inst_funct  = inst[`SEG_FUNCT];
    wire[`HALF_DATA_BUS]  inst_imm    = inst[`SEG_IMM];
    
    // out signal
    assign ex_shamt             =   inst_shamt;
    assign current_pc           =   pc;
    assign delayslot_flag_out   =   delayslot_flag_in;  
    assign stall_request        =   load_related_1 || load_related_2;

    // Intermediate signal

    // calculate link addresss
    wire[`ADDR_BUS] link_addr = pc + 8;

    // extract immediate from instructions
    wire[`DATA_BUS] zero_ext_imm    = {16'b0, inst_imm};
    wire[`DATA_BUS] zero_ext_imm_hi = {inst_imm, 16'b0};
    wire[`DATA_BUS] sign_ext_imm    = {{16{inst_imm[15]}}, inst_imm};

    wire[`ADDR_BUS] addr_plus_4 = pc + 4;
    wire[25:0] jump_addr = inst[25:0];
    wire[`DATA_BUS] sign_ext_imm_sll2 = {{14{inst[15]}}, inst[15:0], 2'b00};

    reg [`ADDR_BUS] branch_addr_result;
    reg branch_occur, is_branch_inst;

    assign btb_set_pc = pc;
    assign btb_set_taken = branch_occur;
    assign btb_set_target = branch_occur ? branch_addr_result : 0;
    assign next_delayslot_flag_out = is_branch_inst;

    // FunctGen  u_FunctGen (
    //     .op                 (inst_op   ),
    //     .rt                 (inst_rt   ),
    //     .funct              (inst_funct),

    //     .funct_out          (ex_funct  )
    // );

    always @(*) begin
        case (inst_op)
            `OP_SPECIAL : ex_funct <= inst_funct;

            `OP_SPECIAL2: begin
                case (inst_funct)
                    `FUNCT_MADD:    ex_funct <= `FUNCT2_MADD;
                    `FUNCT_MADDU:   ex_funct <= `FUNCT2_MADDU;
                    `FUNCT_MUL:     ex_funct <= `FUNCT2_MUL;
                    `FUNCT_MSUB:    ex_funct <= `FUNCT2_MSUB;
                    `FUNCT_MSUBU:   ex_funct <= `FUNCT2_MSUBU;
                    `FUNCT_CLZ:     ex_funct <= `FUNCT2_CLZ;
                    `FUNCT_CLO:     ex_funct <= `FUNCT2_CLO;
                    default:        ex_funct <= `FUNCT_NOP;
                endcase
            end

            `OP_ANDI: ex_funct <= `FUNCT_AND;

            `OP_LUI, `OP_ORI, `OP_JAL: ex_funct <= `FUNCT_OR;

            `OP_XORI: ex_funct <= `FUNCT_XOR;

            `OP_LB, `OP_LBU, `OP_LH, `OP_LHU, `OP_LW,
            `OP_SB, `OP_SH, `OP_SW, `OP_ADDI: ex_funct <= `FUNCT_ADD;

            `OP_ADDIU: ex_funct <= `FUNCT_ADDU;

            `OP_SLTI: ex_funct <= `FUNCT_SLT;

            `OP_SLTIU: ex_funct <= `FUNCT_SLTU;

            `OP_REGIMM: begin
                case(inst_rt)
                    `REGIMM_BLTZAL, `REGIMM_BGEZAL: ex_funct <= `FUNCT_OR;
                    default: ex_funct <= `FUNCT_NOP;
                endcase
            end

            default: ex_funct <= `FUNCT_NOP;
        endcase
    end

    // OperandGen  u_OperandGen (
    //     .pc                 (pc          ),
    //     .op                 (inst_op     ),
    //     .rt                 (inst_rt     ),
    //     .funct              (inst_funct  ),
    //     .imm                (inst_imm    ),
    //     .reg_data_1         (reg_data_1  ),
    //     .reg_data_2         (reg_data_2  ),

    //     .operand_1          (ex_operand_1),
    //     .operand_2          (ex_operand_2)
    // );

    // generate operand_1
    always @(*) begin
        case (inst_op)
            `OP_SPECIAL: begin
                ex_operand_1 <= inst_funct == `FUNCT_JALR ? link_addr : reg_data_1;
            end 
            // immediate
            `OP_ADDI, `OP_ADDIU, `OP_SLTI, `OP_SLTIU,
            `OP_ANDI, `OP_ORI, `OP_XORI, `OP_LUI,
            // memory accessing
            `OP_LB, `OP_LH, `OP_LW, `OP_LBU,
            `OP_LHU, `OP_SB, `OP_SH, `OP_SW,
            `OP_SPECIAL2: begin
                ex_operand_1 <= reg_data_1;
            end
            `OP_REGIMM: begin
                ex_operand_1 <= inst_rt == `REGIMM_BLTZAL || inst_rt == `REGIMM_BGEZAL ? link_addr : 0;
            end
            `OP_JAL: begin
                ex_operand_1 <= link_addr;
            end
            default: begin              //LUI
                ex_operand_1 <= 0;
            end
        endcase
    end

    // generate operand_2
    always @(*) begin
        case (inst_op)
            `OP_LUI: begin
                ex_operand_2 <= zero_ext_imm_hi;
            end
            // arithmetic & logic (immediate)
            `OP_ADDI, `OP_ADDIU, `OP_SLTI, `OP_SLTIU,
            // memory accessing
            `OP_LB, `OP_LH, `OP_LW, `OP_LBU,
            `OP_LHU, `OP_SB, `OP_SH, `OP_SW: begin
                ex_operand_2 <= sign_ext_imm;
            end
            `OP_SPECIAL: begin
                ex_operand_2 <= reg_data_2;
            end
            `OP_XORI, `OP_ANDI, `OP_ORI: begin
                ex_operand_2 <= zero_ext_imm;
            end 
            default: begin
                ex_operand_2 <= 0;
            end
        endcase
    end

    // RegGen  u_RegGen (
    //     .inst               (inst             ),
    //     .op                 (inst_op          ),
    //     .rs                 (inst_rs          ),
    //     .rt                 (inst_rt          ),
    //     .rd                 (inst_rd          ),

    //     .reg_read_en_1      (reg_read_en_1    ),
    //     .reg_read_en_2      (reg_read_en_2    ),
    //     .reg_addr_1         (reg_addr_1       ),
    //     .reg_addr_2         (reg_addr_2       ),
    //     .reg_write_en       (wb_reg_write_en  ),
    //     .reg_write_addr     (wb_reg_write_addr)
    // );

    // generate read address
    always @(*) begin
        case (inst_op)
            // branch
            `OP_BEQ, `OP_BNE, `OP_BLEZ, `OP_BGTZ,
            // memory accessing
            `OP_SB, `OP_SH, `OP_SW,
            // r-type
            `OP_SPECIAL, `OP_SPECIAL2: begin
                reg_read_en_1   <= 1;
                reg_addr_1      <= inst_rs;    
                reg_read_en_2   <= 1;
                reg_addr_2      <= inst_rt;
            end
            // arithmetic & logic (immediate)
            `OP_ADDI, `OP_ADDIU, `OP_SLTI, `OP_SLTIU,
            `OP_ANDI, `OP_ORI, `OP_XORI,
            // memory accessing
            `OP_LB, `OP_LH, `OP_LW, `OP_LBU, `OP_LHU: begin
                reg_read_en_1   <= 1;
                reg_addr_1      <= inst_rs;    
                reg_read_en_2   <= 0;
                reg_addr_2      <= 0;
            end 
            // reg-imm
            `OP_REGIMM: begin
                case(inst_rt)
                    `REGIMM_BLTZ, `REGIMM_BLTZAL,
                    `REGIMM_BGEZ, `REGIMM_BGEZAL: begin
                        reg_read_en_1   <= 1;
                        reg_addr_1      <= inst_rs;    
                        reg_read_en_2   <= 0;
                        reg_addr_2      <= 0;
                    end
                    default: begin
                        reg_read_en_1   <= 0;
                        reg_addr_1      <= 0;    
                        reg_read_en_2   <= 0;
                        reg_addr_2      <= 0;
                    end
                endcase
            end
            `OP_CP0: begin
                reg_read_en_1   <= 1;
                reg_addr_1      <= inst_rt;    
                reg_read_en_2   <= 0;
                reg_addr_2      <= 0;
            end
            default: begin              //LUI, JAL, J
                reg_read_en_1   <= 0;
                reg_addr_1      <= 0;    
                reg_read_en_2   <= 0;
                reg_addr_2      <= 0;
            end 
        endcase
    end

    // generate write address
    always @(*) begin
        case (inst_op)
            `OP_SPECIAL, `OP_SPECIAL2: begin
                wb_reg_write_en    <= 1;
                wb_reg_write_addr  <= inst_rd;
            end
            // load
            `OP_LB, `OP_LBU, `OP_LH, `OP_LHU, `OP_LW,
            // immediate
            `OP_ADDI, `OP_ADDIU, `OP_SLTI, `OP_SLTIU,
            `OP_ANDI, `OP_ORI, `OP_XORI, `OP_LUI: begin
                wb_reg_write_en    <= 1;
                wb_reg_write_addr  <= inst_rt;
            end 
            `OP_JAL: begin
                wb_reg_write_en    <= 1;
                wb_reg_write_addr  <= 31;
            end
            `OP_REGIMM: begin
                case(inst_rt) 
                    `REGIMM_BGEZAL, `REGIMM_BLTZAL: begin
                        wb_reg_write_en    <= 1;
                        wb_reg_write_addr  <= 31;
                    end
                    default: begin
                        wb_reg_write_en    <= 0;
                        wb_reg_write_addr  <= 0;
                    end
                endcase
            end
            `OP_CP0: begin
                if(inst_rs == `CP0_MFC0 && inst[10 : 3] == 0) begin
                    wb_reg_write_en    <= 1;
                    wb_reg_write_addr  <= inst_rt;
                end
                else begin
                    wb_reg_write_en    <= 0;
                    wb_reg_write_addr  <= 0;
                end
            end
            default: begin
                wb_reg_write_en    <= 0;
                wb_reg_write_addr  <= 0;
            end
        endcase
    end

    // MemGen  u_MemGen (
    //     .op                 (inst_op          ),
    //     .reg_data_2         (reg_data_2       ),

    //     .mem_read_flag      (mem_read_flag    ),
    //     .mem_write_flag     (mem_write_flag   ),
    //     .mem_sign_ext_flag  (mem_sign_ext_flag),
    //     .mem_sel            (mem_sel          ),
    //     .mem_write_data     (mem_write_data   )
    // );

    // mem_write_flag
    always @(*) begin
        case (inst_op)
            `OP_SB, `OP_SH, `OP_SW: mem_write_flag <= 1;
            default: mem_write_flag <= 0;
        endcase
    end

    // mem_read_flags
    always @(*) begin
        case (inst_op)
            `OP_LB, `OP_LBU, `OP_LH, `OP_LHU, `OP_LW: mem_read_flag <= 1;
            default: mem_read_flag <= 0;
        endcase
    end

    // mem_sign_ext_flag
    always @(*) begin
        case (inst_op)
            `OP_LB, `OP_LH, `OP_LW: mem_sign_ext_flag <= 1;
            default: mem_sign_ext_flag <= 0;
        endcase
    end

    // mem_sel
    always @(*) begin
        case (inst_op)
            `OP_LB, `OP_LBU, `OP_SB: mem_sel <= 4'b0001;
            `OP_LH, `OP_LHU, `OP_SH: mem_sel <= 4'b0011;
            `OP_LW, `OP_SW: mem_sel <= 4'b1111;
            default: mem_sel <= 0;
        endcase
    end

    // mem_write_data
    always @(*) begin
        case (inst_op)
            `OP_SB, `OP_SH, `OP_SW: mem_write_data <= reg_data_2;
            default: mem_write_data <= 0;
        endcase
    end

    // BranchGen  u_BranchGen (
    //     .pc                       ( pc                        ),
    //     .inst                     ( inst                      ),
    //     .op                       ( inst_op                   ),
    //     .funct                    ( inst_funct                ),
    //     .rt                       ( inst_rt                   ),
    //     .reg_data_1               ( reg_data_1                ),
    //     .reg_data_2               ( reg_data_2                ),
    //     .btb_pre_taken            ( btb_pre_taken             ),
    //     .btb_pre_addr             ( btb_pre_addr              ),

    //     .btb_set                  ( btb_set                   ),
    //     .btb_set_pc               ( btb_set_pc                ),
    //     .btb_set_taken            ( btb_set_taken             ),
    //     .btb_set_target           ( btb_set_target            ),
    //     .branch_flag              ( branch_flag               ),
    //     .branch_addr              ( branch_addr               ),
    //     .next_delayslot_flag_out  ( next_delayslot_flag_out   ),
    //     .next_pc_is_error         ( next_pc_is_error          )
    // );

    always @(*) begin
        // if (inst_in_effect) begin
        case (inst_op)
            `OP_J, `OP_JAL: begin 
                branch_addr_result <= {addr_plus_4[31:28], jump_addr, 2'b0};
                branch_occur <= 1;
                is_branch_inst <= 1;
            end 
            `OP_SPECIAL, `OP_SPECIAL2: begin
                if(inst_funct == `FUNCT_JR || inst_funct == `FUNCT_JALR) begin 
                    branch_addr_result <= reg_data_1;
                    branch_occur <= 1;
                    is_branch_inst <= 1;
                end
                else begin
                    branch_addr_result <= 0;
                    branch_occur <= 0;
                    is_branch_inst <= 0;
                end 
            end 
            `OP_BEQ: begin
                if (reg_data_1 == reg_data_2) begin
                    branch_addr_result <= addr_plus_4 + sign_ext_imm_sll2;
                    branch_occur <= 1;
                end
                else begin
                    branch_addr_result <= 0;
                    branch_occur <= 0;
                end
                is_branch_inst <= 1;
            end 
            `OP_BGTZ: begin
                if (!reg_data_1[31] && reg_data_1) begin
                    branch_addr_result <= addr_plus_4 + sign_ext_imm_sll2;
                    branch_occur <= 1;
                end
                else begin
                    branch_addr_result <= 0;
                    branch_occur <= 0;
                end
                is_branch_inst <= 1;
            end
            
            `OP_BLEZ: begin
                if (reg_data_1[31] || !reg_data_1) begin
                    branch_addr_result <= addr_plus_4 + sign_ext_imm_sll2;
                    branch_occur <= 1;
                end
                else begin
                    branch_addr_result <= 0;
                    branch_occur <= 0;
                end
                is_branch_inst <= 1;
            end 
            `OP_BNE: begin
                if (reg_data_1 != reg_data_2) begin
                    branch_addr_result <= addr_plus_4 + sign_ext_imm_sll2;
                    branch_occur <= 1;
                end
                else begin
                    branch_addr_result <= 0;
                    branch_occur <= 0;
                end
                is_branch_inst <= 1;
            end 
            `OP_REGIMM: begin
                case (inst_rt)
                    `REGIMM_BLTZ, `REGIMM_BLTZAL: begin
                        if (reg_data_1[31]) begin
                            branch_addr_result <= addr_plus_4 + sign_ext_imm_sll2;
                            branch_occur <= 1;
                        end
                        else begin
                            branch_addr_result <= 0;
                            branch_occur <= 0;
                        end
                        is_branch_inst <= 1;
                    end 
                    `REGIMM_BGEZ, `REGIMM_BGEZAL: begin
                        if (!reg_data_1[31]) begin
                            branch_addr_result <= addr_plus_4 + sign_ext_imm_sll2;
                            branch_occur <= 1;
                        end
                        else begin
                            branch_addr_result <= 0;
                            branch_occur <= 0;
                        end
                        is_branch_inst <= 1;
                    end 
                    default: begin
                        branch_addr_result <= 0;
                        branch_occur <= 0;
                        is_branch_inst <= 0;
                    end 
                endcase
            end
            default: begin
                branch_addr_result <= 0;
                branch_occur <= 0;
                is_branch_inst <= 0;
            end 
        endcase
    end

    always @(*) begin
        if(is_branch_inst) begin
            if(btb_pre_taken) begin
                if(branch_occur) begin
                    if (btb_pre_addr == branch_addr_result) begin
                        next_pc_is_error <= 0;
                        branch_addr <= branch_addr_result;
                        branch_flag <= 0;
                    end
                    else begin
                        next_pc_is_error <= 0;
                        branch_addr <= branch_addr_result;
                        branch_flag <= 1;
                    end
                    btb_set <= 1;
                end
                else begin
                    next_pc_is_error <= 0;
                    branch_addr <= pc + 8;
                    branch_flag <= 1;
                    btb_set <= 0;
                end
            end
            else begin
                if (branch_occur) begin
                    next_pc_is_error <= 0;
                    branch_addr <= branch_addr_result;
                    branch_flag <= 1;
                    btb_set <= 1;
                end
                else begin
                    next_pc_is_error <= 0;
                    branch_addr <= 0;
                    branch_flag <= 0;
                    btb_set <= 0;
                end
            end
        end
        else begin
            if(btb_pre_taken) begin
                next_pc_is_error <= 1;
                branch_addr <= addr_plus_4;
                branch_flag <= 1;
            end
            else begin
                next_pc_is_error <= 0;
                branch_addr <= 0;
                branch_flag <= 0;
            end
            btb_set <= 0;
        end
    end

    // CP0Gen  u_CP0Gen (
    //     .inst                    ( inst             ),
    //     .op                      ( inst_op          ),
    //     .rs                      ( inst_rs          ),
    //     .rd                      ( inst_rd          ),
    //     .reg_data_1              ( reg_data_1       ),

    //     .cp0_write_en            ( cp0_write_en     ),
    //     .cp0_read_en             ( cp0_read_en      ),
    //     .cp0_write_data          ( cp0_write_data   ),
    //     .cp0_addr                ( cp0_addr         )
    // );

    always @(*) begin
        case (inst_op)
            `OP_CP0: begin
                if (inst_rs == `CP0_MTC0 && inst[10:3] == 0) begin
                    cp0_write_en <= 1;
                    cp0_read_en <= 0;
                    cp0_write_data <= reg_data_1;
                    cp0_addr <= {inst_rd, inst[2:0]};
                end
                else if (inst_rs == `CP0_MFC0 && inst[10:3] == 0) begin
                    cp0_write_en <= 0;
                    cp0_read_en <= 1;
                    cp0_write_data <= 0;
                    cp0_addr <= {inst_rd, inst[2:0]};
                end
                else begin
                    cp0_write_en <= 0;
                    cp0_read_en <= 0;
                    cp0_write_data <= 0;
                    cp0_addr <= 0;
                end
            end 
            default:begin
                cp0_write_en <= 0;
                cp0_read_en <= 0;
                cp0_write_data <= 0;
                cp0_addr <= 0;
            end 
        endcase
    end

    wire eret_flag, syscall_flag, break_flag, overflow_flag;
    reg invalid_inst_flag;
    assign exception_type ={
        eret_flag, /* ADE */ 1'b0,
        syscall_flag, break_flag, /* TP */ 1'b0,
        overflow_flag, invalid_inst_flag, /* IF */ 1'b0};
    
    // ExceptionGen  u_ExceptionGen (
    //     .inst                    ( inst                ),
    //     .op                      ( inst_op             ),
    //     .rs                      ( inst_rs             ),
    //     .rt                      ( inst_rt             ),
    //     .funct                   ( inst_funct          ),

    //     .eret_flag               ( eret_flag           ),
    //     .syscall_flag            ( syscall_flag        ),
    //     .break_flag              ( break_flag          ),
    //     .overflow_flag           ( overflow_flag       ),
    //     .invalid_inst_flag       ( invalid_inst_flag   )
    // );

    assign eret_flag = (inst == `CP0_ERET_FULL) ? 1 : 0;
    assign syscall_flag = (inst_op == `OP_SPECIAL && inst_funct == `FUNCT_SYSCALL) ? 1 : 0;
    assign break_flag = (inst_op == `OP_SPECIAL && inst_funct == `FUNCT_BREAK) ? 1 : 0;
    assign overflow_flag = ((inst_op == `OP_SPECIAL && (inst_funct == `FUNCT_ADD || inst_funct == `FUNCT_SUB)) 
                            || (inst_op == `OP_ADDI)) ? 1 : 0;
    always @(*) begin
        case (inst_op)
            `OP_SPECIAL: begin
                case (inst_funct)
                    `FUNCT_SLL, `FUNCT_SRL, `FUNCT_SRA, `FUNCT_SLLV,
                    `FUNCT_SRLV, `FUNCT_SRAV, `FUNCT_JR, `FUNCT_JALR,
                    `FUNCT_MOVN, `FUNCT_MOVZ, `FUNCT_ADD, `FUNCT_SUB,
                    `FUNCT_MFHI, `FUNCT_MTHI, `FUNCT_MFLO, `FUNCT_MTLO,
                    `FUNCT_MULT, `FUNCT_MULTU, `FUNCT_DIV, `FUNCT_DIVU,
                    `FUNCT_ADDU, `FUNCT_SUBU, `FUNCT_AND, `FUNCT_OR,
                    `FUNCT_XOR, `FUNCT_NOR, `FUNCT_SLT, `FUNCT_SLTU,
                    `FUNCT_SYSCALL, `FUNCT_BREAK: begin
                        invalid_inst_flag <= 0;
                    end
                    default: invalid_inst_flag <= 1;
                endcase
            end 

            `OP_SPECIAL2: begin
                case (inst_funct)
                    `FUNCT_MADD, `FUNCT_MUL,
                    `FUNCT_MADDU, `FUNCT_MSUB,
                    `FUNCT_MSUBU: begin
                        invalid_inst_flag <= 0;
                    end 
                    default: invalid_inst_flag <= 1;
                endcase
            end

            `OP_REGIMM: begin
                case (inst_rt)
                    `REGIMM_BLTZ, `REGIMM_BLTZAL, 
                    `REGIMM_BGEZ, `REGIMM_BGEZAL: begin
                        invalid_inst_flag <= 0;
                    end 
                    default: invalid_inst_flag <= 1;
                endcase
            end

            `OP_CP0: begin
                case (inst_rs)
                    `CP0_MFC0, `CP0_MTC0, `CP0_ERET: begin
                        invalid_inst_flag <= 0;
                    end
                    default: invalid_inst_flag <= 1;
                endcase
            end

            `OP_J, `OP_JAL, `OP_BEQ, `OP_BNE, `OP_BLEZ, `OP_BGTZ,
            `OP_ADDIU, `OP_SLTI, `OP_SLTIU, `OP_ANDI, `OP_ORI,
            `OP_XORI, `OP_LUI, `OP_LB, `OP_LH, `OP_LW, `OP_LBU,
            `OP_LHU, `OP_SB, `OP_SH, `OP_SW, `OP_ADDI: begin
                invalid_inst_flag <= 0;
            end

            default: invalid_inst_flag <= 1;
        endcase
    end

endmodule // ID