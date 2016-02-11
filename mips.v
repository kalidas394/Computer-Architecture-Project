 `ifndef _PROC_DEFS
 `define _PROC_DEFS
         
        //`define CODE_FOR_SYNTHESIS				
        `define USE_SIMULATION_CODE				 
        
        `define PC_WIDTH				8
        `define	INSTR_MEM_ADDR_WIDTH	8
        `define MEM_ADD_WIDTH_DATA		8
 
        /************** Operation Code in instructions ****************/
        `define OP_NOP			4'b0000
        `define OP_ADD			4'b0010
        `define OP_SUB			4'b0001
        `define OP_MUL			4'b0011
        `define OP_DIV			4'b0101
        `define OP_MODU			4'b0100
        `define OP_SL			4'b0110
        `define OP_SR			4'b0111
        `define OP_ADDI			4'b1001
        `define OP_LD			4'b1010
        `define OP_ST			4'b1011
        `define OP_BZ			4'b1100
        
        /************** ALU operation command ****************/
        `define ALU_NC			3'bxxx		// Dont do anything 
        `define ALU_ADD			3'b011
        `define ALU_SUB			3'b010
        `define ALU_MUL			3'b000
        `define ALU_DIV			3'b001
        `define ALU_MODU			3'b100
        `define ALU_SL			3'b101
        `define ALU_SR			3'b110
        
        /************** Branch condition code ****************/
        `define BRANCH_Z		3'b000
        //`define BRANCH_GT		3'b001
        //`define BRANCH_LE		3'b010
        
 
 
 `endif
 


module alu
(
        input		[15:0]	a,		//src1
        input		[15:0]	b,		//src2
        input		[2:0]	cmd,	//function sel
        
        output	reg	[15:0]	r		//result	
);
        always @ (*) begin
                case(cmd)
                        `ALU_NC	:
                                r = 16'bx;
                        `ALU_ADD:
                                r = a + b;
                        `ALU_SUB:
                                r = a - b;
                        `ALU_MUL:
                                r = a * b;
                        `ALU_DIV	:
                                r = a / b;
                        `ALU_MODU:
                                r = a % b;
                        `ALU_SL	:
                                r = a << b;
                        `ALU_SR	:
                                r = {{16{a[15]}},a} >> b;
                        //`ALU_SRU	:
                        //	r = {16'b0,a} >> b;
                        default	:
                                begin
                                        r = 0;
`ifndef CODE_FOR_SYNTHESIS
                                        $display("ERROR: Unknown alu cmd: %b \n", cmd);
                                        //$stop;
`endif
                                end
                endcase
        end
        
endmodule 

`ifndef _PROC_DEFS
 `define _PROC_DEFS
         
        //`define CODE_FOR_SYNTHESIS				
        `define USE_SIMULATION_CODE				 
        
        `define PC_WIDTH				8
        `define	INSTR_MEM_ADDR_WIDTH	8
        `define MEM_ADD_WIDTH_DATA		8
 
        /************** Operation Code in instructions ****************/
        `define OP_NOP			4'b0000
        `define OP_ADD			4'b0010
        `define OP_SUB			4'b0001
        `define OP_MUL			4'b0011
        `define OP_DIV			4'b0101
        `define OP_MODU			4'b0100
        `define OP_SL			4'b0110
        `define OP_SR			4'b0111
        `define OP_ADDI			4'b1001
        `define OP_LD			4'b1010
        `define OP_ST			4'b1011
        `define OP_BZ			4'b1100
        
        /************** ALU operation command ****************/
        `define ALU_NC			3'bxxx		// Dont do anything 
        `define ALU_ADD			3'b011
        `define ALU_SUB			3'b010
        `define ALU_MUL			3'b000
        `define ALU_DIV			3'b001
        `define ALU_MODU			3'b100
        `define ALU_SL			3'b101
        `define ALU_SR			3'b110
        
        /************** Branch condition code ****************/
        `define BRANCH_Z		3'b000
        //`define BRANCH_GT		3'b001
        //`define BRANCH_LE		3'b010
        
 
 
 `endif






module data_mem
(
        input					clk,
        
        // address input, shared by read and write port
        input	[15:0]			mem_access_addr,
        
        // write port
        input	[15:0]				mem_write_data,
        //input							mem_write_en,	//Praswrite
        // read port
        output	[15:0]			mem_read_data
        
);


        reg [15:0] ram [(2**`MEM_ADD_WIDTH_DATA)-1:0];

        wire [`MEM_ADD_WIDTH_DATA-1 : 0] ram_addr = mem_access_addr[`MEM_ADD_WIDTH_DATA-1 : 0];

        always @(posedge clk)
                if (mem_write_data)
                        ram[ram_addr] <= mem_write_data;

        assign mem_read_data = ram[ram_addr]; 
   
endmodule 

module EX_stage
(
        input					clk,
        input					rst,
        // from ID_stage
        input		[56:0]		pipeline_reg_in,	
        
        //	[56:22],35bits:	ex_alu_s0s1[2:0] 
        // ex_Rp_data[15:0], ex_Rq_data[15:0]
        //	[21:5],17bits:	mem_write_en, mem_write_data[15:0]
        //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux, 
        
        // to MEM_stage
        output	reg	[37:0]		pipeline_reg_out,	//	[37:22],16bits:	ALU_Output[15:0];
                                                                                                //	[21:5],17bits:	mem_write_en, mem_write_data[15:0]
                                                                                                //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux, 
        
        // to hazard detection unit
        output		[2:0]		ex_op_dest
);
        wire	[2:0]		alu_s0s1	= pipeline_reg_in[56:54];				//S2  //alu_s0 and alu_s1
        wire	[15:0]	Rp_data	= pipeline_reg_in[53:38];						//RF Rp_data and Rq_data
        wire	[15:0]	Rq_data	= pipeline_reg_in[37:22];
        
        wire	[15:0]	ALU_Output;
        
        /********************** ALU *********************/
        alu alu_inst(
                .a		( Rp_data),
                .b		( Rq_data),
                .cmd	( alu_s0s1),	//alu_s0 and alu_s1
                .r		( ALU_Output)
        );
        
        
        /********************** singals to MEM_stage *********************/
        always @ (posedge clk) begin
                if(rst) begin
                        pipeline_reg_out[37:0] <= 0;
                end
                else begin
                        pipeline_reg_out[37:22] <= ALU_Output;
                        pipeline_reg_out[21:0] <= pipeline_reg_in[21:0];
                end
        end
        
        
        /********************** to hazard detection unit *********************/
        assign ex_op_dest = pipeline_reg_in[3:1];
endmodule 

module ID_stage
(
        input					clk,
        input					rst,
        input					instruction_decode_en,
        //input					insert_bubble,
        
        
        // to EX_stage
        output	reg	[56:0]		pipeline_reg_out,	
        
        //	[56:22],35bits:	ex_alu_s0s1[2:0] //alu_s0 and alu_s1, ex_Rp_data[15:0], ex_Rq_data[15:0]
        //	[21:5],17bits:	mem_write_en, mem_write_data[15:0]
        //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux, 
        
        // to IF_stage
        input		[15:0]		instruction,
        output		[5:0]		branch_offset_imm,
        output	reg			branch_taken,
        
        // to register file
        output		[3:0]		Rp_addr,	// register file read port p address
        output		[3:0]		Rq_addr,	// register file read port q address
        input		[15:0]		Rp_data,	// register file read port p data
        input		[15:0]		Rq_data,	// register file read port q data
        
        // to hazard detection unit
        output		[2:0]		decoding_op_src1,		//source_1 register number
        output		[2:0]		decoding_op_src2		//source_2 register number
        
);
    
        /********************** internal wires ***********************************/
        //----------------- Instruction Register signals --------------------//
        reg		[15:0]		instruction_reg;
        wire	[3:0]		ir_op_code;		//operation code
        wire	[2:0]		ir_dest;		//destination register number
        wire	[2:0]		ir_src1;		//source_1 register number
        wire	[2:0]		ir_src2;		//source_2 register number
        wire	[5:0]		ir_imm;			//immediate number carried by the instruction
        
        //---------------- data path control signals --------------------------//
        // write back stage signals
        reg					write_back_en;			// S3
        wire	[2:0]			write_back_dest;		// dest
        reg					write_back_result_mux;	// S1
        // mem stage signals
        //wire					mem_write_en;	//Praswrite
        wire	[15:0]		mem_write_data;
        // ex stage signals
        reg	[2:0]			ex_alu_s0s1;				//S2  //alu_s0 and alu_s1
        wire	[15:0]		ex_Rp_data;
        wire	[15:0]		ex_Rq_data;
        // instruction decode stage signals
        reg					Rq_data_mux;			// S4
        wire				decoding_op_is_branch;	//S5	// Check if Branch Instruction
        wire				decoding_op_is_store;	//S6  // Check if Store Instruction
        wire	[3:0]		ir_op_code_with_bubble;
        wire	[2:0]		ir_dest_with_bubble;
        //reg					branch_condition_satisfied;
        
        
        /********************** Instruction Register *********************/
        always @ (posedge clk or posedge rst) begin
                if(rst) begin
                        instruction_reg <= 0;
                end
                else begin
                        if(instruction_decode_en) begin
                                instruction_reg <= instruction;
                        end
                end
        end
        assign ir_op_code = instruction_reg[15:12];
        assign ir_dest = instruction_reg[11: 9];
        assign ir_src1 = instruction_reg[ 8: 6];
        assign ir_src2 = (decoding_op_is_store)? instruction_reg[11: 9] : instruction_reg[ 5: 3];
        assign ir_imm  = instruction_reg[ 5: 0];
        
        /********************** pipeline bubble insertion *********************/
        // if instrcution decode is frozen, insert bubble operations into the pipeline
        assign ir_op_code_with_bubble = ( instruction_decode_en )?  ir_op_code : 0;
        // if instrcution decode is frozen, force destination reg number to 0, 
        // this operation is to prevent pipeline stall.
        assign ir_dest_with_bubble = ( instruction_decode_en )?  ir_dest : 0;
        
        /********************** Data path control logic *********************/
        always @ (*) begin
                if(rst) begin
                        write_back_en			= 0;	// S3
                        write_back_result_mux	= 0;	// S1
                        ex_alu_s0s1				= 0;	// S2	//alu_s0 and alu_s1
                        Rq_data_mux			= 0;	// S4
                end
                else begin
                        case( ir_op_code_with_bubble )
                                `OP_NOP	:
                                        begin
                                                write_back_en			= 0;		// S3
                                                write_back_result_mux	= 1'bx;		// S1
                                                ex_alu_s0s1				= `ALU_NC;	// S2
                                                Rq_data_mux			= 1'bx;		// S4
                                        end
                                `OP_ADD	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_ADD;	// S2
                                                Rq_data_mux			= 0;		// S4
                                        end
                                `OP_SUB	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_SUB;	// S2
                                                Rq_data_mux			= 0;		// S4
                                        end
                                `OP_MUL	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_MUL;	// S2
                                                Rq_data_mux			= 0;		// S4
                                        end
                                `OP_DIV	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_DIV;	// S2
                                                Rq_data_mux			= 0;		// S4
                                        end
                                `OP_MODU	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_MODU;	// S2
                                                Rq_data_mux			= 1'bx;		// S4
                                        end
                                `OP_SL	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_SL;	// S2
                                                Rq_data_mux			= 0;		// S4
                                        end
                                `OP_SR	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_SR;	// S2
                                                Rq_data_mux			= 0;		// S4
                                        end
                                `OP_ADDI:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 0;		// S1
                                                ex_alu_s0s1				= `ALU_ADD;	// S2
                                                Rq_data_mux			= 1;		// S4
                                        end
                                `OP_LD	:
                                        begin
                                                write_back_en			= 1;		// S3
                                                write_back_result_mux	= 1;		// S1
                                                ex_alu_s0s1				= `ALU_ADD;	// S2
                                                Rq_data_mux			= 1;		// S4
                                        end
                                `OP_ST	:
                                        begin
                                                write_back_en			= 0;		// S3
                                                write_back_result_mux	= 1'bx;		// S1
                                                ex_alu_s0s1				= `ALU_ADD;	// S2
                                                Rq_data_mux			= 1;		// S4
                                        end
                                `OP_BZ	:
                                        begin
                                                write_back_en			= 0;		// S3
                                                write_back_result_mux	= 1'bx;		// S1
                                                ex_alu_s0s1				= `ALU_NC;	// S2
                                                Rq_data_mux			= 1;		// S4
                                        end
                                default	:
                                        begin
                                                write_back_en			= 0;		// S3
                                                write_back_result_mux	= 1'bx;		// S1
                                                ex_alu_s0s1				= `ALU_NC;	// S2
                                                Rq_data_mux			= 1'bx;		// S4
`ifndef CODE_FOR_SYNTHESIS
                                                $display("ERROR: Unknown Instruction: %b", ir_op_code_with_bubble);
                                                //$stop;
`endif
                                        end
                        endcase
                end
        end
        
        assign decoding_op_is_branch = ( ir_op_code == `OP_BZ )? 1 : 0;	// S5
        assign decoding_op_is_store	= ( ir_op_code == `OP_ST )? 1 : 0;	// S6
        
        /********************** singals to EX_stage *********************/
        assign mem_write_data = Rq_data;
        //assign mem_write_en = decoding_op_is_store;	//Praswrite
        assign write_back_dest = ir_dest_with_bubble;
        assign ex_Rp_data = Rp_data;
        assign ex_Rq_data = (Rq_data_mux)? {{10{ir_imm[5]}},ir_imm} : Rq_data;
        
        //	pipeline_reg_out:
        //	[56:22],35bits:	ex_alu_s0s1[2:0], ex_Rp_data[15:0], ex_Rq_data[15:0],
        //	[21:5],17bits:	mem_write_en, mem_write_data[15:0],
        //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux,
        
        always @ (posedge clk or posedge rst) begin
                if(rst) begin
                        pipeline_reg_out[56:0] <= 0;
                end
                else begin
                        pipeline_reg_out[56:0] <= {
                                ex_alu_s0s1[2:0],		// pipeline_reg_out[56:54]	//S2
                                ex_Rp_data[15:0],		// pipeline_reg_out[53:38]	// Rp_data and Rq_data
                                ex_Rq_data[15:0],		// pipeline_reg_out[37:22]	
                                //mem_write_en, 			// pipeline_reg_out[21]		//	//Praswrite
                                mem_write_data[15:0],	// pipeline_reg_out[20:5]	//
                                write_back_en, 			// pipeline_reg_out[4]		//S3
                                write_back_dest[2:0], 	// pipeline_reg_out[3:1]	//dest
                                write_back_result_mux 	// pipeline_reg_out[0]		//S1
                                };
                end
        end
        
                         
        /********************** interface with register file *********************/
        assign Rp_addr = ir_src1;
        assign Rq_addr = ir_src2;
        
        /********************** branch signals generate *********************/
        always @ (*) begin
                if(decoding_op_is_branch) begin
                        case( ir_dest_with_bubble )
                                `BRANCH_Z	:
                                        begin
                                                if(Rp_data == 0)
                                                        branch_taken = 1;
                                                else
                                                        branch_taken = 0;
                                        end
                                        
                                default:
                                        begin
                                                branch_taken = 0;
`ifndef CODE_FOR_SYNTHESIS
                                                $display("ERROR: Unknown branch condition %b, in branch instruction %b \n", ir_dest_with_bubble, ir_op_code_with_bubble);
                                                //$stop;
`endif					
                                        end
                        endcase
                end
                else begin
                        branch_taken = 0;
                end
        end
        assign branch_offset_imm = ir_imm;
        //assign branch_taken = decoding_op_is_branch & branch_condition_satisfied ;
        
        /********************** to hazard detection unit *********************/
        assign decoding_op_src1 = ir_src1;
        assign decoding_op_src2 = (
                                        ir_op_code == `OP_NOP 	||
                                        ir_op_code == `OP_ADDI 	||
                                        ir_op_code == `OP_LD 	||
                                        ir_op_code == `OP_BZ 	
                                        )?
                                        3'b000 : ir_src2;
        
endmodule 

module IF_stage
(
        input							clk,
        input							rst,				
        input							instruction_fetch_en,
        
        input	[5:0]					branch_offset_imm,
        input							branch_taken,
        
        output	reg	[`PC_WIDTH-1:0]		pc,
        output	[15:0]					instruction
);
    
        // The program Counter is controlled below.
        always @ (posedge clk or posedge rst) begin
            if (rst) begin
                pc <= `PC_WIDTH'b0;
            end 
                else begin
                        if(instruction_fetch_en) begin
                                if(branch_taken)
                                        pc <= pc + {{(`PC_WIDTH-6){branch_offset_imm[5]}}, branch_offset_imm[5:0]};	
                                else
                                        pc <= pc + `PC_WIDTH'd1;
                        end
                end
        end
        
        // instantiation for the instruction memory
        instruction_mem imem(
                .clk				(clk),
                .pc				(pc),		
                .instruction	(instruction)
        );
        
        
endmodule 

module instruction_mem		
(
        input					clk,
        input	[`PC_WIDTH-1:0]	pc,
        
        output	[15:0]			instruction
);
        
        reg	[15:0] rom [2**`INSTR_MEM_ADDR_WIDTH-1 : 0];
        
        wire [`INSTR_MEM_ADDR_WIDTH-1 : 0] rom_addr = pc[`INSTR_MEM_ADDR_WIDTH-1 : 0];
        
        // always @ (posedge clk) begin
        // always @ (*) begin
            // instruction = rom[rom_addr];
        // end
        
        assign instruction = rom[rom_addr];
        
        
endmodule 


`ifndef USE_SIMULATION_CODE		
module instruction_mem		// a synthesisable rom implementation
(
        input					clk,		// asynchronized!!
        input	[`PC_WIDTH-1:0]	pc,
        
        output reg	[15:0]		instruction
);
        
        wire [`INSTR_MEM_ADDR_WIDTH-1 : 0] rom_addr = pc[`INSTR_MEM_ADDR_WIDTH-1 : 0];
        
        
        // Algo. State Machine 
        // ASM code in rom:
        // L1:	ADDI		R1,R0,8
        // 		ADDI		R2,R1,8
        // 		ADDI		R3,R2,8
        // 		ADD			R4,R2,R3
        // 		ST			R4,R1,2
        // 		LD			R5,R1,2
        // 		SUB			R6,R4,R5
        // 		BZ			R6,L1
        // 		ADDI		R7,R7,1
        always @(*)
                case (rom_addr)
                        4'b0000: instruction = 16'b1001001000001000;
                        4'b0001: instruction = 16'b1001010001001000;
                        4'b0010: instruction = 16'b1001011010001000;
                        4'b0011: instruction = 16'b0001100010011000;
                        4'b0100: instruction = 16'b1011100001000010;
                        4'b0101: instruction = 16'b1010101001000010;
                        4'b0110: instruction = 16'b0010110100101000;
                        4'b0111: instruction = 16'b1100000110111000;
                        4'b1000: instruction = 16'b1001111111000001;
                        4'b1001: instruction = 16'b0000000000000000;
                        4'b1010: instruction = 16'b0000000000000000;
                        4'b1011: instruction = 16'b0000000000000000;
                        4'b1100: instruction = 16'b0000000000000000;
                        4'b1101: instruction = 16'b0000000000000000;
                        4'b1110: instruction = 16'b0000000000000000;
                        4'b1111: instruction = 16'b0000000000000000;
                        default: instruction = 16'b0000000000000000;
         endcase	
endmodule 
`endif

module MEM_stage
(
        input					clk,
        input					rst,
        
        // from EX_stage
        input		[37:0]		pipeline_reg_in,	//	[37:22],16bits:	ALU_Output[15:0];
                                                                                                //	[21:5],17bits:	mem_write_en, mem_write_data[15:0]
                                                                                                //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux, 
        
        // to WB_stage
        output	reg	[36:0]		pipeline_reg_out,	//	[36:21],16bits:	ALU_Output[15:0]
                                                                                                //	[20:5],16bits:	mem_read_data[15:0]
                                                                                                //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux, 
        output		[2:0]		mem_op_dest
);
        
        wire	[15:0]		ALU_Output = pipeline_reg_in[37:22];
        //wire					mem_write_en = pipeline_reg_in[21];		//Praswrite
        wire	[15:0]		mem_write_data = pipeline_reg_in[20:5];
        
        wire	[15:0]		mem_read_data ;		
        
        /********************** Data memory *********************/
        // a ram
        /*data_mem dmem (			//Praswrite
                .clk(clk), 
                .mem_access_addr		( ALU_Output ), 
                .mem_write_data		( mem_write_data ), 
                .mem_write_en			( mem_write_en ), 
                .mem_read_data			( mem_read_data )
        );*/
        
        /********************** singals to WB_stage *********************/
        always @ (posedge clk) begin
                if(rst) begin
                        pipeline_reg_out[36:0] <= 0;
                end
                else begin
                        pipeline_reg_out[36:21] <= ALU_Output;
                        pipeline_reg_out[20:5]	<= mem_read_data ;
                        pipeline_reg_out[4:0] 	<= pipeline_reg_in[4:0];
                end
        end

        assign mem_op_dest = pipeline_reg_in[3:1];

endmodule 

module Sim_Processor_Alu
(
        input						clk,
        input						rst,

        output	[`PC_WIDTH-1:0]		pc
);
        wire 						pipeline_stall_n ;
        wire	[5:0]				branch_offset_imm;
        wire						branch_taken;
        wire	[15:0]				instruction;
        wire	[56:0]				ID_pipeline_reg_out;
        wire	[37:0]				EX_pipeline_reg_out;
        wire	[36:0]				MEM_pipeline_reg_out;
        
        wire	[3:0]				Rp_addr;	// register file read port p address
        wire	[3:0]				Rq_addr;	// register file read port q address
        wire	[15:0]			Rp_data;	// register file read port p data
        wire	[15:0]			Rq_data;	// register file read port q data
        wire	[2:0]				decoding_op_src1;		//source_1 register number
        wire	[2:0]				decoding_op_src2;		//source_2 register number
        wire	[2:0]				ex_op_dest;				//EX stage destinaton register number
        wire	[2:0]				mem_op_dest;			//MEM stage destinaton register number
        wire	[2:0]				wb_op_dest;				//WB stage destinaton register number
        wire						reg_write_en;
        wire	[2:0]				reg_write_dest;
        wire	[15:0]			W_data;
        
        IF_stage IF_stage_inst (
                .clk					(clk), 
                .rst					(rst), 
                .instruction_fetch_en	(pipeline_stall_n),
                .branch_offset_imm		(branch_offset_imm), 
                .branch_taken			(branch_taken), 
                .pc						(pc),
                .instruction			(instruction)
        );
        
        ID_stage ID_stage_inst (
                .clk					(clk),
                .rst					(rst),
                .instruction_decode_en	(pipeline_stall_n),
                .pipeline_reg_out		(ID_pipeline_reg_out),
                .instruction			(instruction),
                .branch_offset_imm		(branch_offset_imm),
                .branch_taken			(branch_taken),
                .Rp_addr		(Rp_addr),	//
                .Rq_addr		(Rq_addr),	//
                .Rp_data		(Rp_data),	//
                .Rq_data		(Rq_data),	//
                .decoding_op_src1		(decoding_op_src1),		
                .decoding_op_src2		(decoding_op_src2)
        );
        
        EX_stage EX_stage_inst (
                .clk					(clk), 
                .rst					(rst), 
                .pipeline_reg_in		(ID_pipeline_reg_out), 
                .pipeline_reg_out		(EX_pipeline_reg_out), 
                .ex_op_dest				(ex_op_dest)
        );
        
        MEM_stage MEM_stage_inst (
                .clk					(clk), 
                .rst					(rst), 
                .pipeline_reg_in		(EX_pipeline_reg_out), 
                .pipeline_reg_out		(MEM_pipeline_reg_out), 
                .mem_op_dest			(mem_op_dest)
        );
        
        WB_stage WB_stage_inst (
                .pipeline_reg_in		(MEM_pipeline_reg_out), 
                .reg_write_en			(reg_write_en), 
                .reg_write_dest			(reg_write_dest), 
                .W_data			(W_data), 
                .wb_op_dest				(wb_op_dest)
        );
        
        register_file register_file_inst (
                .clk					(clk), 
                .rst					(rst), 
                .reg_write_en			(reg_write_en), 
                .reg_write_dest			(reg_write_dest), 
                .W_data			(W_data), 
                .Rp_addr		(Rp_addr), 
                .Rp_data		(Rp_data), 
                .Rq_addr		(Rq_addr), 
                .Rq_data		(Rq_data)
        );
        
        /*
        hazard_detection_unit hazard_detection_unit_inst (
                .decoding_op_src1		(decoding_op_src1), 
                .decoding_op_src2		(decoding_op_src2), 
                .ex_op_dest				(ex_op_dest), 
                .mem_op_dest			(mem_op_dest), 
                .wb_op_dest				(wb_op_dest), 
                .pipeline_stall_n		(pipeline_stall_n)
        );
        */
endmodule 

module register_file
(
        input				clk,
        input				rst,
        
        // write port
        input						reg_write_en,
        input		[2:0]			reg_write_dest,
        input		[15:0]		W_data,
        
        //read port p
        input		[3:0]			Rp_addr,
        output	[15:0]		Rp_data,
        //read port q
        input		[3:0]			Rq_addr,
        output	[15:0]		Rq_data
);
        reg	[15:0]			RF_array [7:0];
        
        // write port
        //reg [2:0] i;
        always @ (posedge clk or posedge rst) begin
                if(rst) begin
                        // for(i=0; i<8; i=i+1)
                                // RF_array[i] <= 15'b0;
                        RF_array[0] <= 15'b0;
                        RF_array[1] <= 15'b0;
                        RF_array[2] <= 15'b0;
                        RF_array[3] <= 15'b0;
                        RF_array[4] <= 15'b0;
                        RF_array[5] <= 15'b0;
                        RF_array[6] <= 15'b0;
                        RF_array[7] <= 15'b0;
                end
                else begin
                        if(reg_write_en) begin
                                RF_array[reg_write_dest] <= W_data;
                        end
                end
                
        end
        
        //read port p
        // always @ (*) begin
                // if( Rp_addr == 0) begin
                        // Rp_data = 15'b0;
                // end
                // else begin
                        // Rp_data = RF_array[Rp_addr];
                // end
        // end
        assign Rp_data = ( Rp_addr == 0)? 15'b0 : RF_array[Rp_addr];
        
        //read port q
        // always @ (*) begin
                // if( Rq_addr == 0) begin
                        // Rq_data = 15'b0;
                // end
                // else begin
                        // Rq_data = RF_array[Rq_addr];
                // end
        // end
        assign Rq_data = ( Rq_addr == 0)? 15'b0 : RF_array[Rq_addr];

endmodule 


module WB_stage
(
        //input					clk,
        
        // from EX stage
        input		[36:0]		pipeline_reg_in,	
        
        // Below is the structure of the Register
        //	[36:21],16bits:	ALU_Output[15:0]
        //	[20:5],16bits:	mem_read_data[15:0]
        //	[4:0],5bits:	write_back_en, write_back_dest[2:0], write_back_result_mux, 
        
        // to register file
        output					reg_write_en,
        output		[2:0]		reg_write_dest,
        output		[15:0]	W_data,
        
        output		[2:0]		wb_op_dest
);
        
        wire [15:0]	ALU_Output = pipeline_reg_in[36:21];
        wire [15:0]	mem_read_data = pipeline_reg_in[20:5];
        wire		write_back_en = pipeline_reg_in[4];
        wire [2:0]	write_back_dest = pipeline_reg_in[3:1];
        wire		write_back_result_mux = pipeline_reg_in[0];
        
        /********************** to register file *********************/
        assign reg_write_en = write_back_en;
        assign reg_write_dest = write_back_dest;
        assign W_data = (write_back_result_mux)? mem_read_data : ALU_Output;
        
        /********************** to hazard detection unit *********************/
        assign wb_op_dest = pipeline_reg_in[3:1];
        
        
endmodule 

