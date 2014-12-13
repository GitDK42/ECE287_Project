module register(rst, CLK, D, Q, en);
	
	input rst; 
	input CLK; 
	input en;  
	input [15:0] D; 
	output [15:0] Q;
	reg [15:0] Q;
	
	always @(posedge CLK or negedge rst)
		if (rst == 1'b0)
			Q <= 16'd0;
		else
			begin
				if (en)
					Q <= D;
				else	
					Q <= Q;
			end
endmodule

module ALU(rst, CLK, src1, src2, result, operation);
	input CLK, rst; // needed?
	input [2:0] operation;
	input [15:0] src1, src2;
	output [15:0] result;
	reg [15:0] result;
	reg temp;
	always@(posedge CLK or negedge rst) // always@(*)?
	begin
		if (rst == 1'b0)
			result = 16'd0;
		else
		case (operation)
			3'b000: // bitwise AND
				begin
				result <= src1 & src2;
				end
			3'b001: // bitwise OR
				begin
				result <= src2 | src2;
				end
			3'b010: // Add
				begin
				result <= src1 + src2;
				end
			3'b110: // Subtract
				begin
				result <= src1 - src2;
				end
			3'b101: //do nothing
				begin
					temp <= temp;
				end
			default: // Set result to 1;
				begin
				result <= 16'd1;
				end
			endcase
	end
endmodule

//////////////////////////////////////////////////////////////////////////////////////
	// PseudoMemory, more or less mimics RAM
	// Takes 4 Inputs: address, input for writing to, output for reading from, and write/read signals as well as a rst and a CLK
	//
module pseudoMemory(rst, CLK, address, Mem_IN, Mem_OUT, wren);
	input rst, CLK;
	input wren;																								// If HIGH, write value from memory, else read value
	input [15:0] address, Mem_IN;
	output [15:0] Mem_OUT;
	reg [15:0] Mem_OUT;
	reg [15:0] data0, data1, data2, data3, data4, data5;
	
	always@(posedge CLK, negedge rst)
	begin
		if (rst == 1'b0)
		begin
			data0 <= 16'd0;
			data1 <= 16'd0;
			data2 <= 16'd0;
			data3 <= 16'd0;
			data4 <= 16'd0;
			data5 <= 16'd0;
		end
		else if (wren == 1'b0)
		begin
			///////////////////////////////////////////////////////////////////////
			//==========================INSTRUCTIONS=============================//
			///////////////////////////////////////////////////////////////////////
			case (address)
				16'd0: Mem_OUT <= 16'b0000000001001111; 		// set C to 1 (C = 1) 
				16'd4: Mem_OUT <= 16'b0000000011001111;		// set A to 1 (A = 1) 
				16'd8: Mem_OUT <= 16'b0000011110000000;		// set C to C+A (C = 2) 
				16'd12: Mem_OUT <= 16'b0000110110000010; 		// set B to C+A (B = 3) 
				16'd16: Mem_OUT <= 16'b0000000010001111; 		// set B to 1 (B = 1) 
				16'd20: Mem_OUT <= 16'b0000000001001111; 		// set A to 1 (A = 1) 
				16'd24: Mem_OUT <= 16'b0000000010001111;	   // set B to 1 (B = 1) 
				16'd28: Mem_OUT <= 16'b0000011111000000; 		// set C to A+C (C = 2)
				16'd32: Mem_OUT <= 16'b0000011111000000; 		// set C to A+C (C = 3)
		
			///////////////////////////////////////////////////////////////////////
			//==============================DATA=================================//
			///////////////////////////////////////////////////////////////////////
				
				16'd36: Mem_OUT <= data0;					 		// Load data0 to a register 
				16'd40: Mem_OUT <= data1;							// Load data1 to a register 
				16'd44: Mem_OUT <= data2;							// Load data2 to a register 
				16'd48: Mem_OUT <= data3;					 		// Load data3 to a register 
				16'd52: Mem_OUT <= data4;					 		// Load data4 to a register 
				16'd56: Mem_OUT <= data5;					 		// Load data5 to a register 
				16'd60: Mem_OUT <= 16'b0000000000001000;	   // Load 8 (constant) to a register
				16'd64: Mem_OUT <= 16'b0000000000001111; 		// Load 15 (constant) to a register
				default: Mem_OUT <= 16'b0000000000000000;		// Load 0 (constant) to a register or nop
			endcase
		end
		else 
		begin
			case (address)												// Only data are valid options to write to. So sayeth me.
			
			///////////////////////////////////////////////////////////////////////
			//==============================DATA=================================//
			///////////////////////////////////////////////////////////////////////
				
				16'd36: data0 <= Mem_IN;					 		// Write Mem_IN to data0
				16'd40: data1 <= Mem_IN;							// Write Mem_IN to data1
				16'd44: data2 <= Mem_IN;							// Write Mem_IN to data2
				16'd48: data3 <= Mem_IN;					 		// Write Mem_IN to data3
				16'd52: data4 <= Mem_IN;					 		// Write Mem_IN to data4
				16'd56: data5 <= Mem_IN;					 		// Write Mem_IN to data5
				// 16'd60 and 16'd64 left out due to being constants and unable to overwrite	
				default: Mem_OUT <= 16'b0000000000000000;		// Invalid address, do nothing.
			endcase
			Mem_OUT <= 16'd0;
		end
	end
endmodule

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//================================================================================================================================//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

module Processor(rst, CLK, regDisplay, proceed, LEDG, out);
//////////////////////////////////////////////////////////////////////////////////////
	// Inputs
	input rst, CLK;										// rst = KEY0, CLK = CLOCK_50;
	input [2:0] regDisplay;								// regDisplay = [17:15]SW
	input [2:0] proceed;									// person = SW[x:0];
	
//////////////////////////////////////////////////////////////////////////////////////
	// Outputs
	output [15:0] out;									// out = [15:0]LEDR
	output [3:0] LEDG;									// display for current stage
	
//////////////////////////////////////////////////////////////////////////////////////
	// Regs
	reg [3:0] S, NS;
	reg [4:0] cycles;										// counter to act as secondary clock for buffer cycles 
	reg [15:0] PC_reg;
	wire [15:0] PC;

//////////////////////////////////////////////////////////////////////////////////////
	// Control Signals
	reg RegDst;
	reg ALUSrc;
	reg MemtoReg;
	reg RegWrite;
	reg Branch;
	// MemRead is HIGH when enMDR_denMem is HIGH (MDR takes output from Memory)
	// MemWrite is HIGH when enMDR_denMem is LOW (Memory takes output from MDR)
					
	// Select destination register
	reg [1:0] dst;
	
	// Select output
	reg [15:0] out_reg;
	
	// Location
	reg [3:0] LEDG;
	
	// break down of each instruction
	reg [3:0] OpCode;										// instruction[15:12]
	reg [1:0] rs, rt, rd;								// instruction[11:10],[9:8],[7:6]
	reg [1:0] shamt;										// instruction[5:4]
	reg [3:0] funct;										// instruction[3:0]
	reg [7:0] address;									// instruction[7:0]
	
//////////////////////////////////////////////////////////////////////////////////////
	// Temporary Registers
	// Temporary Registers Regs
	reg [15:0] in_reg;									// has accompanying wire
	reg writeA_reg;										// has accompanying wire
	reg writeB_reg;										// has accompanying wire
	reg writeC_reg;										// has accompanying wire
	
	// Temporary Register Wires
	wire [15:0] in;										// has accompanying reg
	wire [15:0] outA;
	wire [15:0] outB;
	wire [15:0] outC;
	wire writeA;											// has accompanying reg
	wire writeB;											// has accompanying reg
	wire writeC;											// has accompanying reg
	
//////////////////////////////////////////////////////////////////////////////////////
	// Special Registers
	// Special Registers Regs
	// IR
	reg enIR_reg;											// has accompanying wire
	// ALUOut
	reg enALU_reg;											// has accompanying wire
	
	// Special Registers Wires 
	// IR
	// input = MDR_OUT
	wire [15:0] instruction;
	wire enIR;												// has accompanying reg
	// ALUOut
	// input = (ALU) result
	wire [15:0] ALUOut;
	wire enALU;												// has accompanying reg
	
//////////////////////////////////////////////////////////////////////////////////////
	// ALU
	// ALU Regs
	reg [15:0] src1_reg;
	reg [15:0] src2_reg;
	reg [2:0] operation_reg;
	// ALU Wires
	wire [15:0] src1;
	wire [15:0] src2;
	wire [15:0] result; 
	wire [2:0] operation;
//////////////////////////////////////////////////////////////////////////////////////
	// Memory Registers
	// Memory Registers Regs
	//MAR
	reg [15:0] MAR_IN_reg;								// has accompanying wire
	reg enMAR_reg	;										// has accompanying wire
	// MDR
	reg [15:0] MDR_IN_reg;								// has accompanying wire
	reg enMDR_reg;											// has accompanying wire
	
	// Memory Registers Wires
	// MAR
	wire [15:0] MAR_IN;									// has accompanying reg
	wire [15:0] MAR_OUT;									// goes to Memory address input
	wire enMAR;												// has accompanying reg
	// MDR
	wire [15:0] MDR_IN;									// has accompanying reg
	wire [15:0] MDR_OUT;									// goes to IR_input and Data for MemtoReg
	wire enMDR;												// has accompanying reg
	
//////////////////////////////////////////////////////////////////////////////////////
	// Memory
	// Memory Regs
	reg wren_reg;	
	// Memory Wires
//wire [15:0] mem_address;// address input from MAR
	// data input from MDR
	// output goes to MDR											
	wire wren;												
	
//////////////////////////////////////////////////////////////////////////////////////
	// Constants
	//defparam r_type = 4'd0;
	//defparam load 	= 4'd8;
	//defparam store 	= 4'd12;
	//defparam branch = 4'd4;
	//defparam nop 	= 4'd15;
	
//////////////////////////////////////////////////////////////////////////////////////
	// Parameters
	parameter IF      	= 	  4'd0;
	parameter ID      	= 	  4'd1;
	parameter R_TYPE  	=    4'd2;
	parameter LOAD    	= 	  4'd3;
	parameter STORE   	= 	  4'd4;
	parameter BRANCH 	 	= 	  4'd5;
	parameter EX      	=	  4'd6;
	parameter MA      	=    4'd7;
	parameter WB     		=	  4'd8;
	parameter RST_CYCLES =    4'd9;
	parameter DANGER  	=    4'd10;

//////////////////////////////////////////////////////////////////////////////////////
	// 1.) Define next state conditions
	always@(*)
	begin
		case (S)
			IF:																								// Let run for 6 cycles (3 + 3 buffer cycles) 5:0
			begin
				if (cycles == 5'd6 && proceed == 3'b000)
					NS = ID;
				else
					NS = IF;
			end
			ID:																								// Let run for 2 cycles (1+ 1 buffer cycle) 7:6
			begin
				if (cycles == 5'd8 && proceed == 3'b001)
				begin
					case (OpCode)
						4'd0: 	NS = R_TYPE;
						4'd8:    NS = LOAD;
						4'd12:   NS = STORE;
						4'd4: 	NS = BRANCH;
						// TODO: add nop?
						default: NS = DANGER;
					endcase
				end
				else
				NS = ID;
			end
			R_TYPE:
			begin
				if (cycles == 5'd10 && proceed == 3'b010)											// Let run for 2 cycles (1 + 1 buffer cycle) 9:8
					NS = EX;
				else
					NS = R_TYPE;
			end
			LOAD:
			begin
				if (cycles == 5'd10 && proceed == 3'b010)											// Let run for 2 cycles (1 + 1 buffer cycle) 9:8
					NS = EX; 
				else
					NS = LOAD;
			end
			STORE:
			begin
				if (cycles == 5'd10 && proceed == 3'b010)											// Let run for 2 cycles (1 + 1 buffer cycle) 9:8
					NS = EX; 
				else
					NS = STORE;
			end
			BRANCH:
			begin
				if (cycles == 5'd10 && proceed == 3'b010)											// Let run for 2 cycles (1 + 1 buffer cycle) 9:8
					NS = EX;
				else
					NS = BRANCH;
			end
			EX:
			begin
				if (cycles == 5'd14 && proceed == 3'b011)											// Let run for 4 cycles (2 + 2 buffer cycle) 13:10
					NS = MA;
				else
					NS = EX;
						
			end
			MA:
			begin
				if (cycles == 5'd16 && proceed == 3'b100)											// Let run for 2 cycles (1 + 1 buffer cycle) 15:14
					NS = WB;
				else
					NS = MA;
			end
			WB:
			begin
				if (cycles == 5'd19 && proceed == 3'b101)											// Let run for 2 cycles (1 + 1 buffer cycle) 17:16
					NS = RST_CYCLES;
				else
					NS = WB;
			end
			RST_CYCLES:
			begin
				if (cycles == 5'd0 && proceed == 3'b110)											// Let run for 2 cycles (1 + 1 buffer cycle) 19:18
					NS = IF;
				else
					NS = RST_CYCLES;
			end
			DANGER:																							// Stuck here forever and ever! Until you remember there is a rst button...
			begin
				NS = DANGER;
			end
			default: NS = DANGER;
		endcase
	end
	
//////////////////////////////////////////////////////////////////////////////////////
	// 2.) S to NS transition 
	always@(posedge CLK or negedge rst)
	begin
		if (rst == 1'b0)
			S <= IF;
		else
			S <= NS;
	end

//////////////////////////////////////////////////////////////////////////////////////
	// 3.) Output generation
	always@(posedge CLK or negedge rst)
	begin
		if (rst == 1'b0)
		begin
			cycles <= 5'd0;
			PC_reg <= 16'd0;
		end
		else
		begin
			case (S)
				
				// Instruction Fetch: 
				// Spend 6 cycles: 5:0
				// 1st cycle: MAR(PC)
				// 2nd cycle: Mem(MAR, MDR)and MDR receives instruction from Mem
				// 3rd cycle: IR(MDR) (enIR HIGH)
				IF:
				begin
					LEDG = 4'b0000;
					
					if (cycles < 5'd5)	
						begin
						enMAR_reg 			<= 1'b1; 													// Enable MAR
						MAR_IN_reg 			<= PC;														// Set MAR_IN to PC
						enMDR_reg 			<= 1'b1;														// Enable MDR to take Memory_Out (Set Memory to read)
						enIR_reg				<= 1'b1;														// Enable IR to accept MDR value
						wren_reg				<= 1'b0;
						cycles 				<= cycles + 5'd1;											// Update buffer clock
						end
					else if (cycles < 5'd6)																	
						begin
						enMAR_reg 			<= 1'b0; 													// Disable MAR
						enMDR_reg		 	<= 1'b0;														// Disable MDR to take Memory_Out (Set Memory to write...)
						enIR_reg 			<= 1'b0;														// Disable IR to accept MDR value
						PC_reg 				<= PC + 16'd4;												// Increment PC
						cycles 				<= cycles + 5'd1;											// Update buffer clock
						end
					else 
							cycles <= cycles;																// Buffer clock holds value until next stage.					
				end
				
				// Instruction Decode: 
				// Spend 2 cycles: 7:6
				// get OpCode, rs, rt, rd, shamt, funct, and address
				// case statement for instruction output from ID, set appropriate signals 
				ID:
				begin					
					LEDG = 4'b0001;
					
					enIR_reg <= 1'b0;																		// Disable IR
					OpCode  <= instruction[15:12];
					rs 	  <= instruction[11:10];
					rt 	  <= instruction[9:8];
					rd 	  <= instruction[7:6];
					shamt   <= instruction[5:4];
					funct   <= instruction[3:0];
					address <= instruction[7:0];
					// case statement handled in 1.) NS assignments 					
					
					if (cycles < 5'd8)
						cycles <= cycles + 5'd1;														// track number of clock cycles
					else cycles <= cycles;
				end
				
				// R_TYPE Instruction (Substage of ID): 
				// Spend 2 cycles: 9:8
				// set src registers
				// determine ALU operation from funct			
				R_TYPE:
				begin
					LEDG = 4'b1000;
					// Determine src1
					case (rs)
						2'b00:																				// src1 = Zero Register
						begin
							src1_reg <= 16'd0;
						end
						2'b01:																				// src1 = Register A
						begin	
							src1_reg <= outA;
						end
						2'b10:																				// src1 = Register B
						begin	
							src1_reg <= outB;
						end
						2'b11:																				// src1 = Register C
						begin
							src1_reg <= outC;
						end
					endcase
					
					// Determine src2
					case (rt)
						2'b00:																				// src2 = Zero Register
						begin
							src2_reg <= 16'd0;
						end
						2'b01:																				// src2 = Register A
						begin
							src2_reg <= outA;
						end
						2'b10:																				// src2 = Register B
						begin
							src2_reg <= outB;
						end
						2'b11:																				// src2 = Register C
						begin
							src2_reg <= outC;
						end
					endcase
					
					// Destination Register determined in WB (Appropriate write signal enabled)
					
					// Set control signals
					RegDst				<= 1'b1;
					ALUSrc				<= 1'b0;
					MemtoReg 			<= 1'b0;
					RegWrite 			<= 1'b1;
					wren_reg   			<= 1'b0;
					Branch				<= 1'b0;
					
					// Determine ALU operation
					case (funct)
						4'b0000:
							begin
							operation_reg <= 3'b010; 													// Add
							end
						4'b0010:
							begin
							operation_reg <= 3'b110; 													// Subtract
							end
						4'b0100:
							begin
							operation_reg <= 3'b000; 													// Bitwise AND
							end
						4'b0101:
							begin
							operation_reg <= 3'b001; 													// Bitwise OR
							end
						4'b1010:
							begin
							operation_reg <= 3'b111; 													// Set on <
							end
						default:
							begin
							operation_reg <= 3'b011; 													// Set on 1 (anything better?)
							end
					endcase
					
					if (cycles < 5'd10)
						cycles <= cycles + 5'd1;						   							// track number of clock cycles
					else cycles <= cycles;
				end
				
				// LOAD Instruction (Substage of ID):
				// Spend 2 cycles: 9:8
				// set src register 
				// set ALU operation (add)
				LOAD:
				begin
					LEDG = 4'b1001;
					// Determine src1 (should usually be zero register)
					case (rs)
						2'b00:																				// src1 = Zero Register
						begin
							src1_reg <= 16'd0;
						end
						2'b01:																				// src1 = Register A
						begin
							src1_reg <= outA;
						end
						2'b10:																				// src1 = Register B
						begin
							src1_reg <= outB;
						end
						2'b11:																				// src1 = Register C
						begin
							src1_reg <= outC;
						end
					endcase
					
					// Set control signals
					RegDst		<= 1'b0;
					ALUSrc		<= 1'b1;
					MemtoReg 	<= 1'b1;
					RegWrite 	<= 1'b1;
					wren_reg		<= 1'b0;																	// read/load from memory
					Branch		<= 1'b0;
					operation_reg <= 3'b010;															// set ALU operation to add
					
					if (cycles < 5'd10)
						cycles <= cycles + 5'd1;														// track number of clock cycles
					else cycles <= cycles;
				end
				
				// STORE Instruction (Substage of ID): 
				// Spend 2 cycles: 9:8
				// set src register (should be zero?)
				// set ALU operation (add)
				STORE:
				begin
					LEDG = 4'b1010;
					// Determine src1 (should always be zero register)
					case (rs)
						2'b00:																				// src1 = Zero Register
						begin
							src1_reg <= 16'd0;
						end
						2'b01:																				// src1 = Register A
						begin
							src1_reg <= outA;
						end
						2'b10:																				// src1 = Register B
						begin
							src1_reg <= outB;
						end
						2'b11:																				// src1 = Register C
						begin
							src1_reg <= outC;
						end
					endcase
					
					// Set control signals
					RegDst	<= 1'b0;
					ALUSrc	<= 1'b1;
					MemtoReg <= 1'b0;
					RegWrite <= 1'b0;
					wren_reg	<= 1'b1;																		// Write/store to memory
					Branch	<= 1'b0;	
					operation_reg <= 3'b010;															// set ALU operation to add
					
					if (cycles < 5'd10)
						cycles <= cycles + 5'd1;														// track number of clock cycles
					else cycles <= cycles;
				end
				
				// TODO: BRANCH Instruction (Substage of ID): 
				// Spend 2 cycles: 9:8
				// set src register
				// set ALU operation (subtract)
				BRANCH:
				begin
					LEDG = 4'b1100;
					// Set control signals
					RegDst	<= 1'b0;
					ALUSrc	<= 1'b0;
					MemtoReg <= 1'b0;
					RegWrite <= 1'b0;
					wren_reg	<= 1'b0;																		// read/load from memory
					Branch	<= 1'b1;
					operation_reg <= 3'b110;															// set ALU operation to subtract
					
					if (cycles < 5'd10)
						cycles <= cycles + 5'd1;														// track number of clock cycles
					else cycles <= cycles;
				end
				
				// Execute: 
				// Spend 4 cycles: 13:10
				// 1st cycle: feed ALU
				// 2nd cycle: store ALU result in ALUOut
				EX:
				begin
					LEDG = 4'b0011;
					
					if (cycles == 5'd10 || cycles == 5'd11)
					begin				
							src2_reg <= src2_reg; 														// R-Type or Branch instruction
					end
					else if (cycles == 5'd12 | cycles == 5'd13)
					begin
						enALU_reg <= 1'b1;																// enables ALU Register
					end
					else 
					begin
						enALU_reg <= 1'b0;																// disable ALU Register
					end
					
					if (cycles < 5'd14)
						cycles <= cycles + 5'd1;														// track number of clock cycles
					else cycles <= cycles;
				end
				
				// Memory Access: 
				// Spend 2 cycles: 15:14
				MA:
				begin
					enALU_reg <= 1'b0;																	// disable ALU Register
					LEDG = 4'b0100;
					case (OpCode)
						4'd0:																					// R-Type (No memory access affects registers only)
						begin
							if (cycles < 5'd16)
								cycles <= cycles + 5'd1; 												// proceed
							else cycles <= cycles;
						end
							
						4'd8:    																			// load
						begin
							cycles <= cycles;	
						end
						
						4'd12:   																			// store
						begin
							cycles <= cycles;						
						end
						
						4'd4:																					// branch  (no memory access, affects PC only)
							begin
							if (cycles <= 5'd16)
								cycles <= cycles + 5'd1;												// proceed
							else cycles <= cycles;
							end
							
						default: 
							begin
							if (cycles < 5'd16)
								cycles <= cycles + 5'd1;												// proceed
							else cycles <= cycles;
							end
					endcase	
				end
				
				// Write Back: 
				// Spend 3 cycles: 18:16
				// case for OpCode
				// R-Type: MemtoReg LOW, ALUOut writes dst register
				// load: MemtoReg HIGH, MDR writes dst register
				// store: MDR writes DATA to address designated by MAR
				WB: //***************THIS CURRENTLY ONLY WORKS FOR R-TYPE***************//
				begin
					LEDG = 4'b0101;
					if(cycles == 5'd16)
					begin
						enALU_reg <= 1'b0;																// disable ALU Register
					end
					else
					begin
					// determine register input
					in_reg <= ALUOut; //MemtoReg ? MDR_OUT : ALUOut;
					dst <= RegDst ? rd : rt;
					// Determine register destination by enabling appropriate write signal

					
					case (dst)
					2'b00:																					// if zero register set all writes to 0 
						begin	
						writeA_reg <= 1'b0;
						writeB_reg <= 1'b0;
						writeC_reg <= 1'b0;
						end
					2'b01:																					// if Register A set A to write.
						begin	
						writeA_reg <= RegWrite;
						writeB_reg <= 1'b0;
						writeC_reg <= 1'b0;
						end
					2'b10:																					// if Register B set B to write.
						begin		
						writeA_reg <= 1'b0;
						writeB_reg <= RegWrite;
						writeC_reg <= 1'b0;
						end
					2'b11:																					// if Register C set C to write.
						begin
						writeA_reg <= 1'b0;
						writeB_reg <= 1'b0;
						writeC_reg <= RegWrite;
						end
					endcase
				end
					if (cycles < 5'd19)
						cycles <= cycles + 5'd1;														// track number of cycles
					else cycles <= cycles;
				end
				
				// Reset Cycles:
				// Spend 2 cycles: 20:19
				// Resets the count on the cycles and ensures all register enable signals are set to LOW
				RST_CYCLES:
				begin
					LEDG = 4'b0110;					
					
					// disable all writes
					writeA_reg	  			<= 1'b0;
					writeB_reg	  			<= 1'b0;
					writeC_reg	  			<= 1'b0;
					enIR_reg 	  			<= 1'b0;
					enALU_reg     			<= 1'b0;
					enMAR_reg	   		<= 1'b0;
					enMDR_reg	   		<= 1'b0;
					in_reg					<= 1'b0;
					// double set 
					if (cycles < 5'd20)
						cycles <= cycles + 5'd1;
					else
						cycles <= 5'd0;																	// reset cycle count
				end
				DANGER: LEDG <= 4'b1111;
			endcase
			// Generate output
			case (regDisplay)
			3'b000: out_reg <= instruction;
			3'b001: out_reg <= ALUOut;
			3'b010: out_reg <= outA;
			3'b011: out_reg <= outB;
			3'b100: out_reg <= outC;
			3'b101: out_reg <= writeA;
			3'b110: out_reg <= writeB;
			3'b111: out_reg <= writeC; 						
			endcase
		end
	end
	
/////////////////////////////////////////////////////////////////////////////////////
	// wire assigments
	
	// module instantiation inputs
	// Temporary Registers
	assign in = in_reg;

	// ALU
	assign src1 = src1_reg;
	assign src2 = src2_reg;
	// PC
	assign PC = PC_reg;
	// Memory
	assign MAR_IN = MAR_IN_reg;
	//assign MDR_IN = MDR_IN_reg;
	
	// module instantiation signals
	assign writeA = writeA_reg;
	assign writeB = writeB_reg;
	assign writeC = writeC_reg;
	assign enIR = enIR_reg;
	assign enALU = enALU_reg;
	assign enMAR = enMAR_reg;
	assign enMDR = enMDR_reg;
	assign wren = wren_reg;
	assign operation = operation_reg;


/////////////////////////////////////////////////////////////////////////////////
	// module instantiations
	// Register for dedicated zero output
	//can dropregister inst_zero(rst, CLK, in, zero, writeZero);
	
	// Registers for temporary storage
	register inst_A(rst, CLK, in, outA, writeA);
	register inst_B(rst, CLK, in, outB, writeB);
	register inst_C(rst, CLK, in, outC, writeC);
	
	// Register for current instruction
	register inst_IR(rst, CLK, MDR_OUT, instruction, enIR);
	
	// Register for dedicated ALU output
	register inst_ALUOut(rst, CLK, result, ALUOut, enALU);
	
	// Registers for memory access
	register inst_MAR(rst, CLK, MAR_IN, MAR_OUT, enMAR); 					
	register isnt_MDR(rst, CLK, MDR_IN, MDR_OUT, enMDR);					 
	
	// ALU 
	ALU inst_ALU(rst, CLK, src1, src2, result, operation);
	
	// PseudoMemory (later RAM)
	pseudoMemory inst_Mem(rst, CLK, MAR_OUT, MDR_OUT, MDR_IN, wren);

	// display output
	assign out = out_reg;
endmodule
