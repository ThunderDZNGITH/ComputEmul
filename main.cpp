/*
 * GENERAL REGISTERS :
 * - ax - 16 bits - Accumulator Register	#001
 * 		- ah - 8 bits - High Accumulator	#002
 *		- al - 8 bits - Low Accumulator		#003
 *
 * - bx - 16 bits - Base Register			#004
 * 		- bh - 8 bits - High Base			#005
 *		- bl - 8 bits - Low Base			#006
 *
 * - cx - 16 bits - Count Register			#007
 * 		- ch - 8 bits - High Count			#008
 *		- cl - 8 bits - Low Count			#009
 *
 * - dx - 16 bits - Data Register			#010
 * 		- dh - 8 bits - High Data			#011
 *		- dl - 8 bits - Low Data			#012
 *
 * SEGMENT REGISTERS :
 * - cs - 16 bits - Code Segment			#013
 * - ds - 16 bits - Data Segment			#014
 * - ss - 16 bits - Stack Segment			#015
 * - es - 16 bits - Extra Segment			#016
 *
 * POINTER AND INDICE REGISTERS :
 * - sp - 16 bits - Stack Pointer			#017
 * - bp - 16 bits - Base Pointer			#018
 * - si - 16 bits - Source Index			#019
 * - di - 16 bits - Destination Index		#020
 *
 * FLAGS REGISTER :
 * - flags - 16 bits - Flags				#031
 *
 *
 *
 * OPCODES :
 *
 * - NOP 	- No Operation 			- 0xFF 	- Ex:  	NOP					- CPU does nothing.
 * - ADD 	- Addition 				- 0x01	- Ex:  	ADD ax, bx  		- Add bx to ax 																(ax = ax + bx)
 * - SUB 	- Substraction			- 0x02	- Ex:	SUB ax, bx			- Substract bx from ax 														(ax = ax - bx)
 * - MUL 	- Multiplication		- 0x03	- Ex:	MUL ax, bx			- Multiply bx with ax 														(ax = ax * bx)
 * - DIV 	- Division				- 0x04	- Ex:	DIV ax, bx			- Divide bx by ax 															(ax = ax / bx)
 * - INC 	- Incrementation		- 0x05	- Ex:	INC ax				- Increment ax by 1															(ax = ax + 1)
 * - DEC 	- Decrementation		- 0x06 	- Ex:	DEC ax				- Decrement ax by 1															(ax = ax - 1)
 *
 * - AND 	- Bitwise AND			- 0x07	- Ex: 	AND ax, bx			- Bitwise AND between ax and bx												(ax = ax & bx)
 * - OR	 	- Bitwise OR			- 0x08	- Ex:	OR ax, bx			- Bitwise OR between ax and bx												(ax = ax | bx)
 * - XOR 	- Bitwise XOR			- 0x09	- Ex:	XOR ax, bx			- Bitwise XOR between ax and bx												(ax = ax ^ bx)
 * - NOT 	- Bitwise NOT			- 0x0A	- Ex:	NOT	ax				- Bitwise NOT on ax															(ax = ~ax)
 *
 * - MOV 	- Move					- 0x0B	- Ex:	MOV ax, bx			- Copy the value of bx into ax												(ax = bx)
 * - PUSH 	- Push onto stack		- 0x0C	- Ex:	PUSH ax				- Push the value of ax onto the stack										(Stack = ax, SP = SP - 2)
 * - POP 	- Pop from stack		- 0x0D	- Ex:	POP ax				- Pop the top value from the stack into ax									(ax = Stack, SP = SP + 2)
 * 
 * - JMP 	- Jump unconditionally	- 0x0E	- Ex:	JMP label			- Jump to the specified label 												(unconditional jump)
 * - JE 	- Jump if Equal			- 0x0F	- Ex:	JE label			- Jump to the specified label if the Zero Flag (ZF) is set 					(equal)
 * - JNE 	- Jump if Not Equal		- 0x10	- Ex:	JNE label			- Jump to the specified label if the Zero Flag (ZF) is clear 				(not equal)
 * - JG 	- Jump if Greater		- 0x11	- Ex:	JG label			- Jump to the specified label if the Signed Comparison indicates greater 	(SF == OF and ZF == 0)
 * - JL 	- Jump if Less			- 0x12	- Ex:	JL label			- Jump to the specified label if the Signed Comparison indicates less 		(SF != OF)
 *
 * - CMP 	- Compare				- 0x13	- Ex:	CMP ax, bx			- Subtract bx from ax, set flags, but do not store the result 				(affect Zero, Sign, and Carry flags)
 * - TEST 	- Bitwise test			- 0x14	- Ex:	TEST ax, bx			- Perform bitwise AND between ax and bx, set flags based on the result, but do not store the result
 *
 * - SHL 	- Shift Left			- 0x15	- Ex:	SHL ax, 1			- Shift bits of ax left by 1 position 										(logical shift left, fill with 0 on the right)
 * - SHR 	- Shift Right			- 0x16	- Ex:	SHR ax, 1			- Shift bits of ax right by 1 position 										(logical shift right, fill with 0 on the left)
 * - ROR 	- Rotate Right			- 0x17	- Ex:	ROR ax, 1			- Rotate bits of ax right by 1 position 									(rightmost bit moves to the leftmost position)
 * - ROL 	- Rotate Left			- 0x18	- Ex:	ROL ax, 1			- Rotate bits of ax left by 1 position 										(leftmost bit moves to the rightmost position)
 *
 */

#include <iostream>
#include <iomanip>
#include <bitset>
#include <fstream>

const char CURRENT_VERSION[] = "v1.0.0";

std::uint16_t ProgramCounter = 0;

std::uint16_t REG_zero = 0;

	//	|---------------|
	//	| 	 GENERAL   	|
	//	|	REGISTERS	|	
	//	|---------------|

std::uint16_t REG_ax = 0;
	std::uint8_t REG_ax_ah = 0;
	std::uint8_t REG_ax_al = 0;

std::uint16_t REG_bx = 0;
	std::uint8_t REG_bx_bh = 0;
	std::uint8_t REG_bx_bl = 0;

std::uint16_t REG_cx = 0;
	std::uint8_t REG_cx_ch = 0;
	std::uint8_t REG_cx_cl = 0;

std::uint16_t REG_dx = 0;
	std::uint8_t REG_dx_dh = 0;
	std::uint8_t REG_dx_dl = 0;

	//	|---------------|
	//	| 	 SEGMENT   	|
	//	|	REGISTERS	|	
	//	|---------------|

std::uint16_t REG_cs = 0;

std::uint16_t REG_ds = 0;

std::uint16_t REG_ss = 0;

std::uint16_t REG_es = 0;

	//	|---------------|
	//	| 	 POINTER	|
	//	|	   AND		|
	//	|	 INDICE		|
	//	|	REGISTERS	|	
	//	|---------------|

std::uint16_t REG_sp = 0;

std::uint16_t REG_bp = 0;

std::uint16_t REG_si = 0;

std::uint16_t REG_di = 0;
	
	// 	|---------------|
	//	|	  FLAGS		|
	//	|	REGISTER	|
	//	|---------------|
std::uint16_t REG_flags = 0; 
/* 	b0 = Carry Flag (CF)  
 *	b2 = Parity FLag (PF)
 *	b4 = Auxiliary Carry Flag (AF)  
 *	b6 = Zero Flag (ZF)  
 *	b7 = Sign Flag (SF)  
 *	b8 = Trap Flag (TF) 
 *	b9 = Interrupt Flag (IF)
 *	b10 = Direction Flag (DF)
 *	b11 = Overflow Flag (OF)
 *  b1, b3, b5, b12, b13, b14, b15 = NON USED (0)
 */

enum Register {
	ZERO = 0,
	AX = 0b001,
	BX = 0b010,
	CX = 0b011,
	DX = 0b100,
	CS = 0b101,
	DS = 0b110,
	SS = 0b111
	//ES =
};

	//	|---------------|
	//	|	 OPCODES	|
	//	|---------------|

enum OpCodes {
	NOP 	= 0xFF,
	
	ADD 	= 0x01,
	SUB 	= 0x02,		
	MUL 	= 0x03,
	DIV 	= 0x04,
	INC 	= 0x05,
	DEC 	= 0x06,
	
	AND 	= 0x07,
	OR  	= 0x08,
	XOR 	= 0x09,
	NOT 	= 0x0A,
	
	MOV 	= 0x0B,
	PUSH	= 0x0C,
	POP		= 0x0D,
	
	JMP		= 0x0E,
	JE		= 0x0F,
	JNE		= 0x10,
	JG		= 0x11,
	JL		= 0x12,
	
	CMP		= 0x13,
	TEST	= 0x14,
	
	SHL		= 0x15,
	SHR		= 0x16,
	ROR		= 0x17,
	ROL		= 0x18,
	
	INT		= 0x19
};

template <typename T>
void print_binary(T num) {
    constexpr size_t num_bits = sizeof(T) * 8;
    for (size_t i = num_bits; i > 0; --i) {
        std::cout << ((num >> (i - 1)) & 1);
    }
}

void printRegisters() {
	std::cout << "REGISTERS TRACE :" << std::endl;
	std::cout << std::endl;
	
	//	|---------------|
	//	| 	 GENERAL   	|
	//	|	REGISTERS	|	
	//	|---------------|
	
	std::cout << "	- GENERAL REGISTERS -" << std::endl;
	std::cout << "		- ax = " << std::dec << +REG_ax << " / 0x" << std::hex << +REG_ax << std::dec << " / 0b"; print_binary(REG_ax); std::cout << std::endl;
	std::cout << "			-> ah = "  << std::dec << +REG_ax_ah << " / 0x" << std::hex << +REG_ax_ah << std::dec << " / 0b"; print_binary(REG_ax_ah); std::cout << std::endl;
	std::cout << "			-> al = "  << std::dec << +REG_ax_al << " / 0x" << std::hex << +REG_ax_al << std::dec << " / 0b"; print_binary(REG_ax_al); std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "		- bx = " << +REG_bx << " / 0x" << std::hex << +REG_bx << std::dec << " / 0b"; print_binary(REG_bx); std::cout << std::endl;
	std::cout << "			-> bh = "  << +REG_bx_bh << " / 0x" << std::hex << +REG_bx_bh << std::dec << " / 0b"; print_binary(REG_bx_bh); std::cout << std::endl;
	std::cout << "			-> bl = "  << +REG_bx_bl << " / 0x" << std::hex << +REG_bx_bl << std::dec << " / 0b"; print_binary(REG_bx_bl); std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "		- cx = " << +REG_cx << " / 0x" << std::hex << +REG_cx << std::dec << " / 0b"; print_binary(REG_cx); std::cout << std::endl;
	std::cout << "			-> ch = "  << +REG_cx_ch << " / 0x" << std::hex << +REG_cx_ch << std::dec << " / 0b"; print_binary(REG_cx_ch); std::cout << std::endl;
	std::cout << "			-> cl = "  << +REG_cx_cl << " / 0x" << std::hex << +REG_cx_cl << std::dec << " / 0b"; print_binary(REG_cx_cl); std::cout << std::endl;
	std::cout << std::endl;
	std::cout << "		- dx = " << +REG_dx << " / 0x" << std::hex << +REG_dx << std::dec << " / 0b"; print_binary(REG_dx); std::cout << std::endl;
	std::cout << "			-> dh = "  << +REG_dx_dh << " / 0x" << std::hex << +REG_dx_dh << std::dec << " / 0b"; print_binary(REG_dx_dh); std::cout << std::endl;
	std::cout << "			-> dl = "  << +REG_dx_dl << " / 0x" << std::hex << +REG_dx_dl << std::dec << " / 0b"; print_binary(REG_dx_dl); std::cout << std::endl;
	std::cout << std::endl;
	
	//	|---------------|
	//	| 	 SEGMENT   	|
	//	|	REGISTERS	|	
	//	|---------------|
	std::cout << "	- SEGMENT REGISTERS -" << std::endl;
	std::cout << "		- cs = " << std::dec << +REG_cs << " / 0x" << std::hex << +REG_cs << std::dec << " / 0b"; print_binary(REG_cs); std::cout << std::endl;
	std::cout << "		- ds = " << std::dec << +REG_ds << " / 0x" << std::hex << +REG_ds << std::dec << " / 0b"; print_binary(REG_ds); std::cout << std::endl;
	std::cout << "		- ss = " << std::dec << +REG_ss << " / 0x" << std::hex << +REG_ss << std::dec << " / 0b"; print_binary(REG_ss); std::cout << std::endl;
	std::cout << "		- es = " << std::dec << +REG_es << " / 0x" << std::hex << +REG_es << std::dec << " / 0b"; print_binary(REG_es); std::cout << std::endl;
	std::cout << std::endl;
	
	//	|---------------|
	//	| 	 POINTER	|
	//	|	   AND		|
	//	|	 INDICE		|
	//	|	REGISTERS	|	
	//	|---------------|
	std::cout << "	- POINTER AND INDICE REGISTERS -" << std::endl;
	std::cout << "		- sp = " << std::dec << +REG_sp << " / 0x" << std::hex << +REG_sp << std::dec << " / 0b"; print_binary(REG_sp); std::cout << std::endl;
	std::cout << "		- bp = " << std::dec << +REG_bp << " / 0x" << std::hex << +REG_bp << std::dec << " / 0b"; print_binary(REG_bp); std::cout << std::endl;
	std::cout << "		- si = " << std::dec << +REG_si << " / 0x" << std::hex << +REG_si << std::dec << " / 0b"; print_binary(REG_si); std::cout << std::endl;
	std::cout << "		- di = " << std::dec << +REG_di << " / 0x" << std::hex << +REG_di << std::dec << " / 0b"; print_binary(REG_di); std::cout << std::endl;
	std::cout << std::endl;
	
	// 	|---------------|
	//	|	  FLAGS		|
	//	|	REGISTER	|
	//	|---------------|
	std::cout << "	- FLAGS REGISTER -" << std::endl;
	std::cout << "		- FLAGS = " << std::dec << +REG_flags << " / 0x" << std::hex << +REG_flags << std::dec << " / 0b"; print_binary(REG_flags); std::cout << std::endl;
	std::cout << std::endl;
	
	//	|---------------|
	//	|	 PROGRAM	|
	//	|	 COUNTER	|
	//	|---------------|
	std::cout << "PROGRAM ADRESS : 0x" << std::hex << ProgramCounter << " (" << std::dec << ProgramCounter << ")" <<std::endl;
}

void performCode(char code[]) {
	
}

void update() {
	/*
	 *	Registers Update
	 */
	
	//	|---------------|
	//	| 	 GENERAL   	|
	//	|	REGISTERS	|	
	//	|---------------|
	
	// AX
	REG_ax_ah = (REG_ax >> 8) & 0xFF;
	REG_ax_al = REG_ax & 0xFF;
	//REG_ax = (REG_ax_ah << 8) | REG_ax_al;
	// BX
	REG_bx_bh = (REG_bx >> 8) & 0xFF;
	REG_bx_bl = REG_bx & 0xFF;
	//REG_bx = (REG_bx_bh << 8) | REG_bx_bl;
	// CX
	REG_cx_ch = (REG_cx >> 8) & 0xFF;
	REG_cx_cl = REG_cx & 0xFF;
	//REG_cx = (REG_cx_ch << 8) | REG_cx_cl;
	// DX
	REG_dx_dh = (REG_dx >> 8) & 0xFF;
	REG_dx_dl = REG_dx & 0xFF;
	//REG_dx = (REG_dx_dh << 8) | REG_dx_dl;
	
}

void updateFlags(std::uint16_t result) {
    REG_flags = 0;  // Réinitialiser les flags

    // Zero Flag (ZF) : activé si le résultat est zéro
    if (result == 0) {
        REG_flags |= (1 << 6);  // ZF (Zero Flag) à la position 6
    }

    // Sign Flag (SF) : activé si le bit de signe (bit 15) est défini
    if (result & 0x8000) {
        REG_flags |= (1 << 7);  // SF (Sign Flag) à la position 7
    }

    // Carry Flag (CF) : activé si un dépassement se produit lors d'une addition ou si une soustraction emprunte
    if (result > 0xFFFF) {
        REG_flags |= (1 << 0);  // CF (Carry Flag) à la position 0
    }

    // Parity Flag (PF) : activé si le nombre de bits à 1 dans le résultat est pair
    int parity = 0;
    for (int i = 0; i < 16; i++) {
        if ((result >> i) & 1) {
            parity++;
        }
    }
    if (parity % 2 == 0) {
        REG_flags |= (1 << 2);  // PF (Parity Flag) à la position 2
    }

    // Auxiliary Carry Flag (AF) : activé si un dépassement se produit dans le nibble bas (4 bits)
    if ((result & 0x0F) > 0x0F) {
        REG_flags |= (1 << 4);  // AF (Auxiliary Carry Flag) à la position 4
    }

    // Overflow Flag (OF) : activé si une addition de deux nombres avec le même signe donne un résultat avec un signe différent
    // ou si une soustraction d'un nombre avec des signes différents produit un résultat avec un signe incorrect
    // Cela peut être détecté avec la condition suivante
    if (((result & 0x8000) != (result >> 15) & 0x1) && (result != 0)) {
        REG_flags |= (1 << 11);  // OF (Overflow Flag) à la position 11
    }
}

void setReg(uint16_t reg, uint16_t value) {
	switch(reg) {
		case Register::ZERO:
			break;
		case Register::AX:
			REG_ax = value;
			break;
		case Register::BX:
			REG_bx = value;
			break;
		case Register::CX:
			REG_cx = value;
			break;
		case Register::DX:
			REG_dx = value;
			break;
		default:
			break;
	}
}

uint16_t* getReg(uint8_t reg) {
	switch(reg) {
		case Register::ZERO:
			return &REG_zero;
		case Register::AX:
			return &REG_ax;
		case Register::BX:
			return &REG_bx;
		case Register::CX:
			return &REG_cx;
		case Register::DX:
			return &REG_dx;
		default:
			return nullptr;
			break;
	}
}

void decodeParameters(uint8_t code[2]) {
    uint8_t opcode = code[0];
    uint8_t params = code[1];

    // Extraire le "mod" (2 bits de poids fort)
    uint8_t mod = (params >> 6) & 0x03;  // 0x03 = 00000011 (pour les 2 bits)

    // Extraire le registre source (bits 3 à 5)
    uint8_t srcReg = (params >> 3) & 0x07;  // 0x07 = 00000111 (pour les 3 bits)

    // Extraire le registre destination (bits 0 à 2)
    uint8_t destReg = params & 0x07;
	
	if(opcode == OpCodes::ADD) {
		uint16_t regA = *getReg(srcReg);
		uint16_t regB = *getReg(destReg);
				std::cout << regA << " " << regB << " " << getReg(srcReg) << " " << getReg(destReg) << std::endl;
		setReg(srcReg, regA+regB);
		updateFlags(*getReg(srcReg));
		update();
	}
	
	if(opcode == OpCodes::SUB) {
		uint16_t regA = *getReg(srcReg);
		uint16_t regB = *getReg(destReg);
				std::cout << regA << " " << regB << " " << getReg(srcReg) << " " << getReg(destReg) << std::endl;
		setReg(srcReg, regA-regB);
		updateFlags(*getReg(srcReg));
		update();
	}
	
	if(opcode == OpCodes::MUL) {
		uint16_t regA = *getReg(srcReg);
		uint16_t regB = *getReg(destReg);
				std::cout << regA << " " << regB << " " << getReg(srcReg) << " " << getReg(destReg) << std::endl;
		setReg(srcReg, regA*regB);
		updateFlags(*getReg(srcReg));
		update();
	}
	
	if(opcode == OpCodes::DIV) {
		uint16_t regA = *getReg(srcReg);
		uint16_t regB = *getReg(destReg);
				std::cout << regA << " " << regB << " " << getReg(srcReg) << " " << getReg(destReg) << std::endl;
		setReg(srcReg, regA/regB);
		updateFlags(*getReg(srcReg));
		update();
	}
	
	if(opcode == OpCodes::INC) {
		uint16_t regA = *getReg(srcReg);
				std::cout << regA << " " << getReg(srcReg) << std::endl;
		setReg(srcReg, regA+1);
		updateFlags(*getReg(srcReg));
		update();
	}
	
	if(opcode == OpCodes::DEC) {
		uint16_t regA = *getReg(srcReg);
				std::cout << regA << " " << getReg(srcReg) << std::endl;
		setReg(srcReg, regA-1);
		updateFlags(*getReg(srcReg));
		update();
	}
	
	if(opcode == OpCodes::MOV) {
		uint16_t regA = *getReg(srcReg);
		uint16_t regB = *getReg(destReg);
				std::cout << regA << " " << regB << " " << getReg(srcReg) << " " << getReg(destReg) << std::endl;
		setReg(srcReg, regB);
		updateFlags(*getReg(srcReg));
		update();
	}

	if(opcode == OpCodes::JMP) {
		ProgramCounter = params;
		update();
	}
	
    std::cout << "Opcode: " << std::bitset<8>(opcode) << std::endl;
	std::cout << std::hex << std::bitset<8>(opcode) << std::endl;
    std::cout << "Mod: " << std::bitset<2>(mod) << std::endl;
    std::cout << "Registre source: " << std::bitset<3>(srcReg) << std::endl;
    std::cout << "Registre destination: " << std::bitset<3>(destReg) << std::endl;
}

int main () {
	std::cout << "ComputEmul " << CURRENT_VERSION << std::endl;
	printRegisters();
	/*
	uint8_t code[2] = {0x05, 0b11001000};
	decodeParameters(code);
	printRegisters();
	uint8_t code1[2] = {0x05, 0b11010000};
	decodeParameters(code1);
	printRegisters();
	uint8_t code2[2] = {0x01, 0b11001010};
	decodeParameters(code2);
	printRegisters();
	uint8_t code3[2] = {0x06, 0b11010000};
	decodeParameters(code3);
	printRegisters(); */
	
    std::ifstream inputFile("input.bin", std::ios::binary);
    
    if (!inputFile) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    uint8_t code[2]; // Array to hold the pair of bytes

	inputFile.seekg(0xFE, std::ios::beg);

    if (!inputFile) {
        std::cerr << "Erreur lors du déplacement dans le fichier." << std::endl;
        return 1;
    }

	unsigned char bytes[2];
    inputFile.read(reinterpret_cast<char*>(&bytes), sizeof(bytes));
	unsigned short combined = (bytes[0] << 8) | bytes[1];

	if(combined == 0x55AA) {
		//ProgramCounter = 504;
		inputFile.seekg(0, std::ios::beg);
		// Read the binary file byte by byte
		int nbligne = 0;
		while (!inputFile.eof() && ProgramCounter < 255) {
			nbligne++;
			inputFile.seekg(ProgramCounter, std::ios::beg);
			// Read 2 bytes into the array
			inputFile.read(reinterpret_cast<char*>(code), sizeof(code));

			// Check if we have read exactly 2 bytes
			if (inputFile.gcount() == sizeof(code)) {
				// Decode the parameters with the 2-byte array
				decodeParameters(code);
				// Print the registers (assuming you want this here)
				
				printRegisters();
				ProgramCounter = ProgramCounter+2;
				std::cout << nbligne << std::endl;
			}
		}
	} else {
		std::cout << "No bootsector find" << std::endl;
	}

    // Close the file
    inputFile.close();

    return 0;
}