# ComputEmul v1
## Description
This project was initially made to understand how a x86 CPU work and how it perform to run code.

## Features
- **x86's like instruction set** : 26 Instructions from the x86 instruction set.
- **16-bit real mode emulation** : Only real mode for now.
- **Real registers operation** : Registers act like real ones.

## Try it 

### Prerequisites
- **G++ Compiler** : ```sudo apt-get update && sudo apt-get install build-essential```

### Run it !
First, run ```sh createBin.sh``` , it will create a "input.bin" file where you can write your code.
Then run ```sh build.sh``` , it will compile the emulator and run the code file.


## Instruction set :

 | **Instruction** | **Description**            | **OpCode** | **Exemple**                        | **Effect**                                                                 |
|-----------------|----------------------------|----------|------------------------------------|---------------------------------------------------------------------------|
| NOP             | No Operation               | 0xFF     | NOP                                | CPU does nothing.                                                         |
| ADD             | Addition                   | 0x01     | ADD ax, bx                        | Add bx to ax (ax = ax + bx)                                                |
| SUB             | Substraction               | 0x02     | SUB ax, bx                        | Substract bx from ax (ax = ax - bx)                                      |
| MUL             | Multiplication             | 0x03     | MUL ax, bx                        | Multiply bx with ax (ax = ax * bx)                                        |
| DIV             | Division                   | 0x04     | DIV ax, bx                        | Divide bx by ax (ax = ax / bx)                                            |
| INC             | Incrementation             | 0x05     | INC ax                             | Increment ax by 1 (ax = ax + 1)                                           |
| DEC             | Decrementation             | 0x06     | DEC ax                             | Decrement ax by 1 (ax = ax - 1)                                           |
| AND             | Bitwise AND                | 0x07     | AND ax, bx                        | Bitwise AND between ax and bx (ax = ax & bx)                             |
| OR              | Bitwise OR                 | 0x08     | OR ax, bx                         | Bitwise OR between ax and bx (ax = ax | bx)                               |
| XOR             | Bitwise XOR                | 0x09     | XOR ax, bx                        | Bitwise XOR between ax and bx (ax = ax ^ bx)                             |
| NOT             | Bitwise NOT                | 0x0A     | NOT ax                             | Bitwise NOT on ax (ax = ~ax)                                              |
| MOV             | Move                       | 0x0B     | MOV ax, bx                        | Copy the value of bx into ax (ax = bx)                                   |
| PUSH            | Push onto stack            | 0x0C     | PUSH ax                            | Push the value of ax onto the stack (Stack = ax, SP = SP - 2)             |
| POP             | Pop from stack             | 0x0D     | POP ax                             | Pop the top value from the stack into ax (ax = Stack, SP = SP + 2)        |
| JMP             | Jump unconditionally       | 0x0E     | JMP label                          | Jump to the specified label (unconditional jump)                          |
| JE              | Jump if Equal              | 0x0F     | JE label                          | Jump to the specified label if the Zero Flag (ZF) is set (equal)          |
| JNE             | Jump if Not Equal          | 0x10     | JNE label                         | Jump to the specified label if the Zero Flag (ZF) is clear (not equal)   |
| JG              | Jump if Greater            | 0x11     | JG label                          | Jump to the specified label if the Signed Comparison indicates greater (SF == OF and ZF == 0) |
| JL              | Jump if Less               | 0x12     | JL label                          | Jump to the specified label if the Signed Comparison indicates less (SF != OF) |
| CMP             | Compare                    | 0x13     | CMP ax, bx                        | Subtract bx from ax, set flags, but do not store the result (affect Zero, Sign, and Carry flags) |
| TEST            | Bitwise test               | 0x14     | TEST ax, bx                       | Perform bitwise AND between ax and bx, set flags based on the result, but do not store the result. |
| SHL             | Shift Left                 | 0x15     | SHL ax, 1                         | Shift bits of ax left by 1 position (logical shift left, fill with 0 on the right) |
| SHR             | Shift Right                | 0x16     | SHR ax, 1                         | Shift bits of ax right by 1 position (logical shift right, fill with 0 on the left) |
| ROR             | Rotate Right               | 0x17     | ROR ax, 1                         | Rotate bits of ax right by 1 position (rightmost bit moves to the leftmost position) |
| ROL             | Rotate Left                | 0x18     | ROL ax, 1                         | Rotate bits of ax left by 1 position (leftmost bit moves to the rightmost position) |

 ## Registers :
 GENERAL REGISTERS :
 - ax - 16 bits - Accumulator Register	#001 -> 001
    - ah - 8 bits - High Accumulator	  #002 ->
   	- al - 8 bits - Low Accumulator		  #003 ->
 
 - bx - 16 bits - Base Register			    #004 -> 010
    - bh - 8 bits - High Base			      #005 ->
   	- bl - 8 bits - Low Base			      #006 ->
 
 - cx - 16 bits - Count Register			  #007 -> 011
    - ch - 8 bits - High Count			    #008 ->
    - cl - 8 bits - Low Count			      #009 ->
 
 - dx - 16 bits - Data Register			    #010 -> 100
    - dh - 8 bits - High Data			      #011 ->
    - dl - 8 bits - Low Data			      #012 ->
 
 SEGMENT REGISTERS :
 - cs - 16 bits - Code Segment			    #013 -> 101
 - ds - 16 bits - Data Segment			    #014 -> 110
 - ss - 16 bits - Stack Segment			    #015 -> 111
 - es - 16 bits - Extra Segment			    #016 ->
 
 POINTER AND INDICE REGISTERS :
 - sp - 16 bits - Stack Pointer			    #017 ->
 - bp - 16 bits - Base Pointer			    #018 ->
 - si - 16 bits - Source Index			    #019 ->
 - di - 16 bits - Destination Index		  #020 ->
 
  FLAGS REGISTER :
 - flags - 16 bits - Flags				      #031 ->

   And a ZERO register but it is a special register which store only 0000 0000 (0x00) -> 000

## Address
First, the cpu check the 255(0xFE) and 256(0xFF) byte for 0x55AA (magic boot code) 
Then goes from 0x00 to 0xFD.
CPU read 2 bytes by 2 bytes :
1 byte for OPCODE and 1 byte for PARAM (MOD (2bits) + SRC REG (3 bits) + DEST REG (3 bits)

/!\ For now, MOD isn't managed, so you can write 00, 01, 10 or 11, it doesn't care /!\

/!\ When you jump (JMP), set the target address 2 byte before the wanted address /!\
Ex : 
  If i want to jump to memory address 0xBE, i will do JMP 0xBC which is equal to JMP (0xBE - 0x02)

## Code sample :
```
        00 01
0x00 -> 05 C8  // INC ax      // Increment ax
0x02 -> 05 D0  // INC bx      // Increment bx
0x04 -> 0E 54  // JMP 0x54    // Jump to code address 0x54 (byte 84)

0x56 -> 01 CA  // ADD ax, bx  // Add bx to ax (ax = ax + bx)            (0x01 for ADD, 0xCA for param : 0xCA = 0b11001010 = 11 001 010 = MOD (here 11) + SRC REG (here AX) + DEST REG (here BX))
0x58 -> 0E 89  // JMP 0x89    // Jump to code address 0x89 (byte 137)

0x8B -> 05 C8 // INC ax       // Increment ax
```
At the end :
  - AX = 3
  - BX = 1


## CONTRIBUTE
Feel free to make new fork, pull requests and issues !
I am solo so it's not easy to test all issues myself.

## NEXT
As you can see, the code is not object oriented, all registers aren't implemented, all opcodes aren't implemented and they don't really follow the x86 way to encode them.
In the v2 i plan to redo all this emulator to work better, faster, and with more possibilities...
There is also no RAM, no memory, no I/O, nothing, just a CPU who can execute some basic code.

I also want to precise that i'm not a pro developper, i do that on my free time and with my few knowledge.

For now you can already test some code and do some research.
