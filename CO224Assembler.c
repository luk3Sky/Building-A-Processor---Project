/*
Program	: CO224 Assembler
Author	: Isuru Nawinne
Version	: 1.1
Date		: 06/12/2018


Description:

This program can be used to to convert hand-written assembly codes into machine code in CO224 laboratory exercises.
This simple assembler assumes an ISA containing the following instructions: loadi, mov, add, sub, and, or, load, store. 
All instructions are encoded into 32-bits assuming the following format:

Bits 31-24 : OP-CODE		: Given as one of (loadi, mov, add, sub, and, or, load, store)
Bits 16-23 : Destination Operand: Given as a register number (0-7), or an 8-bit memory address as an immediate value in hex (e.g. 0xFF)
Bits 08-15 : Source Operand 2	: Given as a register number (0-7), or ignored if an 'X' is given
Bits 00-07 : Source Operand 1	: Given as a register number (0-7), an 8-bit signed integer given as an immediate value in hex (e.g. 0xFF), or an 8-bit memory address as an immediate value in hex (e.g. 0x1A)

This assembler will perform basic error checks on your program. A valid instruction should contain four tokens separated by space character (e.g. load 5 X 0x1A), corresponding to the details given above. In addition, empty lines and valid comments are permitted. A valid comment should start with "//".

NOTE	: You must define the op-codes assinged to instructions, to match the definitions in your processor architecture. 
	  Edit the relevant section below.

Compiling the program	: gcc CO224Assembler.c -o CO224Assembler
Using the assembler	: ./CO224Assembler <your_assembly_file_name> (e.g. ./CO224Assembler program.s)
Generated output file	: <your_assembly_file_name>.machine
*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>


#define LINE_SIZE 512


int main( int argc, char *argv[] )
{

	/* OP-CODE DEIFINITIONS
	CHANGE THESE according to op-codes assigned in your processor architecture 
	*************************************************************************/
	char *op_loadi 	= "00000000";
	char *op_mov 	= "00000001";
	char *op_add 	= "00000010";
	char *op_sub 	= "00000011";
	char *op_and 	= "00000100";
	char *op_or 	= "00000101";
	char *op_load 	= "00000110";
	char *op_store 	= "00000111";
	/************************************************************************/
	
	const char delim[] = " "; // OP-CODE and Operands should be provided separated by spaces
	FILE *fi, *fo;
	char line[LINE_SIZE];
	long line_count = 0;
	char *in_token;
	char out_token[] = "00000000";
	char out_file[256];

	strcpy(out_file,argv[1]);
	strcat(out_file,".machine");

 	if ((fi = fopen(argv[1],"r")) == NULL){
		printf("Err0r: Cannot open source file!\n");
		exit(1);
	}

	if ((fo = fopen(out_file,"wb")) == NULL){
		printf("Error: Cannot open output file!\n");
		fclose(fi);
		exit(1);
	}

 	while(fgets(line, LINE_SIZE, fi)!=NULL)
	{
		in_token = strtok(line, delim);
		line_count++;
		int count = 0;
		while(in_token!=NULL)
		{
			count++;

			// Encoding the op-code
			if(strcasecmp(in_token,"loadi")==0) strcpy(out_token, op_loadi);
			else if(strcasecmp(in_token,"mov")==0) strcpy(out_token, op_mov);
			else if(strcasecmp(in_token,"add")==0) strcpy(out_token, op_add);
			else if(strcasecmp(in_token,"sub")==0) strcpy(out_token, op_sub);
			else if(strcasecmp(in_token,"and")==0) strcpy(out_token, op_and);
			else if(strcasecmp(in_token,"or")==0) strcpy(out_token, op_or);
			else if(strcasecmp(in_token,"load")==0) strcpy(out_token, op_load);
			else if(strcasecmp(in_token,"store")==0) strcpy(out_token, op_store);

			// Encoding register numbers
			else if(strcmp(in_token,"0")==0 || strcmp(in_token,"0\n")==0) strcpy(out_token, "00000000");
			else if(strcmp(in_token,"1")==0 || strcmp(in_token,"1\n")==0) strcpy(out_token, "00000001");
			else if(strcmp(in_token,"2")==0 || strcmp(in_token,"2\n")==0) strcpy(out_token, "00000010");
			else if(strcmp(in_token,"3")==0 || strcmp(in_token,"3\n")==0) strcpy(out_token, "00000011");
			else if(strcmp(in_token,"4")==0 || strcmp(in_token,"4\n")==0) strcpy(out_token, "00000100");
			else if(strcmp(in_token,"5")==0 || strcmp(in_token,"5\n")==0) strcpy(out_token, "00000101");
			else if(strcmp(in_token,"6")==0 || strcmp(in_token,"6\n")==0) strcpy(out_token, "00000110");
			else if(strcmp(in_token,"7")==0 || strcmp(in_token,"7\n")==0) strcpy(out_token, "00000111");

			// Encoding ignored operands
			else if(strcasecmp(in_token,"X")==0) strcpy(out_token, "00000000");

			// Encoding immediate values (must be provided in hex format)
			// If you want to be able to use decimal numbers as immediate values, you must convert them into signed 2's complement format as appropriate !
			else if(strstr(in_token,"0x") && (strstr(in_token,"0x") == in_token))
			{
				int i;
				for(i=0;i<2;i++)
				{
					if(toupper(in_token[2+i])=='0') strcpy(out_token+(4*i), "0000");
					if(toupper(in_token[2+i])=='1') strcpy(out_token+(4*i), "0001");
					if(toupper(in_token[2+i])=='2') strcpy(out_token+(4*i), "0010");
					if(toupper(in_token[2+i])=='3') strcpy(out_token+(4*i), "0011");
					if(toupper(in_token[2+i])=='4') strcpy(out_token+(4*i), "0100");
					if(toupper(in_token[2+i])=='5') strcpy(out_token+(4*i), "0101");
					if(toupper(in_token[2+i])=='6') strcpy(out_token+(4*i), "0110");
					if(toupper(in_token[2+i])=='7') strcpy(out_token+(4*i), "0111");
					if(toupper(in_token[2+i])=='8') strcpy(out_token+(4*i), "1000");
					if(toupper(in_token[2+i])=='9') strcpy(out_token+(4*i), "1001");
					if(toupper(in_token[2+i])=='A') strcpy(out_token+(4*i), "1010");
					if(toupper(in_token[2+i])=='B') strcpy(out_token+(4*i), "1011");
					if(toupper(in_token[2+i])=='C') strcpy(out_token+(4*i), "1100");
					if(toupper(in_token[2+i])=='D') strcpy(out_token+(4*i), "1101");
					if(toupper(in_token[2+i])=='E') strcpy(out_token+(4*i), "1110");
					if(toupper(in_token[2+i])=='F') strcpy(out_token+(4*i), "1111");
				}
			}
		
			// Handling comments and empty lines
			else if(strcmp(in_token,"\n")==0||(strstr(in_token,"//") && (strstr(in_token,"//") == in_token))){
				count--;
				break;
			}
			// Handling lines/words which are not part of an instruction
			else
			{
				count = 99;
				break;//strcpy(out_token, "");
			}

			fputs(out_token, fo);			
			in_token = strtok(NULL, delim);
		}
		
		if(count==4) // Line contains a valid instruction
			fputs("\n", fo);
		else if(count!=0) // Line is neither a valid instruction, nor a valid comment / empty line
		{
			printf("Error: Incorrect instruction format! (line: %li)\n",line_count);
			fclose(fi);
			fclose(fo);
			exit(1);
		}

	}

	fclose(fi); 
	fclose(fo); 
  
	return 0;
}
