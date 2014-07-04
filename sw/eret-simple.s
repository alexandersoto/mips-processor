.ktext
	addi $v0, $zero, 0xa
	addi $10, $zero, 0xDEAD
	addi $11, $zero, 0xC0DE
	lui $12, 0x00400000
	mtc0 $12, $14
	.word 0x42000018
	
.text  
	addi $6, $zero, 0xDEAD
	addi $5, $zero, 0x6969
	ori $4, $zero, 0x4018
	lui  $13, 0x80000000
	addi  $13, $13, 0x18
	.word	0xE	
