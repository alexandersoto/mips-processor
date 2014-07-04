### Test if the index coprocessor register is working as expected
	.ktext
main:
	li $10, 0x13371337
	li $11, 0x13381338
	mtc0 $10, $2
	mtc0 $11, $2
	mtc0 $10, $2
	mfc0 $12, $2
	addi  $13, $12, 0x2 
	mtc0 $13, $2
	mfc0 $14, $2
	addi  $13, $12, 0x2 
	.word 0xE
	addi $12, $zero, 0xDEAD
