### Test normal operation of BLTZ
	.ktext
main:
	addi $10, $zero, 0x1337
	addi $11, $zero, 0x1338
	.word 0xE
	addi $12, $zero, 0x1339
