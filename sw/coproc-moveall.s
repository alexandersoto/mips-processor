.ktext
main:
	addi $7, $zero, 0x1a
	
	# Move 1 into EPC
	mtc0 $7, $14
	
	# Move 1 from EPC
	mfc0 $8, $14
	
	addi $7, $zero, 0x1c

	# Move 1 into EPC
	mtc0 $7, $14
	
	# Move 1 from EPC
	mfc0 $10, $14


	# Done
	.word 0xE
