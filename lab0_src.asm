# (s0-s11): local variables must add to stack
# (t0-t6): global
.data
A:      .word	0,3,2,0,3,1,0,3,2
B:      .word	1,1,0,3,1,2,0,0,0
C:      .word 	0,0,0,0,0,0,0,0,0

.text
main:
	li	a0,3
	addi    sp, sp, -4   # saves ra to stack
	sw      ra, 0(sp)
	
	call 	matrixMult
	
	lw      ra, 0(sp)
	addi    sp, sp, 4   # load ra from stack
	
	j done1
					


#----------------------------------------------------------------------------------------
# Matrix multiplication function
# a0 = array size
# more testing
#========================================================================================
matrixMult:
init:	
	addi	sp,sp,-24
	sw	s0,0(sp)
	sw	s1,4(sp)
	sw	s2,8(sp)
	sw	s3,12(sp)
	sw	s4,16(sp)
	sw	s5,20(sp)
					
init6:		
	mv      a2, a0       # moves the size			
	la	s1,A	#load array A
	la	s2,B	#load array B
	la	s6,C	#load output array 
	mv	s3,x0	# i
	mv	s4,x0	# j
	mv	s5,x0	# k
	       
loop2hell:
# Algo goes here
# result[i][j] += m1[i][k] * m2[k][j]
# Fetch m1[i][k] and m2[k][j]
# Fetch result[i][j] into, mult m1 and m2, add to result and then store
# t0:  m1[i][k], t6: m2[k][j], t2: result[i][j]
#---------------------------------------------------------------------------------------
m1_fet:             
	mv      a0, s3       # i to arg0
	mv      a1, s5       # k to arg1
	addi    sp, sp, -4   # saves ra to stack
	sw      ra, 0(sp)
	call    Matrix        # gets the Matrix
	lw      ra, 0(sp)    # restores ra
	addi    sp, sp, 4
	add     a0, a0, s1   # adds index to appropriate LUT
	lw      t0, 0(a0)    # retrieves value at matrix A index
                    
m2_fet:             
	addi    sp, sp, -4   # saves ra to stack
	sw      ra, 0(sp)
	mv      a1, s4       # j to arg1
	mv      a0, s5       # k to arg0
	call    Matrix        # gets the Matrix
	lw      ra, 0(sp)    # restores ra
	addi    sp, sp, 4
	add     a0, a0, s2   # adds index to appropriate LUT
	lw      t6, 0(a0)    # retrieves value at matrix B index
	

m1_m2_mult:         
	addi    sp, sp, -4   # saves ra to stack
	sw      ra, 0(sp)
	mv      a0, t0       # move m1 into a0
	mv      a1, t6       # move m2 into a1
	call    Multi        # multiplies m1 * m2
	lw      ra, 0(sp)    # restores ra
	addi    sp, sp, 4
	mv	t0,a0		#t0 store sum

result_fet:         
	addi    sp, sp, -4   # saves ra to stack
	sw      ra, 0(sp)
	mv      a0, s3       # i to arg0
	mv      a1, s4       # j to arg1
	call    Matrix        # gets the Matrix
	add     a0, a0, s6   # adds index to appropriate LUT
	lw      t6, 0(a0)    # retrieves value at matrix index
	lw      ra, 0(sp)    # restores ra
	addi    sp, sp, 4

result_add:         
	add     t0, t0, t6   # puts product into t0
	

result_store:       
	sw      t0, 0(a0)    # stores t0 sum	
	j       loops_admin

loops_admin:
	mv	a7,a2
	addi	a7,a7,-1
k_admin:    
	addi    s5, s5, 1    # k++
	addi	t4,t4,1	
	bgt     s5, a7, j_admin # k < size
	j loop2hell
	

j_admin:
	mv      s5, x0
	addi	t5,t5,1
	addi    s4, s4, 1    # j++
	bgt     s4,a7,i_admin
	j loop2hell
	
i_admin:   
	
	mv      s4, x0 
	#addi	t6,t6,1 
	addi    s3, s3, 1    # i++
	bgt	s3,a7,restore
	j loop2hell

restore:	
	mv	s3,x0
	lw      s5, 20(sp)   # k
	lw      s4, 16(sp)   # j
	lw      s3, 12(sp)   # i
	lw      s2, 8(sp)    # load output array
	lw      s1, 4(sp)    # load array B
	lw      s6, 0(sp)    # load array A
	addi    sp, sp, 24

return:		
	ret
#===========================================================================================
# Subroutine 1: Matrix
# Retrieves and returns the index
# Assumes a0 = i (rows), a1 = j (columns), a2 = size of array, return index in a0
#-------------------------------------------------------------------------------------------
Matrix:
save2:		
	addi	sp,sp,-8
	sw	s0,0(sp)
	sw	s1,4(sp)
		
init2:		
	mv	s0,x0	
	slli	a1,a1,2			#shift i values to word size
	slli	s1,a2,2			#shift from byte-sized to wordsized array	
column:		
	beqz	a0,row			#branch when number of remaining cols = 0
	add	s0,s0,s1			#shift columns
	addi	a0,a0,-1		#increment loop
	j column
		
row:		
	add	a0,s0,a1		#add row to lut index
				
restore2:	
	mv	a1,x0
	lw	s1,4(sp)
	lw	s0,0(sp)
	addi	sp,sp,8
	ret


#-------------------------------------------------------------------------------------

#Subroutine 2
#Func Multi <a0,a1>
#Return result in a0 

#Func (a0, a1):
#int x=0, sum=0;
#for (x<=a0):
#	sum = sum + a1
#	x++
#return sum //in a0
#-------------------------------------------------------------------------------------
#Inputs: a0 = index 0, a1 = index 1

Multi:
init3:	
	addi	sp,	sp,	-8	#make space (stack 4)
	sw	s0,	0(sp)		#save x on stack
	sw	s1,	4(sp)		#save sum on stack
	
	mv	s0,	x0		#intialize x (iterator)
	mv	s1,	x0		#initialize sums
	beq	a1,	x0,	zer
loop3:	
	beq	s0,	a0,	done3	#if x>a0: start return sequence
	add	s1,	s1,	a1	#add sum=sum+a1
	addi	s0,	s0,	1	#increment x++
	j	loop3			#repeat!

done3:
	mv	a1,	x0		#reset a1	
	mv	a0,	s1		#overwrite a0 to be the new sum
	lw	s1,	4(sp)		#restore s1
	lw	s0,	0(sp)		#restore s0
	addi 	sp,	sp,	8	#restore stack pointer
	ret				#finish

zer:	mv	a0,	x0		#output zero
	mv	a1,	x0		#reset a1	
	lw	s1,	4(sp)		#restore s1
	lw	s0,	0(sp)		#restore s0
	addi 	sp,	sp,	8	#restore stack pointer
	ret				#finish
	
done1:
	
