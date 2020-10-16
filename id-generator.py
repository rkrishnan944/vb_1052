#!/usr/bin/env python

import random 

List = []

for i in range(1,9):
	#run again if special characters appear in your string.
	num = random.randint(65, 122)
	List.append(chr(num))
String = "".join((List))
print(String)

# Generated String : lteeBmeZ


        