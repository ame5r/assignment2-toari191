#!/usr/bin/env python

import os

while True:
  option = input('please choose an option:\n1. Move forward\n2. Turn around\n3. Distance to colored object\n4. Find colored object \n')
  if option == 1:#move forward
    os.system('python option1.py')
  elif option == 2:#turn around
    os.system('python option2.py')
  elif option == 3:#distance to colored object
    os.system('python option3.py')
  elif option == 4:#find colored object
	os.system('python option4.py')
  else:
    print('illegal input: {}'.format(option))
	
 
