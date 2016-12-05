import glob, os
import re
import copy
from shutil import copyfile, move
"""##################################################################
directory: This file should be in data_train
	./data_train
	../data_valid
"""###################################################################
# imglabels = {"left":[],"right":[]}
leftimg_labels  = []
rightimg_labels = []
scanlabels = []

# read all the left image into leftimg_labels
def img_label_left(leftimg_labels):
	# left image label:
	for j,leftimg in enumerate(sorted(glob.iglob('*left.png'))):
		title, ext = os.path.splitext(os.path.basename(leftimg))	    
		num_img = re.findall("\d+",title)[0]
		print "Left image:",num_img,j
		leftimg_labels.append(num_img)
	
	return leftimg_labels
# read all the left image into leftimg_labels
def img_label_right(rightimg_labels):
	# right image label:
	for j,rightimg in enumerate(sorted(glob.iglob('*right.png'))):
		title, ext = os.path.splitext(os.path.basename(rightimg))	    
		num_img = re.findall("\d+",title)[0]
		print "right image", num_img,j
		rightimg_labels.append(num_img)

	return rightimg_labels
# read all the scan information into scanlabels
def scan_label(scanlabels):
	for j,txtname in enumerate(sorted(glob.iglob('*.txt'))):
		title,ext = os.path.splitext(os.path.basename(txtname))
		num_txt = re.findall("\d+",title)[0]
		print "Scanning txt", num_txt,j
		scanlabels.append(num_txt)
	return scanlabels

leftimg_labels  = img_label_left(leftimg_labels)
rightimg_labels = img_label_right(rightimg_labels)	
scanlabels = scan_label(scanlabels)

# split files from train to validate
path = '../data_valid/'
valid_length = int(len(leftimg_labels)*0.1)
for i,num in enumerate(leftimg_labels):
	if i < valid_length:
		scan_src = './'+str(num)+'.txt'
		leftimg_src = './'+str(num)+'left.png'
		rightimg_src = './'+str(num)+'right.png'
		scan_dest = path + scan_src
		leftimg_dest = path + leftimg_src
		rightimg_dest = path + rightimg_src
		move(scan_src, scan_dest)
		move(leftimg_src, leftimg_dest)
		move(rightimg_src, rightimg_dest)
		leftimg_labels.pop(i)
		rightimg_labels.pop(i)
		scanlabels.pop(i)
		print num, "saved"
	else:
		break

# double check on validation data
scan_num_valid  = len(glob.glob1('../data_valid',"*.txt"))
left_num_valid  = len(glob.glob1('../data_valid',"*left.png"))
right_num_valid = len(glob.glob1('../data_valid',"*right.png"))

if scan_num_valid==left_num_valid and scan_num_valid==right_num_valid:
	print "Lable Pair success!"
	print scan_num_valid," valid pairs are obtained!"