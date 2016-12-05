import glob, os
import re
import copy
from shutil import copyfile, move
"""##################################################################
directory: ./*png, txt
	./data_train
	./data_valid
"""###################################################################
# imglabels = {"left":[],"right":[]}
leftimg_labels  = []
rightimg_labels = []
scanlabels = []
scanlabels_chosen = []
# read all the left image into leftimg_labels
def img_label_left(leftimg_labels):
	# left image label:
	for j,leftimg in enumerate(sorted(glob.iglob('*left.png'))):
		title, ext = os.path.splitext(os.path.basename(leftimg))	    
		num_img = re.findall("\d+\.\d+",title)[0]
		pos_img = re.sub(num_img, '', title) # left or right
		num = long(float(num_img)*10)
		new_name = str(num) + pos_img + ext
		print "Left image:",num,j
		leftimg_labels.append(num)
		os.rename(leftimg,os.path.join('./',new_name))
	
	return leftimg_labels
# read all the left image into leftimg_labels
def img_label_right(rightimg_labels):
	# right image label:
	for j,rightimg in enumerate(sorted(glob.iglob('*right.png'))):
		title, ext = os.path.splitext(os.path.basename(rightimg))	    
		num_img = re.findall("\d+\.\d+",title)[0]
		pos_img = re.sub(num_img, '', title) # left or right
		num = long(float(num_img)*10)
		new_name = str(num) + pos_img + ext
		print "right image", num,j
		rightimg_labels.append(num)
		os.rename(rightimg,os.path.join('./',new_name))

	return rightimg_labels
# read all the scan information into scanlabels
def scan_label(scanlabels):
	for j,txtname in enumerate(sorted(glob.iglob('*.txt'))):
		title,ext = os.path.splitext(os.path.basename(txtname))
		# print re.findall("\d+\.\d+",title)
		num_txt = re.findall("\d+\.\d+",title)[0]
		num = long(float(num_txt)*10)
		new_name = str(num)+ext
		print "Scanning txt", num,j
		scanlabels.append(num)
		os.rename(txtname,os.path.join('./',new_name))
	
	return scanlabels
# relabel scan image
def scan_label_multi(scanlabels,leftimg_labels, scanlabels_chosen):
	for i, fig_num in enumerate(leftimg_labels):
		fig_scannum = min(scanlabels,key=lambda x: abs(x-fig_num)) # find scanlabels
		scan_fig_dst = str(fig_num)+'.txt'
		scan_fig_src = str(fig_scannum) + '.txt'
		if not os.path.isfile(scan_fig_dst): 
			copyfile(scan_fig_src, scan_fig_dst)
		scanlabels_chosen.append(fig_scannum)

	return scanlabels_chosen



leftimg_labels  = img_label_left(leftimg_labels)
rightimg_labels = img_label_right(rightimg_labels)	
scanlabels = scan_label(scanlabels)
scanlabels_chosen = scan_label_multi(scanlabels,leftimg_labels, scanlabels_chosen)

# # change the NAN value into 10
# for j,txtname in enumerate(sorted(glob.iglob('*.txt'))):
# 	scan_data = np.loadtxt(txtname,delimiter=',')
# 	# scan_data = scan_data[120:240]
# 	scan_data[scan_data==np.inf] = 10
# 	np.savetxt(txtname, scan_data[None,:], delimiter=',',fmt='%.10f',newline=' ')
# 	print txtname


# save training data:
path = './data_train/'
for i, num in enumerate(leftimg_labels):
	if num in rightimg_labels and num in scanlabels_chosen:
		scan_src = './'+str(num)+'.txt'
		leftimg_src = './'+str(num)+'left.png'
		rightimg_src = './'+str(num)+'right.png'
		scan_dest = path + str(num)+'.txt'
		leftimg_dest = path + str(num)+'left.png'
		rightimg_dest = path + str(num)+'right.png'
		move(scan_src, scan_dest)
		move(leftimg_src, leftimg_dest)
		move(rightimg_src, rightimg_dest)
		print num, "saved"


# double check on training data
scan_num = len(glob.glob1('./data_train',"*.txt"))
left_num = len(glob.glob1('./data_train',"*left.png"))
right_num = len(glob.glob1('./data_train',"*right.png"))

if scan_num==left_num and scan_num==right_num:
	print "Lable Pair success!"
	print scan_num," training pairs are obtained!"




