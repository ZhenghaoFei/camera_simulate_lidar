import glob, os
import re
"""##################################################################
This is only for unpack zed camera rosbag file
The name of ros file should be sensing time
Tha dual camera may exist redundant left or right pictures in the end, 
but no redundant in the front
"""###################################################################
imglables = {"left":[],"right":[]}
scanlables = []
def img_lable(imglables,scanlables):
	pair_check = 0
	for i,figname in enumerate(sorted(glob.iglob('*.png'))):
	    title, ext = os.path.splitext(os.path.basename(figname))
	    num_img = re.findall("\d+\.\d+",title)[0]
	    num = long(float(num_img)*10)
	    #num choose in scalable the one that nearest to the num
	    num_updated = min(scanlables,key=lambda x: abs(x-num))

	    if i%2 == 0: 
	    	num_previous = num 
	    	figname_pre = figname
	    # condition for selected: nearest to scaning time and exist pairs
	    if num_updated - num < 3 and (num == num_previous):
	    	# num = min(scanlables,key=lambda x: abs(x-num))
	    	pos_img = re.sub(num_img, '', title) # left or right
	    	new_name = str(num_updated)+pos_img+ext
	    	imglables[pos_img].append(num)
	    	print num,pos_img,i
	    	os.rename(figname, os.path.join('./', new_name))
	    else:
	    	os.remove(figname_pre)
	return imglables

def scan_lable(scanlables):
	for j,txtname in enumerate(sorted(glob.iglob('*.txt'))):
		title,ext = os.path.splitext(os.path.basename(txtname))
		num_txt = re.findall("\d+\.\d+",title)[0]
		num = long(float(num_txt)*10)

		new_name = str(num)+ext
		print num,j
		scanlables.append(num)
		os.rename(txtname,os.path.join('./',new_name))
	return scanlables

# print len(imglables)
scanlables = scan_lable(scanlables)
imglables = img_lable(imglables,scanlables)	

# count number of files in the unpack dir
scan_num = len(glob.glob1('.',"*.txt"))
left_num = len(glob.glob1('.',"*left.png"))
right_num = len(glob.glob1('.',"*right.png"))

if scan_num==left_num and scan_num==right_num:
	print "Lable Pair success!"
	print scan_num," pairs are obtained!"