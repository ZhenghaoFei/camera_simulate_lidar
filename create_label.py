import glob, os
import re
import copy
"""##################################################################
This is only for unpack zed camera rosbag file
The name of ros file should be time of sensoring
Tha dual camera may exist redundant left or right pictures in the end, 
but no redundant in the front; 
The lidar sensoring slower than camera
"""###################################################################
imglables = {"left":[],"right":[]}
imglables_update = imglables = {"left":[],"right":[]}
scanlables = []
def img_lable(imglables,scanlables):
	pair_check = 0
	n=0
	for i,figname in enumerate(sorted(glob.iglob('*.png'))):
		title, ext = os.path.splitext(os.path.basename(figname))	    
		num_img = re.findall("\d+\.\d+",title)[0]
		pos_img = re.sub(num_img, '', title) # left or right
		num = long(float(num_img)*10)
	    #num choose in scalable the one that nearest to the num
		num_updated = min(scanlables,key=lambda x: abs(x-num))
		if i%2 == 0: 
			num_previous = num 
			figname_pre  = figname
			pos_img_pre = pos_img

		# condition for selected: nearest to scaning time and exist pairs
		if num_updated - num < 3 and (num == num_previous):
			# num = min(scanlables,key=lambda x: abs(x-num))
			new_name = str(num_updated)+pos_img+ext
			# if num_updated in 
			imglables[pos_img].append(num_updated)
			print num_updated
			os.rename(figname, os.path.join('./', new_name))
		else:
			os.remove(figname_pre)
		# else: 
		# 	os.remove(new_name)   # delete first redundant one
	for name in imglables["left"]:
		if name not in imglables_update["left"]:
			imglables_update["left"].append(name)
			print name
	for name in imglables["right"]:
		if name not in imglables_update["right"]:
			imglables_update["right"].append(name)
			print name

	imglables["left"]=copy.deepcopy(sorted(list(set(imglables["left"])))) # delete duplicated image name just like copy and replace
	imglables["right"]=copy.deepcopy(sorted(list(set(imglables["right"]))))
	print "scan" , len(scanlables)
	print "left image",len(imglables["left"])
	print "right image",len(imglables["right"])
	scanlables, imglables_new = check_tris(imglables_update,scanlables)
	return scanlables, imglables_new

def scan_lable(scanlables):
	for j,txtname in enumerate(sorted(glob.iglob('*.txt'))):
		title,ext = os.path.splitext(os.path.basename(txtname))
		# print re.findall("\d+\.\d+",title)
		num_txt = re.findall("\d+\.\d+",title)[0]
		num = long(float(num_txt)*10)
		new_name = str(num)+ext
		print num,j
		scanlables.append(num)
		os.rename(txtname,os.path.join('./',new_name))
	return scanlables

def check_tris(imglables,scanlables):
	# num_pairs = max(len(imglables["left"]),len(scanlables))
	for i in range(len(imglables["left"])):
		if scanlables[i]==imglables["left"][i] and scanlables[i]==imglables["right"][i]:
			continue
		else:
			print scanlables[i]
			print imglables["left"][i], imglables["right"][i]
			path = "./"
			scanname = path + str(scanlables[i])+".txt"
		 	if os.path.exists(scanname):
		 		os.remove(scanname)
		 		scanlables.pop(i)
	return scanlables, imglables



# print len(imglables)
scanlables = scan_lable(scanlables)
scanlables, imglables = img_lable(imglables,scanlables)	

# count number of files in the unpack dir
scan_num = len(glob.glob1('.',"*.txt"))
left_num = len(glob.glob1('.',"*left.png"))
right_num = len(glob.glob1('.',"*right.png"))

if scan_num==left_num and scan_num==right_num:
	print "Lable Pair success!"
	print scan_num," pairs are obtained!"
