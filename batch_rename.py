
import os
import shutil

source_path = "/home/ceri/4dage_ceri_program/Specular_Reflector_Filtering/panorama2normal/data/0A00009/images/"      
path = "/home/ceri/4dage_ceri_program/Specular_Reflector_Filtering/panorama2normal/data/"
#获取该目录下所有文件，存入列表中
f=os.listdir(source_path)

n=1524
for each_jpg in f:
    if each_jpg.endswith(".jpg"):
        #设置旧文件名（就是路径+文件名）
        oldname = source_path+each_jpg

        #设置新文件名
        newname = path+str(n+1)+each_jpg

        #用os模块中的rename方法对文件改名
        shutil.copyfile(oldname,newname)
        print(oldname,'======>',newname)

        n+=1
print(n)