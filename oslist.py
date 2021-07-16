import os
import cv2

filePath = '/home/ceri/4dage_ceri_program/Specular_Reflector_Filtering/panoramic2normal/'
p = os.listdir(filePath)
textpath = p[0]
#for each_p in p:
#    img_path = os.path.join(filePath,each_p)
    
# textpath =+ filePath
print(type(textpath))
print("%s" % '"' + filePath + textpath +'"')
Path = filePath + textpath
print(type(Path))
image = cv2.imread(Path)
for file in p:
    if file.endswith('.jpg'):
        print(type(file))