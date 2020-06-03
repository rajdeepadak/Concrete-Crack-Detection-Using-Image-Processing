import cv2 as cv2
import numpy as np
import cv2
import os

folder = "C:\\Users\\rajde\\Desktop\\Sanki ka code\\folder"


def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        img = cv2.imread(os.path.join(folder, filename))
        if img is not None:
            images.append(img)
    return images


path = "C:\\Users\\rajde\\Desktop\\Sanki ka code\\folder\\BRODA.jpg"
img = cv2.imread(path, 1)
# cv2.imshow("crack", img)
# cv2.waitKey(0)
img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

print(type(img))
print(img.dtype)
print(img.shape)
print(img.ndim)
print(img.size)

p = 0
q = 0
s = 0

img = cv2.resize(img, (320, 480))

ret1, th1 = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)
cv2.imshow('Binary', th1)

canny = cv2.Canny(th1, 190, 190)
cv2.imshow('Canny', canny)

kernel = np.ones((5, 5), np.uint8)

dilation = cv2.dilate(canny, kernel, iterations=1)
cv2.imshow('Dilation', dilation)

p = 0
q = 0

lines = cv2.HoughLinesP(dilation, 1, 1 * np.pi / 180, 20, 15, 10)
print(type(lines))

for x in range(0, len(lines)):
    for x1, y1, x2, y2 in lines[x]:
        print(x1, y1, x2, y2)
        m = y2 - y1
        print(m)
        if x2 - x1 > 50 and y2 - y1 < 10:
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            p = 1
            print(p)
        if y2 - y1 > 50 and x2 - x1 < 10:
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
            q = 2
            print(q)
        if (y2 - y1) ^ 2 + ((x2 - x1) ^ 2) > 100:
            print("DIAGONAL CRACK")
            s = 3

        else:
            print("no line")

if p == 1 and q == 2:
    print("MAJOR CRACK FOUND")
elif s == 3 and (p == 0 and q == 0):
    print("DIAGONAL CRACK VERIFIED")
else:
    print("NO MAJOR CRACK FOUND")

cv2.imshow('hough', img)
cv2.waitKey(0)
