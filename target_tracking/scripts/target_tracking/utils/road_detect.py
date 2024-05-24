import cv2
import numpy as np

# 读取图像
image = cv2.imread("H:\\source code\\me\\3.png")
# 将图像转换为HSV颜色空间
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
# 定义道路的颜色范围（这里以黑色为例）
lower_black = np.array([18,18,44])
upper_black = np.array([30,37,63]) 
# 根据颜色范围进行阈值化
mask = cv2.inRange(hsv, lower_black, upper_black)

# 对阈值图像进行形态学操作，以进一步处理图像
kernel = np.ones((5, 5), np.uint8)
mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

# 显示结果
cv2.imshow("1", mask)
cv2.imshow("2", image)
cv2.waitKey(0)