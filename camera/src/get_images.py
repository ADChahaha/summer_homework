import cv2
import numpy as np

# 设置角点精准化迭代过程的终止条件
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# 棋盘格模板规格
len = 40  # 黑白格长度
w = 4
h = 6

# 世界坐标系中的棋盘格点，去掉Z坐标，记为二维矩阵
world_point = np.zeros((w * h, 3), np.float32)
world_point[:, :2] = np.mgrid[0:w * len:len, 0:h * len:len].T.reshape(-1, 2)

# 储存棋盘格角点的世界坐标和图像坐标对
world_points = []  # 在世界坐标系中的三维点
imgpoints = []    # 在图像平面的二维点

# 遍历加载已经保存的图像，并找到棋盘格角点
for i in range(1, 101):  # 假设你保存了100张图像
    filename = f'/home/sora/桌面/camera/photo_{i}.jpg'  # 图像路径
    img = cv2.imread(filename)  # 读取图像
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转换为灰度图像

    # 找到棋盘格角点
    ret, corners = cv2.findChessboardCorners(gray, (w, h), None)

    if ret:
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        world_points.append(world_point)
        imgpoints.append(corners)

        # 将角点在图像上显示（可选）
        cv2.drawChessboardCorners(img, (w, h), corners, ret)
        cv2.imshow('findCorners', img)
        cv2.waitKey(1)

        # 保存处理后的图像（可选）
        cv2.imwrite(f'./processed_image/img{i}_processed.jpg', img)

        print(f'第 {i} 张图像处理完成')

cv2.destroyAllWindows()

# 继续进行相机标定和校正的步骤
# 求解摄像机的内在参数和外在参数
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(world_points, imgpoints, gray.shape[::-1], None, None)

print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)
print("my ret is",ret)
# 接下来的代码处理图像校正等步骤
file_path = 'mtx.txt'
with open(file_path, 'w') as f:
    # 将矩阵转换为字符串并写入文件
    for row in mtx:
        f.write(' '.join(map(str, row)) + '\n')

print(f"矩阵已写入到文件 '{file_path}' 中。")
file_path = 'dist.txt'
with open(file_path, 'w') as f:
    # 将矩阵转换为字符串并写入文件
    for row in dist:
        f.write(' '.join(map(str, row)) + '\n')

print(f"矩阵已写入到文件 '{file_path}' 中。")
file_path = 'ret.txt'
with open(file_path, 'w') as f:
    # 将矩阵转换为字符串并写入文件
    f.write(str(ret))

print(f"矩阵已写入到文件 '{file_path}' 中。")