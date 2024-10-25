import serial
import cv2
import numpy as np
import time

def receive_image():
    START_MARKER = b'\xaa'  # 包开始标记
    END_MARKER = b'\x55'    # 包结束标记
    PACKAGE_SIZE = 200      # 每包数据大小
    JPEG_START = b'\xFF\xD8'
    JPEG_END = b'\xFF\xD9'

    # 设置串口参数
    ser = serial.Serial('COM9', 115200, timeout=0.5)

    image_data = bytearray()  # 存储完整图像数据
    receiving = False         # 用于标识是否在接收图像

    while True:
        byte = ser.read(1)
        if not byte:
            continue  # 没有数据则等待

        # 检测到开始标记后开始接收
        if byte == START_MARKER:
            receiving = True
            packet_data = bytearray()
        elif receiving and byte == END_MARKER:
            # 包结束，将数据加入缓冲区
            image_data.extend(packet_data)

            # 检查是否接收到完整 JPEG 图像
            if JPEG_START in image_data and JPEG_END in image_data:
                start_index = image_data.find(JPEG_START)
                end_index = image_data.find(JPEG_END) + 2  # 包括结束标记
                complete_image = image_data[start_index:end_index]
                
                # 尝试解码图像
                nparr = np.frombuffer(complete_image, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                if img is not None:
                    cv2.imshow('Received Image', img)
                    cv2.waitKey(1)
                    print("Image displayed successfully")
                else:
                    print("Failed to decode image")

                # 清空缓冲区并重置接收状态
                image_data = bytearray()
                receiving = False  # 等待接收下一张图片
        elif receiving:
            packet_data.extend(byte)
            if len(packet_data) > PACKAGE_SIZE:
                print("Warning: Packet larger than expected size")
                receiving = False  # 重置，以便从下一个包重新开始

if __name__ == "__main__":
    receive_image()
