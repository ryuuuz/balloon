import serial
import serial.tools.list_ports
import struct
from PIL import Image
import io
import cv2
import numpy as np

# 列出所有可用的串口设备
def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    for i, port in enumerate(ports):
        print(f"{i}: {port.device} - {port.description}")
    return ports

# 选择串口
def select_serial_port():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports available.")
        return None
    selected_index = int(input("Please select a serial port by index: "))
    if 0 <= selected_index < len(ports):
        return ports[selected_index].device
    else:
        print("Invalid selection.")
        return None

# 定义帧标记
FRAME_START = 0xAA
FRAME_END = 0x55

def receive_image_segments(ser):
    total_segments = None
    current_segment = 0
    rows, cols = None, None
    jpeg_data_list = []

    while True:
        # 读取一个字节的数据
        data = ser.read(1)
        if not data:
            continue

        byte = data[0]
        if byte == FRAME_START:
            # 读取分段号，总分段数、行数、列数和JPEG大小信息
            segment_info = ser.read(6)
            if len(segment_info) < 6:
                continue

            segment_number = segment_info[0]
            total_segments = segment_info[1]
            rows = segment_info[2]  # 解析行数
            cols = segment_info[3]  # 解析列数
            jpeg_size = struct.unpack(">H", segment_info[4:6])[0]  # 大端模式读取JPEG大小

            # 读取JPEG数据
            jpeg_segment = ser.read(jpeg_size)
            if len(jpeg_segment) == jpeg_size:
                jpeg_data_list.append(jpeg_segment)
                current_segment += 1

                print(f"Received segment {segment_number}/{total_segments}, Size: {jpeg_size} bytes")

            # 读取帧尾
            frame_end = ser.read(1)
            if frame_end and frame_end[0] == FRAME_END:
                print("Frame end detected.")
            else:
                print("Error: Frame end not detected.")

            # 如果所有段都收到了，退出循环
            if current_segment == total_segments:
                print("All segments received.")
                break

    return jpeg_data_list, rows, cols

def display_image_in_window(jpeg_data_list, rows, cols, border_thickness=1, border_color=(255, 255, 255)):
    # 计算分割图像的布局
    grid_size = (rows, cols)

    # 假设所有图像分段大小相同，使用第一个图像的大小来创建画布
    first_image = Image.open(io.BytesIO(jpeg_data_list[0]))
    image_np = np.array(first_image)
    h, w, c = image_np.shape

    # 创建一个大的空画布来容纳所有图像段
    canvas_height = grid_size[0] * h
    canvas_width = grid_size[1] * w
    canvas = np.zeros((canvas_height, canvas_width, c), dtype=np.uint8)

    # 将每个图像段放入画布的正确位置，并在图像上绘制边框
    for idx, jpeg_data in enumerate(jpeg_data_list):
        image = Image.open(io.BytesIO(jpeg_data))
        image_np = np.array(image)

        # 绘制边框
        cv2.rectangle(image_np, (0, 0), (w - 1, h - 1), border_color, border_thickness)

        # 计算图像段应该放置的位置
        row = idx // grid_size[1]
        col = idx % grid_size[1]
        y_offset = row * h
        x_offset = col * w

        # 将图像段放置在画布上
        canvas[y_offset:y_offset + h, x_offset:x_offset + w] = image_np

    # 使用OpenCV显示完整的图像
    cv2.imshow("Image Grid with Drawn Borders", canvas)
    cv2.waitKey(1)  # 短暂停留以刷新窗口

if __name__ == '__main__':
    # serial_port = select_serial_port()
    serial_port = "COM9"
    if serial_port:
        baud_rate = 115200
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Using serial port: {serial_port}")

        # 打开窗口
        while True:
            print("Waiting for image data...")
            jpeg_data_list, rows, cols = receive_image_segments(ser)
            if jpeg_data_list:
                print("Displaying image...")
                display_image_in_window(jpeg_data_list, rows, cols)
            else:
                print("No image data received.")
    else:
        print("No valid serial port selected.")
