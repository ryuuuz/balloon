import os
import io
import base64
from flask import Flask, render_template, send_file, request

app = Flask(__name__)

# 当前脚本所在的目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 存储图像文件的路径（从scripts目录跳到balloon目录）
IMAGE_DIR = os.path.join(SCRIPT_DIR, 'images')

# 确保图像存储目录存在
if not os.path.exists(IMAGE_DIR):
    os.makedirs(IMAGE_DIR)

# 用于拼接图像的缓冲区
image_buffer = bytearray()
last_packet_idx = -1  # 用于跟踪最后接收到的包序号

# 设置最大保存图片的数量
MAX_IMAGES = 5

@app.route('/')
def index():
    # 获取最新的几个图像文件
    image_files = sorted(os.listdir(IMAGE_DIR), reverse=True)[:MAX_IMAGES]
    return render_template('index.html', image_files=image_files)

@app.route('/image', methods=['POST'])
def receive_image():
    global image_buffer
    try:
        # 获取传入的JSON数据
        data = request.get_json()

        # 提取JPEG数据
        base64_data = data.get("data", "")

        # 解析base64编码的数据
        jpeg_data = base64.b64decode(base64_data)
        
        # 获取 packet_idx 和 is_last_packet
        packet_idx = int(jpeg_data[0])  # 第一个字节转为整数
        is_last_packet = int(jpeg_data[1])  # 第二个字节转为整数
        print(f"Packet index: {packet_idx}, Last packet: {is_last_packet}")

        if packet_idx == -1 or len(jpeg_data) == 0:
            return "Invalid data", 400

        # 将当前包的数据追加到缓冲区
        image_buffer.extend(jpeg_data[2:])

        # 如果是最后一个包，则保存图像
        if is_last_packet:
            # 确定图像文件的名称
            image_index = len(os.listdir(IMAGE_DIR)) + 1
            image_file_path = os.path.join(IMAGE_DIR, f'image_{image_index}.jpg')
            
            # 保存图像
            with open(image_file_path, 'wb') as f:
                f.write(image_buffer)
            print(f"Image saved to {image_file_path}")

            # 清空缓存，准备接收下一个图像
            image_buffer = bytearray()

            # 如果文件超过最大保存数量，删除最旧的文件
            if len(os.listdir(IMAGE_DIR)) > MAX_IMAGES:
                oldest_image = min(os.listdir(IMAGE_DIR), key=lambda f: os.path.getctime(os.path.join(IMAGE_DIR, f)))
                os.remove(os.path.join(IMAGE_DIR, oldest_image))
            
            return "Image received and saved successfully", 200

        return "Packet received", 200

    except Exception as e:
        print(f"Error processing image: {e}")
        return "Error processing data", 500

@app.route('/image/latest')
def show_latest_image():
    image_file_path = os.path.join(IMAGE_DIR, 'latest_image.jpg')

    if os.path.exists(image_file_path):
        return send_file(image_file_path, mimetype='image/jpeg')
    else:
        return "No image available", 404

@app.route('/image/<int:image_id>')
def show_image(image_id):
    image_file_path = os.path.join(IMAGE_DIR, f'image_{image_id}.jpg')

    if os.path.exists(image_file_path):
        return send_file(image_file_path, mimetype='image/jpeg')
    else:
        return "Image not found", 404

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8800)
